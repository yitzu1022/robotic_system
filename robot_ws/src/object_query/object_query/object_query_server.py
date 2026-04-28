import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point
from std_msgs.msg import String
from object_query_interfaces.srv import ObjectQuery
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np
import json
import os
import random
from scipy.spatial.transform import Rotation as R
from collections import defaultdict
import yaml

class ObjectQueryServer(Node):
    def __init__(self):
        super().__init__('object_query_server')

        # === Declare parameters ===
        # self.declare_parameter('3dmap_path', 'data/Util/Final_GS.npz') old scene map
        self.declare_parameter('3dmap_path', 'data/lab/accumulated_gaussians.npz')
        # self.declare_parameter('map_path', 'data/Util/Final_SEM_GS_converted.npz') old semantic map
        self.declare_parameter('map_path', 'data/lab/semantic_pcd_accumulated_gaussians.npz')
        # self.declare_parameter('semantic_path', 'data/Util/Final_SEM_GS_converted_meta.json') old semantic labels
        self.declare_parameter('semantic_path', 'data/lab/semantic_pcd_accumulated_gaussians_meta.json')
        self.declare_parameter('instance_path', 'data/lab/accumulated_gaussians_instance_semantic_info.json')
        self.declare_parameter('alignment_path', 'data/Util/alignment.yaml')
        self.declare_parameter('auto_align', False) 

        map_3d_path = self.get_parameter('3dmap_path').get_parameter_value().string_value
        map_path = self.get_parameter('map_path').get_parameter_value().string_value
        sem_path = self.get_parameter('semantic_path').get_parameter_value().string_value
        instance_path = self.get_parameter('instance_path').get_parameter_value().string_value
        alignment_path = self.get_parameter('alignment_path').get_parameter_value().string_value
        auto_align = self.get_parameter('auto_align').get_parameter_value().bool_value

        # === ROS Entities ===
        self.srv = self.create_service(ObjectQuery, '/object_query', self.handle_query)
        self.pub_objects = self.create_publisher(String, '/object_list', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/semantic_map_markers', 10)
        self.pcl_pub = self.create_publisher(PointCloud2, '/map_pointcloud', 10)
        preview_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.candidate_pub = self.create_publisher(
            String,
            '/semantic_preview/object_candidates',
            preview_qos,
        )

        # === Data container ===
        self.object_db = defaultdict(list) 
        self.object_instance_ids = defaultdict(list)
        
        self.map_points = None
        self.map_colors = None
        self.map_3d_points = None
        self.map_3d_colors = None
        self.alignment = None
        self.load_alignment(alignment_path)

        # === Load Data ===
        if os.path.exists(instance_path):
            self.load_instance_map(instance_path, map_3d_path)
        else:
            self.get_logger().warn(f'⚠️ instance_path not found ({instance_path}), falling back to semantic map.')
            self.load_semantic_map(map_path, sem_path, auto_align, map_3d_path)

        # === Publish at startup ===
        self.publish_object_list()
        self.publish_point_cloud()
        # Note: We do NOT publish markers here anymore. 
        # Markers appear only after a query.

        # === Timer for continuous point cloud publishing ===
        self.pcl_timer = self.create_timer(1.0, self.publish_point_cloud)  # Publish every 1 second

        self.get_logger().info(f'✅ ObjectQuery service ready. Auto-align & Center: {auto_align}')

    # --------------------------------------------------------------
    def load_alignment(self, alignment_path: str):
        if not os.path.exists(alignment_path):
            self.get_logger().warn(f'⚠️ alignment file not found: {alignment_path}')
            return

        try:
            with open(alignment_path, 'r') as f:
                data = yaml.safe_load(f)

            self.alignment = {
                'mu': np.array(
                    [
                        data['plane_fit']['mu']['x'],
                        data['plane_fit']['mu']['y'],
                        data['plane_fit']['mu']['z'],
                    ],
                    dtype=float,
                ),
                'e1': np.array(data['plane_fit']['basis_e1'], dtype=float),
                'e2': np.array(data['plane_fit']['basis_e2'], dtype=float),
                'normal': np.array(data['plane_fit']['normal_n'], dtype=float),
                'scale': float(data['sim2']['s']),
                'rot': np.array(data['sim2']['R'], dtype=float),
                'trans': np.array([data['sim2']['t']['x'], data['sim2']['t']['y']], dtype=float),
            }
            self.get_logger().info(f'Loaded 3D->map alignment from {alignment_path}')
        except Exception as e:
            self.alignment = None
            self.get_logger().warn(f'⚠️ Failed to load alignment file {alignment_path}: {e}')

    def transform_point_to_map(self, point_xyz):
        if self.alignment is None:
            return float(point_xyz[0]), float(point_xyz[1]), float(point_xyz[2])

        point = np.array(point_xyz, dtype=float)
        delta = point - self.alignment['mu']
        uv = np.array(
            [
                delta @ self.alignment['e1'],
                delta @ self.alignment['e2'],
            ],
            dtype=float,
        )
        height = float(delta @ self.alignment['normal'])
        xy = self.alignment['scale'] * (self.alignment['rot'] @ uv) + self.alignment['trans']
        z = self.alignment['scale'] * height
        return float(xy[0]), float(xy[1]), float(z)

    def publish_candidate_options(self, name: str, instances, instance_ids):
        payload = {
            'event': 'active',
            'name': name,
            'options': [],
        }
        for i, pos in enumerate(instances):
            map_x, map_y, map_z = self.transform_point_to_map(pos)
            payload['options'].append(
                {
                    'index': i,
                    'instance_id': instance_ids[i] if i < len(instance_ids) else '',
                    'raw_x': float(pos[0]),
                    'raw_y': float(pos[1]),
                    'raw_z': float(pos[2]),
                    'map_x': map_x,
                    'map_y': map_y,
                    'map_z': map_z,
                }
            )

        msg = String()
        msg.data = json.dumps(payload)
        self.candidate_pub.publish(msg)

    def clear_candidate_options(self):
        msg = String()
        msg.data = json.dumps({'event': 'clear'})
        self.candidate_pub.publish(msg)

    # --------------------------------------------------------------
    def load_instance_map(self, instance_path: str, map_3d_path: str):
        """Load object positions from instance JSON (centroid per instance). Also loads 3D point cloud."""
        try:
            with open(instance_path, 'r') as f:
                inst_data = json.load(f)

            instances = inst_data.get('instances', {})
            if not instances:
                self.get_logger().error(f'❌ No "instances" key found in {instance_path}')
                return

            self.object_db.clear()
            self.object_instance_ids.clear()
            for inst_id, inst in instances.items():
                name = inst.get('semantic_name', 'unknown').lower()
                centroid = inst.get('centroid', None)
                if centroid is None or len(centroid) < 3:
                    continue
                self.object_db[name].append(tuple(centroid))
                self.object_instance_ids[name].append(str(inst_id))

            self.get_logger().info(
                f'Loaded {len(instances)} instances {len(self.object_db)} categories from {instance_path}')

            # Load 3D point cloud for visualization
            if os.path.exists(map_3d_path):
                data_3d = np.load(map_3d_path)
                self.map_3d_points = data_3d['points']
                if 'colors' in data_3d:
                    c = data_3d['colors']
                    self.map_3d_colors = (c * 255).astype(np.uint8) if c.max() <= 1.0 else c.astype(np.uint8)
                else:
                    self.map_3d_colors = np.full((self.map_3d_points.shape[0], 3), 128, dtype=np.uint8)
            else:
                self.get_logger().warn(f'⚠️ 3D map not found: {map_3d_path}')

        except Exception as e:
            self.get_logger().error(f'❌ Failed to load instance map: {e}')
            import traceback
            traceback.print_exc()

    # --------------------------------------------------------------
    def load_semantic_map(self, map_path: str, sem_path: str, auto_align: bool, map_3d_path: str):
        """Load semantic map with robust JSON parsing for 'segments_info'."""
        if not os.path.exists(map_path):
            self.get_logger().error(f'Map file not found: {map_path}')
            return
        if not os.path.exists(sem_path):
            self.get_logger().error(f'Semantic JSON not found: {sem_path}')
            return
        if not os.path.exists(map_3d_path):
            self.get_logger().error(f'3D map file not found: {map_3d_path}')
            return

        try:
            # 1. Load NPZ Data
            data = np.load(map_path)
            data_3d = np.load(map_3d_path)
            # semantic
            if 'pts' in data: points = data['pts']
            elif 'means3D' in data: points = data['means3D']
            else:
                self.get_logger().error(f"❌ Could not find 'pts' or 'means3D' in {map_path}")
                return

            if 'pan' in data: semantic_ids = data['pan']
            elif 'semantic_ids' in data: semantic_ids = data['semantic_ids']
            else:
                self.get_logger().error(f"❌ Could not find 'pan' or 'semantic_ids' in {map_path}")
                return
            # 3D points and colors
            points_3d = data_3d['points']
            if 'colors' in data_3d:
                colors_3d = data_3d['colors']
                if colors_3d.max() <= 1.0:
                    colors_3d = (colors_3d * 255).astype(np.uint8)
                else:
                    colors_3d = colors_3d.astype(np.uint8)
            else:
                # Default gray color if no colors available
                colors_3d = np.full((points_3d.shape[0], 3), 128, dtype=np.uint8)
            # 2. Load JSON & Build Mapping
            with open(sem_path, 'r') as f:
                sem_json = json.load(f)
            
            segments = []
            if isinstance(sem_json, list): segments = sem_json
            elif isinstance(sem_json, dict):
                if 'segments_info' in sem_json: segments = sem_json['segments_info']
                elif 'segmentation' in sem_json: segments = sem_json['segmentation']
                elif 'segments' in sem_json: segments = sem_json['segments']
            
            # 3. Build Name Mapping
            id_to_name = {}
            for seg in segments:
                seg_id = seg.get('id', None)
                if seg_id is None: continue
                name = seg.get('category_name', seg.get('class', seg.get('label', 'unknown')))
                id_to_name[seg_id] = name

            if not id_to_name:
                self.get_logger().error("❌ JSON loaded but no categories found.")
                return

            # 4. Auto-Align (apply same transformation to both semantic and 3D maps)
            if auto_align:
                rot_matrix = self.compute_alignment_matrix(points, semantic_ids, id_to_name)
                if rot_matrix is not None:
                    points = points @ rot_matrix.T
                    points_3d = points_3d @ rot_matrix.T
                    self.get_logger().info(f"✅ Applied alignment to both semantic and 3D maps")
            

            self.map_points = points
            self.map_3d_points = points_3d
            self.map_3d_colors = colors_3d

            # 5. Generate Colors
            if 'rgb' in data:
                raw_rgb = data['rgb']
                if raw_rgb.max() <= 1.0: self.map_colors = (raw_rgb * 255).astype(np.uint8)
                else: self.map_colors = raw_rgb.astype(np.uint8)
            else:
                unique_ids = np.unique(semantic_ids)
                color_map = {}
                for uid in unique_ids:
                    color_map[uid] = (random.randint(50, 255), random.randint(50, 255), random.randint(50, 255))
                self.map_colors = np.zeros((self.map_points.shape[0], 3), dtype=np.uint8)
                for uid, color in color_map.items():
                    self.map_colors[semantic_ids == uid] = color

            # 6. Build Object Database
            self.object_db.clear()
            self.object_instance_ids.clear()
            for sid, name in id_to_name.items():
                mask = semantic_ids == sid
                if np.any(mask):
                    pts = self.map_points[mask]
                    min_pt = np.min(pts, axis=0)
                    max_pt = np.max(pts, axis=0)
                    centroid = (min_pt + max_pt) / 2.0
                    key = name.lower()
                    self.object_db[key].append(tuple(centroid.tolist()))
                    self.object_instance_ids[key].append(f'semantic_{sid}')

            self.get_logger().info(f'✅ Built object DB with {len(self.object_db)} categories.')

        except Exception as e:
            self.get_logger().error(f'❌ Failed to load semantic map: {e}')
            import traceback
            traceback.print_exc()

    # --------------------------------------------------------------
    def compute_alignment_matrix(self, points, semantic_ids, id_to_name):
        """Compute rotation matrix to align floor to XY plane. Returns None if alignment fails."""
        floor_ids = []
        floor_keywords = ['floor', 'ground', 'carpet', 'tile', 'wood']
        
        for sid, name in id_to_name.items():
            if any(k in name.lower() for k in floor_keywords):
                floor_ids.append(sid)
        
        if not floor_ids:
            self.get_logger().warn("⚠️ No floor objects found for alignment")
            return None

        mask = np.isin(semantic_ids, floor_ids)
        floor_pts = points[mask]
        
        if len(floor_pts) < 50:
            self.get_logger().warn("⚠️ Not enough floor points for reliable alignment")
            return None

        self.get_logger().info(f"📐 Aligning map using {len(floor_pts)} floor points...")

        floor_mean = np.mean(floor_pts, axis=0)
        centered_floor = floor_pts - floor_mean
        u, s, vh = np.linalg.svd(centered_floor, full_matrices=False)
        normal = -vh[2, :] 

        target_axis = np.array([0, 0, 1])
        rot_axis = np.cross(normal, target_axis)
        rot_sin = np.linalg.norm(rot_axis)
        rot_cos = np.dot(normal, target_axis)

        if rot_sin > 1e-6:
            rot_axis = rot_axis / rot_sin
            angle = np.arccos(np.clip(rot_cos, -1.0, 1.0))
            r = R.from_rotvec(rot_axis * angle)
            rot_matrix = r.as_matrix()
            self.get_logger().info(f"   ↪ Computed rotation by {np.degrees(angle):.2f} degrees.")
            return rot_matrix
        else:
            self.get_logger().info("   ↪ Floor already aligned, no rotation needed.")
            return np.eye(3)

    # --------------------------------------------------------------
    def handle_query(self, request, response):
        """Handles query. Returns position and UPDATES visualization for that object only."""
        name = request.name.strip().lower()
        found, point = self.search_object(name)
        
        response.found = found
        response.position = Point(x=point.x, y=point.y, z=point.z)
        
        if found:
            response.message = f'Found {name} at ({point.x:.2f}, {point.y:.2f}, {point.z:.2f})'
            # VISUALIZATION CHANGE: Show ONLY this object
            self.publish_object_marker(name)
        else:
            response.message = f'Object {name} not found.'
            # Optional: Clear markers if object not found
            self.clear_markers()
             
        self.get_logger().info(f'Query: {name} -> {response.message}')
        return response

    def search_object(self, name: str):
        if name in self.object_db:
            instances = self.object_db[name]
            instance_ids = self.object_instance_ids.get(name, [])
            if len(instances) > 1:
                self.get_logger().info(f"Multiple instances of '{name}' found. Waiting for user selection.")
                self.publish_candidate_options(name, instances, instance_ids)
                for i, inst in enumerate(instances):
                    instance_id = instance_ids[i] if i < len(instance_ids) else 'n/a'
                    map_x, map_y, map_z = self.transform_point_to_map(inst)
                    self.get_logger().info(
                        f"  [{i}] instance_id={instance_id} "
                        f"raw=({inst[0]:.2f}, {inst[1]:.2f}, {inst[2]:.2f}) "
                        f"map=({map_x:.2f}, {map_y:.2f}, {map_z:.2f})"
                    )
                choosen_index = input(
                    f"Multiple instances of '{name}' found. Enter index (0-{len(instances)-1}) to select, "
                    "or press Enter for random: "
                )
                selected_idx = None
                if choosen_index.isdigit():
                    idx = int(choosen_index)
                    if 0 <= idx < len(instances):
                        selected_idx = idx
                    else:
                        self.get_logger().warn(f"Invalid index entered. Selecting randomly.")
                if selected_idx is None:
                    selected_idx = random.randrange(len(instances))
                    self.get_logger().info(f"Randomly selected index [{selected_idx}] for '{name}'.")
                self.clear_candidate_options()
                best_pt = instances[selected_idx]
            else:
                best_pt = min(instances, key=lambda p: p[0]**2 + p[1]**2 + p[2]**2)
                self.clear_candidate_options()
            return True, Point(x=best_pt[0], y=best_pt[1], z=best_pt[2])
        else:
            self.clear_candidate_options()
            return False, Point(x=0.0, y=0.0, z=0.0)

    def publish_object_list(self):
        serializable_db = {k: v for k, v in self.object_db.items()}
        msg = String()
        msg.data = json.dumps(serializable_db, indent=2)
        self.pub_objects.publish(msg)

    def clear_markers(self):
        """Helper to clear all markers from Rviz."""
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        self.marker_pub.publish(marker_array)

    def publish_object_marker(self, target_name: str):
        """
        Clears previous markers and displays ONLY the requested object.
        """
        marker_array = MarkerArray()
        
        # 1. DELETE ALL previous markers first
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # 2. Add new markers for the specific object
        if target_name in self.object_db:
            locations = self.object_db[target_name]
            now = self.get_clock().now().to_msg()
            id_counter = 0
            
            for i, pos in enumerate(locations):
                x, y, z = pos
                display_name = f"{target_name} ({i+1})" if len(locations) > 1 else target_name

                # Sphere
                sphere = Marker()
                sphere.header.frame_id = "map"; sphere.header.stamp = now
                sphere.ns = "semantic_objects"; sphere.id = id_counter; id_counter += 1
                sphere.type = Marker.SPHERE; sphere.action = Marker.ADD
                sphere.pose.position.x = x; sphere.pose.position.y = y; sphere.pose.position.z = z
                sphere.pose.orientation.w = 1.0
                sphere.scale.x = 0.3; sphere.scale.y = 0.3; sphere.scale.z = 0.3
                sphere.color.r = 0.0; sphere.color.g = 1.0; sphere.color.b = 0.0; sphere.color.a = 1.0 # Green for active query
                sphere.lifetime.sec = 0 # Persistent until next query clears it
                marker_array.markers.append(sphere)
                
                # Text
                text = Marker()
                text.header.frame_id = "map"; text.header.stamp = now
                text.ns = "semantic_names"; text.id = id_counter; id_counter += 1
                text.type = Marker.TEXT_VIEW_FACING; text.action = Marker.ADD
                text.pose.position.x = x; text.pose.position.y = y; text.pose.position.z = z + 0.3
                text.pose.orientation.w = 1.0
                text.scale.z = 0.2
                text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0
                text.text = display_name; text.lifetime.sec = 0
                marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)

    def publish_point_cloud(self):
        # Publish 3D map point cloud instead of semantic map
        if self.map_3d_points is None or self.map_3d_colors is None:
            self.get_logger().warn("⚠️ 3D map not loaded, cannot publish point cloud", throttle_duration_sec=5.0)
            return
        points = self.map_3d_points
        colors = self.map_3d_colors
        msg = PointCloud2()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1; msg.width = points.shape[0]
        msg.is_bigendian = False; msg.is_dense = True
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        msg.point_step = 16; msg.row_step = msg.point_step * points.shape[0]
        buffer = []
        for i in range(points.shape[0]):
            x, y, z = points[i]
            r, g, b = colors[i]
            rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
            rgb_float = struct.unpack('f', struct.pack('I', rgb_int))[0]
            buffer.append(struct.pack('ffff', x, y, z, rgb_float))
        msg.data = b''.join(buffer)
        self.pcl_pub.publish(msg)

def main():
    rclpy.init()
    node = ObjectQueryServer()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
