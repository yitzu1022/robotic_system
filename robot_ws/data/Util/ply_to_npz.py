import numpy as np
from plyfile import PlyData
import os

def ply_to_npz(ply_file_path, npz_file_path):
    # 1. Load the PLY file using plyfile library
    print(f"Loading PLY file from: {ply_file_path}")
    plydata = PlyData.read(ply_file_path)
    
    # 2. Extract vertices
    vertex = plydata['vertex']
    
    # Check if the file has vertices
    if len(vertex) == 0:
        print("Error: Loaded point cloud has no points.")
        return

    # 3. Extract XYZ coordinates
    points = np.vstack([vertex['x'], vertex['y'], vertex['z']]).T
    print(f"Extracted {points.shape[0]} points")
    
    # 4. Extract color information from spherical harmonics DC components
    # f_dc_0, f_dc_1, f_dc_2 are the DC components of spherical harmonics
    if 'f_dc_0' in vertex.data.dtype.names:
        print("Extracting color from spherical harmonics DC components...")
        
        # Get DC components
        f_dc_0 = vertex['f_dc_0']
        f_dc_1 = vertex['f_dc_1']
        f_dc_2 = vertex['f_dc_2']
        
        # Convert from spherical harmonics to RGB
        # SH_C0 is the normalization constant for 0-th order spherical harmonics
        SH_C0 = 0.28209479177387814
        
        # Convert SH DC to RGB (0-1 range)
        rgb = 0.5 + SH_C0 * np.vstack([f_dc_0, f_dc_1, f_dc_2]).T
        
        # Clamp to valid range and convert to 0-255
        rgb = np.clip(rgb, 0.0, 1.0)
        colors = (rgb * 255).astype(np.uint8)
        
        print(f"Color range - Min: {colors.min()}, Max: {colors.max()}")
        
        # Save with colors
        np.savez_compressed(npz_file_path, points=points, colors=colors)
        print(f"✅ Saved points and colors to NPZ: {npz_file_path}")
        return points, colors
    else:
        print("Warning: No color information (f_dc_0/1/2) found in PLY file")
        # Save without colors
        np.savez_compressed(npz_file_path, points=points)
        print(f"Saved points only to NPZ: {npz_file_path}")
        return points, None

# --- Example Usage ---
# Replace 'input.ply' with the path to your actual PLY file
# Replace 'output.npz' with your desired output path
input_ply = '/home/weichen/robotic-project/robot_ws/data/lab/semantic_pcd_accumulated_gaussians.ply'
output_npz = '/home/weichen/robotic-project/robot_ws/data/lab/semantic_pcd_accumulated_gaussians.npz'

# Check if file exists
if not os.path.exists(input_ply):
    print(f"❌ '{input_ply}' not found.")
else:
    points, colors = ply_to_npz(input_ply, output_npz)
    
    # --- Verify the saved NPZ file ---
    print("\n📊 Verifying the saved NPZ file:")
    loaded_data = np.load(output_npz)
    print(f"Keys available in NPZ: {list(loaded_data.keys())}")
    print(f"Shape of loaded points: {loaded_data['points'].shape}")
    if 'colors' in loaded_data:
        print(f"Shape of loaded colors: {loaded_data['colors'].shape}")
        print(f"Color dtype: {loaded_data['colors'].dtype}")
        print(f"Color sample (first point): {loaded_data['colors'][0]}")
    else:
        print("No colors in NPZ file")
    
    loaded_data.close()
