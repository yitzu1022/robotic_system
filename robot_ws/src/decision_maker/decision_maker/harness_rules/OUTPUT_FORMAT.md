# Output Format

The planner must return exactly one JSON object. No markdown, no prose, and no list of future steps.

Allowed `capability` values:

- `object_query`
- `navigation`
- `grasp_place`
- `robot_pose`
- `finish`

## object_query

```json
{"capability":"object_query","target":"cabinet","reason":"Need to locate the source before navigation."}
```

## navigation

```json
{"capability":"navigation","target":"cabinet","pose":{"x":4.43,"y":-9.84,"theta":0.45},"reason":"Navigate to the source."}
```

## grasp

```json
{"capability":"grasp_place","action":"grasp","target":"pringles","reason":"Grasp the object at the source."}
```

## place

```json
{"capability":"grasp_place","action":"place","target":"pringles","destination":"sofa","reason":"Place the object at the destination."}
```

## robot_pose

```json
{"capability":"robot_pose","reason":"Need to know the robot current pose."}
```

## finish

```json
{"capability":"finish","task_done":true,"reason":"The requested task is complete."}
```
