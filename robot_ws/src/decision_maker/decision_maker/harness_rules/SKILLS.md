# Skills

The agent can call only the capabilities listed here. These are high-level robot skills; the agent must not directly control low-level motors or rewrite execution interfaces.

## object_query

Purpose: Query the semantic map or object query service for an object/place pose.

Input:

```json
{"capability":"object_query","target":"table","reason":"Need the table pose."}
```

Constraints:

- `target` is required.
- Use symbolic names.
- Do not use combined phrases such as `pringles on table` as a target. Use `table` for source lookup and `pringles` for grasp.

## navigation

Purpose: Navigate the robot to a symbolic target or pose.

Input:

```json
{"capability":"navigation","target":"table","pose":{"x":1.0,"y":2.0,"theta":0.0},"reason":"Navigate to the source."}
```

Constraints:

- Requires `target` or `pose`.
- Prefer using a pose returned by a previous `object_query`.
- The agent does not control path planning or motor commands.

## grasp_place

Purpose: Grasp or place an object through the existing manipulation interface.

Grasp input:

```json
{"capability":"grasp_place","action":"grasp","target":"pringles","reason":"Grasp the target object."}
```

Place input:

```json
{"capability":"grasp_place","action":"place","target":"pringles","destination":"sofa","reason":"Place the object at the destination."}
```

Constraints:

- `action` must be `grasp` or `place`.
- `grasp` requires `target`.
- `place` requires `target` and `destination`.
- Place only an object that is expected to be held.

## robot_pose

Purpose: Return the robot current pose from TF in live mode or mock pose in mock mode.

Input:

```json
{"capability":"robot_pose","reason":"Need current robot pose."}
```

Constraints:

- No target is required.
- Live mode reads the configured TF transform such as `map -> base_link`.

## finish

Purpose: End the task.

Input:

```json
{"capability":"finish","task_done":true,"reason":"The task is complete."}
```

Constraints:

- Requires `task_done=true`.
- Do not finish immediately after a failed capability.
