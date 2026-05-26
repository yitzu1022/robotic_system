# Planning Rules

This document describes high-level task decomposition rules for the agent-based decision maker.

## Closed-Loop Planning

- Produce exactly one next capability call per decision.
- Do not produce a full multi-step plan in iterative mode.
- Use the latest execution observation before choosing the next capability.
- Prefer symbolic targets such as `table`, `cabinet`, or `pringles`; do not invent low-level robot controls.

## Transfer Tasks

For tasks like `bring <object> on <source> to <destination>`:

1. First try `object_query` on the object itself.
2. If the object lookup succeeds, navigate to that object pose.
3. If the object lookup fails, use the failure observation to query the source location.
4. Navigate to the source only after the source lookup succeeds.
5. Grasp only the object, not the combined phrase.
6. Locate the destination with `object_query`.
7. Navigate to the destination.
8. Place the object at the destination.
9. Finish only after place succeeds.

For tasks like `move <object> from <source> to <destination>`:

1. First try `object_query` on the object itself.
2. If the object lookup succeeds, navigate to that object pose.
3. If the object lookup fails, use the failure observation to query the source location.
4. Navigate to the source only after the source lookup succeeds.
5. Grasp only the object, then locate/navigate to the destination and place it.

Example:

```json
{"capability":"object_query","target":"table","reason":"Need to locate the source."}
```

## Multi-Object Transfer Tasks

For tasks with multiple objects joined by `and` or commas, the LLM planner must reason about the object list and decompose the request into one complete transfer per object. Never use the combined object phrase as a target.

Example task:

```text
bring bottle and apple to table
```

Expected LLM reasoning/decomposition:

1. Locate `bottle`.
2. Navigate to `bottle`.
3. Grasp `bottle`.
4. Locate `table`.
5. Navigate to `table`.
6. Place `bottle` at `table`.
7. Locate `apple`.
8. Navigate to `apple`.
9. Grasp `apple`.
10. Locate `table`.
11. Navigate to `table`.
12. Place `apple` at `table`.
13. Finish only after every object has been placed.

For tasks where multiple objects share a source, such as:

```text
bring bottle and apple on cabinet to table
```

Use `cabinet` as the source for each object, but still grasp and place one object at a time.

## Pose Query Tasks

For tasks asking where the robot is, use `robot_pose` first, then finish after the pose observation succeeds.
