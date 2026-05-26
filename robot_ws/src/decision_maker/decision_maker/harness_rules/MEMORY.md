# Execution Memory

The harness maintains short-term task memory and passes it back to the agent during closed-loop planning.

## State Fields

- `task`: original user instruction
- `known_poses`: symbolic target to pose mapping from object query results
- `held_object`: object currently believed to be held
- `robot_pose`: latest robot pose observation
- `step_index`: current closed-loop decision step

## Feedback Fields

- `execution_memory`: recent capability calls and observations
- `recent_history`: recent successful or failed executed steps
- `last_execution_result`: most recent observation
- `last_failure`: most recent failed call and result
- `replans_used`: number of replans already consumed
- `max_replans`: replan budget for the task

## Observation Example

```json
{
  "last_action": "object_query",
  "target": "table",
  "success": true,
  "result": {"pose": {"x": 2.0, "y": 0.0, "theta": 0.0}}
}
```
