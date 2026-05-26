# Verification Rules

The harness checks high-level consistency before or after capability execution.

## Invalid or Suspicious Calls

- Unknown capability is invalid.
- `object_query` without `target` is invalid.
- `navigation` without `target` or `pose` is invalid.
- `grasp_place` without valid `action` is invalid.
- `place` without `destination` is invalid.
- `finish` without `task_done=true` is invalid.

## Order Constraints

- For transfer tasks, querying and navigating to the source should happen before grasp.
- Placing should happen after a successful grasp.
- Destination lookup/navigation should happen before placing.
- Do not finish immediately after a failed capability.
- Do not repeat the exact same failed call unless new information has been observed.

## Target Constraints

- Do not use combined relational phrases as action targets, such as `pringles on table`.
- Use the source receptacle/place for source lookup, such as `table`.
- Use the object itself for grasp/place target, such as `pringles`.
