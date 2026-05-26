# Replanning Rules

Replanning is used when a capability call is rejected by the harness or execution fails.

## Failure Feedback

When a capability fails, the next decision receives:

- the failed capability call
- success status
- failure message
- recent execution history
- known poses
- held object state
- robot pose if available

## Replanning Policy

- Inspect `last_execution_result` before choosing the next call.
- If object lookup fails, try a more appropriate symbolic target before stopping.
- For source-aware transfer tasks such as `bring <object> on <source> to <destination>` or `move <object> from <source> to <destination>`, if `object_query(<object>)` fails, retry with `object_query(<source>)`.
- If navigation fails, do not proceed to grasp/place as if navigation succeeded. Retry or choose a corrected target/pose first.
- If grasp fails, do not navigate to destination for placing.
- If place fails, do not finish.
- Avoid infinite loops; respect `agent_max_steps` and `agent_max_replans`.

## Default Budget

The current node defaults to one replan after a failure. This can be changed with:

```bash
-p agent_max_replans:=1
```
