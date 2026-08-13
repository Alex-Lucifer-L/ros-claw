# grasp_efficient_v2

- Version: `efficient_v2`
- Goal: reduce unnecessary vertical clearance while retaining a top-down safe
  approach and the same deterministic Safety/Gateway path.
- Input/output JSON schema:

```json
{
  "target_object": "string",
  "grasp_style": "parallel_top_down",
  "approach_direction": "top_down",
  "preferred_gripper_yaw": 0.0,
  "pre_grasp_strategy": "vertical_clearance",
  "need_reobserve": false,
  "confidence": 0.0,
  "failure_reason": null
}
```
- Difference: recommend a smaller but still explicit pre-grasp clearance and
  do not add a re-observation unless deterministic execution reports failure.

Forbidden: outputting joint angles, base coordinates, unbounded retries,
workspace changes, or a direct motor command.

Test scenes: `standard`, `randomized`, `noisy`, `boundary_reject`.
Change record: first efficiency comparison prompt.
