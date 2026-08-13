# grasp_baseline_v1

- Version: `baseline_v1`
- Goal: deterministic four-motion top-down grasp baseline.
- Input: natural-language task, selected object name/category, visual confidence.
- Output JSON schema:

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

It must not output TCP coordinates, joint angles, a `Command`, workspace
overrides, or a request to bypass Safety Checker.  If target identity is not
clear, use `failure_reason` and request no motion.

Test scenes: `standard`, `boundary_reject`.
Change record: initial offline baseline.
