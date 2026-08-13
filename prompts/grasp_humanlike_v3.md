# grasp_humanlike_v3

- Version: `humanlike_v3`
- Goal: select a shape-aware, continuous top-down grasp and allow **one** new
  visual observation after a failed verification rather than repeating a stale
  grasp blindly.
- Input/output JSON schema:

```json
{
  "target_object": "string",
  "grasp_style": "shape_aware_top_down",
  "approach_direction": "top_down",
  "preferred_gripper_yaw": 0.0,
  "pre_grasp_strategy": "vertical_clearance",
  "need_reobserve": true,
  "confidence": 0.0,
  "failure_reason": null
}
```
- Difference: `need_reobserve` may be `true`; it only authorizes one new
  observation.  It does not authorize an automatic physical retry.

Forbidden: direct joint/TCP output, inventing an object location from words
such as left/right, changing safety limits, or performing more than one retry.
All physical steps remain subject to the deterministic Safety Checker and
Gateway; this prompt cannot override either one.

Test scenes: `multi_object`, `noisy`, `depth_holes`, `boundary_reject`.
Change record: initial closed-loop strategy prompt.
