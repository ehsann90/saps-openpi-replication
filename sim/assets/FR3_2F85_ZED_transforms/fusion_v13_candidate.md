# HG1-B1 Fusion-v13 candidate mount

The committed historical assembly has been copied byte-for-byte to
`sim/assets/fr3_robotiq_droid_historical_hg1.usda`, selected by
`sim/configs/fr3_droid_robotiq_historical_hg1_scene.json`. Its October 7
transform record remains `fr3_2f85_zed_transforms.json`. This explicit copy
also pins the geometry independently of future working-tree edits. Neither
candidate nor frozen historical selection uses the mutable baseline file. The
separate scene
`sim/configs/fr3_droid_robotiq_fusion_v13_candidate_scene.json` selects
`sim/assets/fr3_robotiq_droid_fusion_v13_candidate.usda`. The candidate USD
references the frozen historical assembly and authors the Robotiq parent
translation and mount-joint `physics:localPos0`, both at `(0, 0, 0.010758)` m.
It separately retains the operator's coupling-visual translation
`(0.00207, 0.06565, -0.20565)` m; the historical value is
`(0.00207, 0.06565, -0.20265)` m. The upstream coupling contains no
collision, rigid-body, mass, or PhysX property on its 83 composed prims.
The historical assembly and scene SHA-256 values are pinned in
`fusion_v13_candidate.json`; changing either must trigger another review.

Both values are in the `fr3_link8` flange frame: the parent Xform maps the
upstream `base_link` into link-8, and the fixed joint's body0 is link-8. The
joint's body1 is `base_link`; `physics:localPos1=(0,0,0)`, both joint rotations,
and the parent rotation remain unchanged. The selected CAD base origin is
registered to USD `base_link` using the existing upstream `base_link/visuals`
orientation `(w,x,y,z)=(0.5,0.5,0.5,0.5)` and zero translation. This
visual-frame registration is conditional, not proof that the CAD body origin
and USD physics frame intrinsically coincide.

The selected CAD flange/coupling and coupling/gripper supporting planes are at
0 and +11 mm in the physical flange basis. The gripper CAD origin is at
+10.758 mm, versus +13.758 mm in the historical assembly. The upstream USD
grip frame is +133.717067608 mm from base_link, giving +144.475067608 mm
from flange versus the previous +147.475067608 mm. The nominal camera pose is
unchanged; `fusion_v13_candidate.json` records its revised relative transform
to the gripper. The separately adjusted coupling visual is not CAD-face
registered: its CAD component-to-imported-USD-mesh transform remains
unverified. It is visual-only; no physical coupling is added.

The Fusion v2 face records remain
`plane_qualified_trim_containment_unresolved`. The operator's fingertip labels
are not yet mapped to the USD left/right finger bodies. These limit the
mounting-face claim. This candidate is neither fully CAD-validated nor a
demonstrated remedy for the earlier q5 excursions. Those archives describe
the historical geometry and must remain unchanged.

## Static validation

`tools/hg1_validate_candidate_mount.py` uses the standalone OpenUSD libraries
bundled with Isaac Sim 6.1, without `SimulationApp` or physics stepping. It
checks scene selection, the candidate's three authored opinions, locally
composed joint and fingertip properties, fixed-joint relationships, anchor
coincidence, and the base/grip transforms. The stock FR3 asset is a remote
reference and did not resolve in the current offline environment; seven arm
joint prims, camera prim, and full rigid-body hierarchy therefore require a
read-only composed-stage check in a local Isaac session with that asset
available. The scene JSON camera values and every other config value are
identical, and the candidate overlay authors no arm or camera opinions.

## Later focused contact comparison (after review)

The SIM-P13F `run_contact.py` now accepts opt-in `--config`, with its original
default preserved. First run collider inspection for
the candidate and review its exact pad collider report. Then run candidate
Robotiq/table ON calibration and loaded-contact trial in a unique directory,
using the established 6 ± 2 mm clearance, approach-speed, impulse, secondary
contact, and retreat gates. Only after ON qualifies, replay its saved plan
with table OFF in another unique directory. Compare q5 ON-minus-OFF against
the archived historical Robotiq pair, aligning to first verified pad/table
loaded contact and reporting any changed contact exposure. Do not run a policy
rollout or restart the four-assembly investigation.

When the stock FR3 asset is accessible, run the no-step composed-stage gate
before collider inspection:

```bash
~/isaacsim/python.sh -u sim/scripts/hg1_composed_stage_check.py
```

Require `stock_fr3_arm_composed_and_compared: true` and no limitation before
proceeding. This checks the seven stock arm joint prims and link-8 properties
against the frozen historical assembly. The camera prim is created at runtime;
the gate verifies the unchanged wrist-camera configuration and parent frame.

Proposed candidate inspection command, to run only in a local Isaac session
with GPU and stock FR3 asset access:

```bash
~/isaacsim/python.sh -u outputs/isaac_joint5_diagnostics/matched_contact_comparison/run_contact.py \
  --assembly robotiq --table enabled \
  --config sim/configs/fr3_droid_robotiq_fusion_v13_candidate_scene.json \
  --inspect-colliders \
  --output outputs/isaac_joint5_diagnostics/hg1_b2_candidate/collider_inspection
```

The inspection writes a uniquely named `startup_logs/*.colliders.json` and
stops before calibration. After reviewing it, the candidate ON command is:

```bash
~/isaacsim/python.sh -u outputs/isaac_joint5_diagnostics/matched_contact_comparison/run_contact.py \
  --assembly robotiq --table enabled \
  --config sim/configs/fr3_droid_robotiq_fusion_v13_candidate_scene.json \
  --verified-collider-report outputs/isaac_joint5_diagnostics/hg1_b2_candidate/collider_inspection.20261013T102057077816Z.60311.colliders.json \
  --output outputs/isaac_joint5_diagnostics/hg1_b1_candidate/robotiq_table
```

If ON qualifies, run its no-table control using the exact saved plan:

```bash
~/isaacsim/python.sh -u outputs/isaac_joint5_diagnostics/matched_contact_comparison/run_contact.py \
  --assembly robotiq --table disabled \
  --config sim/configs/fr3_droid_robotiq_fusion_v13_candidate_scene.json \
  --verified-collider-report <reviewed_candidate_collider_report.json> \
  --plan outputs/isaac_joint5_diagnostics/hg1_b2_candidate/robotiq_table/plan.json \
  --output outputs/isaac_joint5_diagnostics/hg1_b2_candidate/robotiq_no_table
```

Then run the pair comparison, writing a new output file:

```bash
python3 tools/hg1_compare_q5_contact.py \
  --historical-on outputs/isaac_joint5_diagnostics/matched_contact_comparison/robotiq_table_multijoint \
  --historical-off outputs/isaac_joint5_diagnostics/matched_contact_comparison/robotiq_no_table_multijoint \
  --candidate-on outputs/isaac_joint5_diagnostics/hg1_b2_candidate/robotiq_table \
  --candidate-off outputs/isaac_joint5_diagnostics/hg1_b2_candidate/robotiq_no_table \
  --output outputs/isaac_joint5_diagnostics/hg1_b2_candidate/comparison.json
```

The explicit config selector requires the control plan and reviewed collider
report to record the same scene config. No HG1-B2 physics experiment has been
run in this environment.
