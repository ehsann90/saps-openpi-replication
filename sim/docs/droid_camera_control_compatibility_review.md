# DROID camera and control compatibility review

Scope: pinned DROID `33ae6a6`, OpenPI `15a9616`, and the current Fusion-v13
candidate scene. This is a source and archived-evidence review; no rollout or
new Isaac/Polymetis experiment was run.

## Verified findings

### Camera calibration and inference

The [DROID calibration guide](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/docs/example-workflows/calibrating-cameras.md)
uses the GUI calibration hub for each camera and a mounted ChArUco board. The
[GUI](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/droid/user_interface/gui.py)
selects standard or high-resolution mode and invokes
[`calibrate_camera`](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/droid/trajectory_utils/misc.py).
The procedure collects left **and** right images while moving the robot through
a 15 Hz calibration trajectory. Its board is 14 × 9 squares, with 20 mm
squares and 16 mm markers in a 5×5 ArUco dictionary
([parameters](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/droid/misc/parameters.py)).
The [detector](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/droid/calibration/calibration_utils.py)
refines markers and ChArUco corners, estimates board-to-camera poses with
OpenCV using SDK intrinsics as a fixed focal-length/principal-point guess,
rejects high reprojection-error views, and uses `calibrateHandEye`. The wrist
result is camera-to-gripper; an external-camera result is camera-to-base.
Results are XYZ position plus XYZ Euler rotation in the OpenCV camera frame.
Left and right stereo views receive separate results. The calibration process
does **not** establish new lens intrinsics; the [ZED reader](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/droid/camera_utils/camera_readers/zed_camera.py)
gets each view's `cameraMatrix` and distortion from the SDK.
It retrieves SDK `VIEW.LEFT`/`VIEW.RIGHT` images; this inference path has no
separate OpenCV undistortion or rectification step.

Extrinsics and timestamps are saved by serial/view in
[`calibration_info.json`](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/droid/calibration/calibration_utils.py).
[`RobotEnv.get_observation`](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/droid/robot_env.py)
loads them, composes the moving wrist pose with measured gripper pose, and
records camera extrinsics and SDK intrinsics in observations. Dataset
[postprocessing](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/droid/postprocessing/schema.py)
retains episode camera metadata. External `ext1`/`ext2` are serial-ordered
camera IDs; `left`/`right` also names stereo views, so these labels must not
be conflated. The [OpenPI DROID example](https://github.com/Physical-Intelligence/openpi/blob/15a9616a00943ada6c20a0f158e3adb39df2ccac/examples/droid/main.py)
chooses **one** external camera's left stereo RGB view and the wrist's left
view, converts camera channels to RGB, and resizes with padding to 224×224.
The pinned [policy transform](../../third_party/openpi/src/openpi/policies/droid_policy.py)
uses those two RGB arrays, seven joint positions, one gripper scalar and prompt;
it does not accept intrinsic or extrinsic matrices. Calibration matters to
inference through the *rendered pixels*, not through a calibration input key.

In the [Fusion-v13 config](../configs/fr3_droid_robotiq_fusion_v13_candidate_scene.json),
the wrist camera is a CAD-derived, nominal **left optical-center** pose under
`fr3_link8`; it is not measured DROID hand-eye calibration. The external
camera preserves one episode's base-relative x/y and orientation but raises z
from 0.34884339 to 0.74884339 m for this tabletop: a deliberate 0.400 m
adaptation, not a calibrated simulator extrinsic. Both use the selected
episode's rectified-left SVO FOV. Isaac's [camera code](../src/isaac_fr3/cameras.py)
uses +X right, +Y up, -Z forward USD axes; OpenCV optical axes use +X right,
+Y down, +Z forward. The config quaternions are already in USD axes. Isaac
renders a centered 320×180 pinhole image and does not apply distortion or
stereo baseline. The 36 mm aperture is virtual. The source principal points,
scaled from 1280×720 to 320×180, are `(162.522, 88.659)` px (wrist) and
`(160.123, 88.308)` px (external), versus Isaac `(160, 90)` px: offsets
`(+2.522, -1.341)` and `(+0.123, -1.692)` px. Focal lengths scale to
183.422 and 130.603 px; the archived Isaac capture measured approximately
183.442 and 130.603 px. These are small *measured projection differences*;
their effect on policy behavior is unmeasured.

The archived [camera validation metadata](../../outputs/isaac_camera_validation/20261007T105410Z_880db492/metadata.json)
shows the older FR3 scene's wrist local-pose readback error of 3.0 nm and
6.5e-8 rad, constant wrist attachment through arm movement, and zero external
drift. Its saved RGB images and projections verify capture and coverage, not
physical calibration. No corresponding board reprojection or image-coordinate
check has been established for the Fusion-v13 assembly.

### Controller compatibility

Official [DROID arm conversion](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/droid/franka/robot.py)
and [solver](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/droid/robot_ik/robot_ik_solver.py)
give `q_des = q_measured + 0.2 × clip(action[:7], -1, 1)` at each update. The
local [mapping](../../src/saps/policies/openpi_droid.py) and
[`position_reference` executor](../src/isaac_fr3/policy_execution.py) use
fresh measured q, eight of 15 actions, and four 1/60 s Isaac steps per action
(15 Hz). The target is held during those four steps, then a fresh-q terminal
hold is installed. Inference and capture do not advance physics; the physical
DROID controller continues during inference. Isaac rejects out-of-limit
targets whereas upstream target handling is not an established match. PhysX
position drives and DROID's Polymetis torque-based impedance also remain
different low-level controllers. The selectable `joint_velocity` backend is an
explicitly scaled experiment, not this reference mapping.

The original [validated matched-HOME comparison](../../outputs/polymetis_pybullet/completed_controller_comparison/REPORT.md)
defines each ratio as `(q_at_next_boundary - q_at_issue) /
(q_target - q_at_issue)`. For summary medians, both systems exclude commands
smaller than 0.01 rad (54 of 56 joint-action pairs). Eight recorded free-space
actions began at measured HOME within 2.38e-8 rad with zero measured velocity.
Isaac used four 60 Hz substeps (66.7 ms) per command. The pinned
Polymetis/PyBullet **Panda**, not physical FR3, used a nominal 15 Hz schedule,
240 Hz physics, and 15–18 server ticks between updates. Its simulator gains
were `Kq=[20,30,25,25,15,10,10]`, `Kqd=[1,1.5,1,1,0.5,0.5,0.5]`,
`Kx=[100,100,100,40,40,40]`, `Kxd=[1,1,1,0.2,0.2,0.2]`; Isaac's loaded
arm drives had stiffness 3,437,746.75 and damping 343,774.6875
([audit](droid_deployment_audits.md)). Median ratios were **0.48614** (Isaac
Franka Hand), **0.48629** (Isaac Robotiq), and **0.40767** (Panda fresh-q).
The Isaac per-joint ratios in the [tracking tables](../../outputs/polymetis_pybullet/completed_controller_comparison/validated/tracking_tables.md)
mostly span about 0.46–0.56. Panda's
[per-joint table](../../outputs/polymetis_pybullet/matched_home_comparison/tracking_tables.md)
varies much more, including overshoot above 1 on some joints; 0.40767 is a
median, not a uniform 40% response. The Panda absolute-target replays used
older Isaac targets and are not identical-target comparisons with the new
fresh-q Isaac replays. No contact, gripper actuation, inference, or physical
robot participated in this controller comparison.

### Existing kinematics

The [manual NumPy FK/Jacobian](../../src/saps/physical/fr3_forward_kinematics.py)
and independent [Pinocchio wrapper](../../src/saps/physical/fr3_kinematics.py)
use `fr3_joint1`–`fr3_joint7` in radians, base frame `fr3_link0`, and the
Franka-Hand `fr3_hand_tcp` frame, with translation in metres and base-resolved
linear/angular Jacobian rows. The robot model comes from the lab FR3 xacro,
with fingers locked. The [validation record](../../docs/physical_fr3_embodiment.md)
reports 1,000 valid configurations: worst TCP position error 8.25e-16 m,
orientation 2.21e-15 rad, and Jacobian error 1.94e-15 versus Pinocchio;
finite-step manual FK versus Pinocchio differed by at most 4.37e-16 m or rad.
Isaac's [Lula IK](../src/isaac_fr3/grasp_poses.py) separately validates joint
order and solves `fr3_hand_tcp` approach poses for camera diagnostics. These
checks validate the FR3 arm/Franka-Hand model; they do not directly validate
the Fusion-v13 Robotiq `grip_frame` or fingertip locations. Pinocchio could
check the simulated flange independently, then use the documented fixed
flange-to-Robotiq transform to inspect TCP/tips if a measured pose discrepancy
appears. No IK is needed for joint-space policy actions.

## Assumptions, unresolved issues, and next action

1. **Highest consequence: contact and gripper geometry.** Fusion-v13's
   [static record](../assets/FR3_2F85_ZED_transforms/fusion_v13_candidate.md)
   and [exploratory ON/OFF comparison](../../outputs/isaac_joint5_diagnostics/hg1_b2_candidate/comparison.json)
   support a diagnostic trial, but CAD face registration, fingertip labels,
   and contact fidelity remain unqualified. Do not interpret grasp success or
   failure as physical DROID evidence.
2. **Visual domain gap.** Wrist pose is nominal CAD geometry; external pose is
   episode-derived and height-adapted. The small intrinsic-center discrepancy
   is quantified above. Before changing any camera parameter, use the existing
   camera validator on Fusion-v13 with a uniquely named output, inspect raw
   left-view images at HOME and a known pose, and compare projected visible
   landmarks/board corners against rendered pixels. Require a repeatable
   signed pixel residual before correcting pose or projection. Current
   evidence does not justify moving cameras or changing FOV/lighting.
3. **Control dynamics.** The 49% versus 40% medians compare different robots,
   controllers, and physics engines. The physical DROID gains and measured
   response are unknown. This is a known comparability limit, not an action
   conversion defect.

**Recommendation:** The current `position_reference` code can be used
unchanged for the next **manual exploratory** Fusion-v13 rollout. Select
`--config sim/configs/fr3_droid_robotiq_fusion_v13_candidate_scene.json`
and `--arm-control-mode position_reference`, use the pinned `pi05_droid`
server/checkpoint and a fresh run directory, and inspect the first saved
observation and action/hold records before interpreting the run. Keep the
0.2 mapping, eight actions, 15 Hz, gripper mapping, 32 solver position
iterations, assembly/contact settings, and current camera/lighting values.
No blocking **source-level** discrepancy or necessary code change was found.
The rollout remains diagnostic until camera geometry and contact fidelity are
independently qualified.
