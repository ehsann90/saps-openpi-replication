# Physical FR3 deployment

SAPS owns policy observations, action mapping, chunk execution, arbitration,
readiness checks and experiment logging. The independent ROS2 repository at
`~/franka_ehsan_ros2_ws/src/franka_droid_stack` owns the hardware:

- `franka_droid_stack`: hardware orchestration and joint-target forwarder.
- `franka_robotiq`: exclusive serial owner, gripper driver and ROS services.
- `franka_zed_cameras`: camera configuration, launch and validation.

Use that repository's README and package READMEs for installation and hardware
operation. SAPS no longer provides camera or serial-driver launch wrappers.
Start the commissioned Franka bringup and impedance controller through the
existing independent robot-control workflow. Preserve controller/tool/load
settings, calibration and limits. The unified hardware launcher does not manage
that controller:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ehsan_ros2_ws/install/setup.bash
ros2 launch franka_droid_stack hardware.launch.py \
  gripper_config:="$HOME/franka_ehsan_ros2_ws/src/franka_droid_stack/franka_robotiq/config/robotiq.json"
```

This launches cameras, the read-only gripper server and the forwarder. Retain
existing owners using the hardware launcher's `start_*:=false` switches; do not
start duplicate drivers. Gripper commands need its separate supervised
`authorize_gripper:=true` option. Startup does not activate/reset the gripper.

From SAPS, use the existing `.venv-physical` environment (system ROS Python 3.12),
the commissioned ignored local C4 configuration, and the DROID server:

```bash
source tools/activate_physical_env.sh
export PHYSICAL_CONFIG="$PWD/configs/local/c4_zed_robotiq.json"
make droid-policy-server
```

The activation script defaults to the current workspace `install/`, replacing
the retired `install-droid-current/` default. `FRANKA_DROID_INSTALL` still permits
an explicit overlay. `PHYSICAL_CONFIG` defaults to the local C4 profile rather
than the older ZED commissioning profile; an explicit override is preserved.
The server uses `pi05_droid` and
`gs://openpi-assets/checkpoints/pi05_droid`, with OpenPI pinned at
`15a9616a00943ada6c20a0f158e3adb39df2ccac`. `make policy-stop` stops it afterward.

Run non-actuating readiness/observations first with unique `PHYSICAL_RUN_ID`
and `PHYSICAL_PROMPT`, using `make physical-c4-readiness` and
`make physical-c4-observations` (`C4_CONFIG` selects the local C4 JSON).
For separately supervised execution use `scripts/physical_policy_execution.py`
with the explicit `--execute`, C4 `--config`, unique `--output-dir`, prompt,
finite limits and application-confirmation bound from the
[bounded development workflow](physical_development_rollout.md).
No new execution defaults or safety thresholds are implied here.
`--lab-stack-dir` identifies the outer hardware Git root.
Strict source qualification still rejects the old manifest/new-tree mismatch;
this migration does not qualify new formal experiments.

The policy contract remains `[15,8]`, eight executed arm actions per chunk at
15 Hz, `q_target = q_measured + 0.2 * clip(action[:7], -1, 1)`, and
`action[7] > 0.5` means CLOSE, otherwise OPEN. Holds and runtime faults do not
indicate task success. There is no deployed policy completion detector.

For offline video export, see [physical observation video](physical_video.md).
The [first successful manipulation record](physical_rollout_20261111.md)
separates operator-observed success from the recorded runtime abort.
[Historical commissioning](physical_zed_robotiq_commissioning.md) retains
calibration, mapping, mechanism tests and hardware findings; its old launch
commands are not the current deployment procedure.
