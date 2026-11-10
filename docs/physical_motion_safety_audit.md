# One-chunk C4 motion-safety audit — 2026-11-09

**NO-GO with the supplied evidence.** The active architecture and configured
hybrid gains are confirmed. A torque-slew safeguard already exists; a new
15 Hz trajectory limiter is not justified merely because joint equilibrium
targets are sent. Actual tool/load compatibility and workspace protection
remain unverified. No settings, policy actions or motion algorithms were changed.

## Evidence and restrictions actually on this path

Fresh researcher evidence:
`~/franka_ehsan_ros2_ws/c4_runtime_inspection_20261109T161032Z/`.
The six supplied files, inspected source/library hashes and native offline
probe results are recorded in [evidence.json](validation/2026-11-09_motion_restrictions/evidence.json).
The captured URDF hash matches the XML file. Earlier failed graph queries from
the Codex environment do not invalidate this supplied capture.

| Layer | Enforced/relevant restriction | What it does not establish |
|---|---|---|
| SAPS | Canonical arm order, finite `[15,8]` predictions, seven-joint position bounds, 0.1 rad target margin, configured flange singularity gate, fresh advancing measured state and repeated health checks. | No plant velocity/acceleration/torque bound follows from position feasibility. |
| SAPS scheduling | Eight selected actions at 15 Hz, independently rounded deadlines, missed-interval rejection, then measured-q terminal hold and existing confirmation/health audits. | `0.2*clip(u)` is an equilibrium offset, not a commanded velocity. Bounded commands do not imply the arm instantly stops at the final tick. |
| Forwarder | Names/count/finiteness, receipt timestamp age/future tolerance, fresh active-controller evidence; unchanged forwarding to the controller. | No geometric collision/workspace check, joint-velocity limiter, absolute torque cap or target-jump filter. It is not an independent motion planner. |
| Impedance controller | Active-state and target validity checks; target age 0.25 s/future tolerance 0.05 s; finite q/dq/Jacobian/Coriolis/effort checks. Latest accepted equilibrium is held. | No numerical speed/acceleration cap or absolute effort saturation in its core/hybrid math. A vanished publisher does not make the held equilibrium disappear. |
| `franka_hardware` effort branch | `initializeTorqueInterface()` selects `startTorqueControl()`. `writeOnceJointEfforts()` applies `limitRate(kMaxTorqueRate, tau, current_state.tau_J_d)` with a private flag defaulting true. All command arrays are checked for finiteness before write. | This is torque slew limiting, not joint speed/acceleration limiting or an absolute-torque clamp. The output is an effort command, although the upstream target topic contains joint positions. |
| Installed libfranka 0.15.0 | `kMaxTorqueRate=999.999 Nm/s`, nominal `kDeltaT=0.001 s`: approximately 0.999999 Nm change per nominal write relative to robot-reported previous desired torque. | `ActiveTorqueControl::writeOnce` itself does not add position/velocity trajectory limiting; it forwards finite torques. Actual timing and robot-native protection settings still matter. |
| Other hardware branches | Joint position/velocity branches contain acceleration/jerk limiters; their defaults are false. | These branches are not used when the streaming controller claims effort interfaces. Their constants are not evidence of acceleration enforcement on the torque path. |
| Controller manager | Inspected source can import joint limiters when `enforce_command_limits` is enabled; inspected default is false. | Supplied `controller_manager.yaml` contains an empty parameter map. It proves neither true nor false. Limiter loading, claimed-interface coverage, actual limits and execution must be established before relying on it. |
| URDF/MoveIt | Deployed arm URDF lists velocity caps `[2.62,2.62,2.62,2.62,5.26,4.18,5.26] rad/s` and efforts `[87,87,87,87,12,12,12] Nm`. MoveIt YAML has planning acceleration limits. | Metadata is not enforcement. MoveIt planning limits do not supervise independently streamed effort control. Native firmware limits can also differ from these conservative URDF values. |
| Robot-native feedback | Existing SAPS checks reject robot fault/mode/collision indicators. | Reflex monitoring is reactive, not verified collision-clearance or reduced-speed protection for this experiment. |

The source trace is through `policy_execution` → `StreamingBoundary` → existing
forwarder → `StreamingJointImpedanceController` → `franka_hardware::write` effort
branch → `writeOnceJointEfforts` → active libfranka torque control. No IK or
Cartesian motion generator is on that route.

Franka distinguishes the torque-interface slew condition from motion-generator
position/velocity/acceleration/jerk requirements in its
[interface specification](https://frankarobotics.github.io/docs/robot_specifications.html).
Its [overview](https://frankarobotics.github.io/docs/doc/libfranka/docs/overview.html)
also distinguishes torque-rate processing from motion-generator processing.
Those interface requirements do not certify the configured lab workspace or
prove that a conservative experimental motion envelope is enforced.

## Is another velocity/acceleration limiter necessary?

**Not established as an unconditional requirement for this effort interface.**
The missing evidence is an adequate, verified motion envelope and stopping
arrangement, not the existence of a particular new software component.

Do not differentiate 15 Hz equilibrium setpoints as if they were a 1 kHz joint
position motion generator. A step in equilibrium creates a stiffness-dependent
effort response; actual speed/acceleration depends on gains, damping, inertia,
contact and native control. Clipping/slewing those targets would change the
established execution protocol and still would not by itself bound plant motion.

The installed `joint_limits` 4.48.1 effort-limit helper was exercised offline:
with the example joint-1 URDF bounds it returns `[-87,87] Nm` at rest and
`[-87,0] Nm` above the configured positive velocity cap. With effort limits
disabled it returns infinities. This confirms an available existing building
block, not an active controller-manager safeguard or an acceleration guarantee.
The local controller-manager source is version 4.36.0 and links installed
hardware_interface/joint_limits 4.48.1; runtime behavior must be checked rather
than inferred from a different version's generic documentation.

No additional limiter or automatic `enforce_command_limits` setting was added.
If verified native/existing protection cannot satisfy the approved initial
motion envelope, a separately reviewed bound/rejection or existing limiter
configuration is needed. Its limits, claimed effort-interface behavior and
effect on the evaluation must be specified first. No unverified values or
permissive approval flags are introduced here.

## Actual runtime controller and tool/load findings

- Streaming impedance controller is **active**; the historical trajectory
  controller is **inactive**. Forwarder node information shows the expected
  target topic and controller destination. Graph wiring is not delivery or
  protection proof; existing live health/T4 checks remain required.
- Configured synchronous controller update rate is **1000 Hz**, hybrid mode
  **true**. This is the configured rate, not a timing measurement.
- Active `Kq=[40,30,50,25,35,25,10]`, `Dq=[4,6,5,5,3,2,1]`,
  `Kx=[750,750,750,15,15,15]`, `Dxd=[37,37,37,2,2,2]` match the lab YAML.
- SDK `F_T_EE` readback is translation **[0,0,0.1034] m**, yaw **−45°**;
  `EE_T_K` is identity. F_T_EE numerically matches the historical Hand TCP
  transform in the captured URDF. A virtual controller point may be intentional;
  it need not be a Robotiq fingertip TCP, but compatibility with the retained
  hybrid gains has not been established by this capture.
- Configured end-effector and total mass are approximately **0.73 kg**;
  external load is **0 kg**. SDK flange-referenced CoM is approximately
  **[−0.01,0,0.03] m**, with diagonal inertia
  **[0.001,0.0025,0.0017] kg m²**, zero off-diagonals. These are configured
  values, not measurements of the installed Robotiq/coupling/camera assembly.
  No measured total, mass allocation or validated inertial provenance was supplied.
- The controller uses `getZeroJacobian(kEndEffector)` and Coriolis from actual
  libfranka state. F_T_EE and total inertial settings therefore affect its
  behavior. EE_T_K is used by libfranka's stiffness-frame branch, which this
  controller does not request. No tool/load setter was called.
- The deployed XML still contains Franka Hand/finger geometry. It cannot be
  used as evidence of the actual Robotiq collision envelope. Its seven-joint
  limits remain usable for the joint-space arithmetic.

The captured `f_t_ee.header.frame_id` says `fr3_link6`. In the inspected semantic
component, a fixed index selects the header name from a link list whose root
is inserted twice; the captured tree explains this label. Numeric F_T_EE is
copied directly from SDK RobotState, not transformed from link6. Inertia header
labels also have a source TODO. Do not reinterpret the numeric SDK values using
those labels alone. The controller consumes raw SDK state, so this finding does
not justify changing its transform or gains. No vendor package was modified.
The inspector's aggregate NO-GO label is not used as independent proof here;
the blocking findings are the missing physical compatibility/protection evidence.

## Minimum clearance/supervision evidence for the initial chunk

1. A documented initial measured q/dq and allowed motion region with verified
   self-collision, table/fixture and external clearances. Cover **all arm links**
   and a conservative measured envelope of coupling, Robotiq finger/linkage
   sweep, wrist camera and cables. Full CAD/articulated IK is not inherently needed.
2. Evidence that the independent protection actually covers this streamed
   torque-control route, or a reviewed conservative exclusion/clearance envelope
   that covers its full possible motion and stopping behavior. Do not credit
   separately running MoveIt/Servo or an operator watching a display as automatic
   geometric supervision.
3. Include tracking overshoot, sensing/reaction latency, torque-slew-limited
   braking, terminal-hold settling and the held-last-target failure case.
   Eight commands and safe target endpoints alone do not bound the swept volume.
4. Verified native protection configuration and available emergency stop with
   supervised abort/hold behavior. Confirm whether any existing manager limiter
   is enabled, loaded and applicable, and whether its behavior meets the chosen
   effort/motion bounds. A Boolean parameter alone is not sufficient evidence.
5. Validate the configured controller point/gains and assembly/load settings
   against measured properties or traceable component data. Do not double-count
   the tool as both administrator end effector and external payload.

No such clearance/protection record was included in the supplied capture.
The initial run therefore remains blocked; no existing guard is cleared solely
by these parameter dumps or by successful library tests.

## Changes, tests and next step

The sole executable-code change refines the existing physical-blocker wording
to request verification of native effort/motion restrictions (including torque
slew), rather than implying that a new software velocity/acceleration limiter
must exist. Guard behavior remains fail-closed in development and strict modes.
Dirty-source development provenance remains unchanged.

Offline checks passed: **160 focused SAPS tests**, **470 full-suite tests** plus
compilation, and **37 cases** in the three selected existing impedance-core,
hybrid-math and fake-forwarder CTest suites. The standalone native-library probe
passed positive/negative slew, in-range, non-finite, no-absolute-clamp and optional
effort-bound checks without creating a robot object or ROS control process.

```bash
cd ~/saps-openpi-replication
make check
git diff --check
cd ~/franka_ehsan_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PYTHONNOUSERSITE=1 colcon test --packages-select fr3_lab_stack --ctest-args \
  -R 'streaming_joint_impedance_core|hybrid_impedance_math|streaming_joint_target_forwarder'
colcon test-result --verbose
```

The aggregate test-result directory includes older suites; only the three suites
(5 core + 5 hybrid + 27 forwarder cases) above were rerun for this ROS audit.
The native probe source/result are kept with the evidence; it is an offline
diagnostic, not a new runtime component. No motion, setting change or commit occurred.

To rerun the native probe without any robot connection:

```bash
cd ~/saps-openpi-replication
source /opt/ros/jazzy/setup.bash
cmake -S docs/validation/2026-11-09_motion_restrictions \
  -B /tmp/c4_native_limits_repeat
cmake --build /tmp/c4_native_limits_repeat
/tmp/c4_native_limits_repeat/c4_native_limits
```

The fresh captured URDF also passed `check_urdf`, and the captured gain/timing
parameters matched the lab YAML. These are structural/configuration checks,
not mounted-hardware or enforced motion-envelope qualification.

After the remaining physical evidence is established and reviewed into the
existing guard, use the exact one-chunk command in the
[development workflow](physical_development_rollout.md#exact-one-chunk-development-command--conditional-on-physical-safety).
It retains development provenance, pretrained pi05-DROID, the existing threshold,
`q_target=q_measured+0.2*clip(action[:7],-1,1)`, eight actions at 15 Hz, binary
Robotiq semantics and terminal hold. That command is still rejected today.
