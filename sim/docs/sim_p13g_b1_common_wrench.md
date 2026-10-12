# SIM-P13G-B1 common-wrench comparison

## Status and question

**Four contact-free conditions completed.** B1 asks whether changing the
attached end-effector assembly changes the FR3's *position-level* response to
the same prescribed external impulse at `fr3_link8`. The comparison controls
for each assembly's no-pulse drift. It cannot separately identify the effects
of mass, inertia, mounting geometry, articulation topology, or contact
constraints.

The earlier [SIM-P13G Stage A report](../../outputs/isaac_joint5_diagnostics/sim_p13g/stage_a_report.md)
and its `analysis.json` measured a peak table-induced q5 shift of
**+0.130094 rad** for Robotiq and
**−0.002350 rad** for Franka during *loaded fingertip/table contact*. Those
conditions differed in contact location and motion history. B1 instead
applies a prescribed load at a common link-8-frame point with table and target
collisions disabled; its result is not a replay of the Stage A contact wrench.

## Frozen protocol and execution provenance

The [approval manifest](../../outputs/isaac_joint5_diagnostics/sim_p13g/b1_approval.json)
records protocol SHA-256
`aef1fb43847c4ab6256aa04e4bc16e3c3becbcec6baa6132d60f49abf38c8333`
and the two completed composed-USD preflights. The frozen values in
`b1_protocol.py` and the actual `run_common_wrench.py` execution path are:

| Item | Value |
| --- | --- |
| Initial arm q setpoint and constant target, rad | `[0, -0.4, 0, -1.9, 0, 1.5, 0]` |
| Initial arm dq, rad/s | seven zeros |
| Gripper | configured open; Franka finger targets 0.04 m each, Robotiq active driver 0 rad |
| Physics timestep | 1/60 s |
| Settle and monitored interval | 316 steps after `command_home`; then sample 0 and 90 stepped samples (1.5 s) |
| Control | no external pulse |
| Pulse | one call immediately before physics step 10, mode `"Impulse"` |
| Link-8-frame application point, m | `[0.05, 0, 0.10]` |
| Link-8-frame impulse, N·s | `[0, 0, 1/6]` (equivalent to 10 N over 1/60 s) |
| Table and target collisions | `/World/Table` and `/World/TargetCube` disabled before settling |
| Robot self-collision | unchanged |
| Seven arm position-drive gains | stiffness 3,437,746.75; damping 343,774.6875 per joint |
| Seven arm effort limits | 87 N·m on joints 1–4; 12 N·m on joints 5–7 |

The runner used the stock Franka scene and the existing derived Robotiq
scene, without tuning their assets, solver, joint drives, or sleep settings.
It recorded the same joint limits and 1/60 s physics timestep across all
conditions. Every archive reports Git HEAD
`2a81a9ae35ded3f57355f831d6ad86348da22caf`. The ignored runner and
protocol files are indexed below and must be retained with the datasets:
the protocol hash covers the frozen proposal, **not** the entire runner source.

After settling, a temporary q2 probe and restoration synchronized the
physics-tensor link-8 transform without advancing simulation. All four
initial-state and pose-restoration gates passed. At step 10, the runner
re-read q/dq and the link-8 tensor pose, transformed the local point and
impulse to world coordinates, and checked the frame without stepping. The
two pulse world points differ by 0.0592 mm and the requested world impulse
vectors by 3.67e-5 N·s in Euclidean norm; their magnitudes are approximately
1/6 N·s. The assembled tools and their dynamics differ, as intended.

Sample 0 follows state restoration. Sample k (1–90) follows physics step k;
`time_s = k/60` is the runner's simulated-time label. All reported q values
are seven measured arm joints in configured order. q and dq use the same
selected DOF indices, but the velocity qualification below limits any dq
interpretation. Contact events came from installed robot rigid-body-owner
reporters; zero *reported* contacts do not independently prove every possible
collision was absent.

## Execution and qualification history

The first Franka control completed but its q/dq became exactly fixed after
step 25. A first sleep qualification failed **before collecting samples**
because `omni.physics.core.IPhysicsSimulation` has no `is_sleeping` method.
The corrected `omni.physx` rigid-body sleep query in the Franka qualification
directly measured link-8 sleep starting at step 25 while the clock and
timeline continued. That qualification's q/dq trajectory is bitwise
identical to the original Franka control.

The first Franka pulse archive failed at step 10 with ten pre-pulse samples:
the core simulation interface has no `apply_force_at_pos` method, and there
was no successful force record. The corrected `omni.physx` force binding
completed the Franka pulse (`v2`). The later Robotiq control and pulse also
completed. The failed archives are retained as diagnostic history and are
**excluded** from the four-condition comparison.

The Franka offline audit found that logged dq disagrees in sign and magnitude
with finite differences of q before sleep. Opt-in source qualifications for
both controls recorded full DOF arrays, pre/post-step state, actual clock,
sleep flags, and wrapper/direct tensor reads. Franka resolved nine DOFs;
Robotiq resolved 13 (seven arm, one active driver, five passive). Both
interfaces returned the same arrays and selected indices, but both read the
same PhysX tensor backend; their agreement is not independent velocity
validation. On awake steps 1–24, q5 backward-difference versus logged-dq
RMSE is **0.0020247 rad/s** (Franka) and **0.0049055 rad/s** (Robotiq), with
zero sign agreement among 18 and 23 materially moving samples respectively
(both magnitudes greater than 1e-4 rad/s). Logged dq is therefore left
unmodified and is not used to infer delivered impulse or response velocity.

## Four-condition position comparison

The accepted archives all report `status: complete`, 91 samples, identical
initial seven-arm q/dq and link-8 tensor pose, unchanged arm targets, and
the approved protocol hash. Within each assembly, the pulse and control
resolved-scene configuration hashes match; across assemblies, they differ
as expected. All four have zero reported contact events. Each control has
zero force-application records; each pulse has exactly one successful
step-10 `"Impulse"` **dispatch** record. The pulse and control q arrays
are bitwise identical on steps 0–9 for all seven joints.

For assembly A and joint j, the reported response is
`Δq_A,j(k) = q_pulse,A,j(k) − q_control,A,j(k)`. The between-assembly contrast
is `ΔΔq_j(k) = Δq_Robotiq,j(k) − Δq_Franka,j(k)`. In the table, each signed
peak is the sample with the **maximum absolute** value of that contrast;
the step gives its location (time = step/60 s). Units are radians.

| Joint | Franka signed peak @ step | Franka final | Robotiq signed peak @ step | Robotiq final | Between-assembly signed peak @ step | Between-assembly final |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | −3.41208e-7 @ 34 | −3.41208e-7 | +7.30610e-7 @ 10 | −5.59554e-7 | +8.06922e-7 @ 11 | −2.18346e-7 |
| 2 | +3.29316e-5 @ 10 | +2.00570e-5 | +3.31104e-5 @ 10 | +1.86563e-5 | −1.40071e-6 @ 34 | −1.40071e-6 |
| 3 | +8.23115e-7 @ 34 | +8.23115e-7 | +1.00463e-6 @ 34 | +1.00463e-6 | +1.81513e-7 @ 34 | +1.81513e-7 |
| 4 | −4.50611e-5 @ 10 | −1.02520e-5 | −5.48363e-5 @ 10 | −1.32322e-5 | −9.77516e-6 @ 10 | −2.98023e-6 |
| **5** | **+1.07125e-5 @ 10** | **+2.47607e-6** | **+1.66281e-5 @ 10** | **+5.89935e-6** | **+5.91563e-6 @ 10** | **+3.42328e-6** |
| 6 | −2.38419e-7 @ 10 | −1.19209e-7 | −1.19209e-7 @ 10 | 0 | +1.19209e-7 @ 10 | +1.19209e-7 |
| 7 | −2.36745e-9 @ 10 | +5.10559e-11 | +9.74906e-8 @ 34 | +9.74906e-8 | +9.74395e-8 @ 34 | +9.74395e-8 |

At step 10, the q5 contrast is +1.07125e-5 rad for Franka and
+1.66281e-5 rad for Robotiq, a between-assembly difference of
+5.91563e-6 rad. At step 90 the corresponding contrasts are +2.47607e-6
and +5.89935e-6 rad. The Robotiq control alone drifts to +4.69653e-4 rad
in q5, versus +1.93964e-4 rad for Franka; these *absolute control drifts*
are not pulse-induced motion. Joint 7 contrasts approach numerical-noise
scale and warrant no mechanical interpretation.

Direct sleep measurements exist for the Franka sleep-qualified control and
both velocity-qualified controls: all first report link-8 asleep at step 25.
The ordinary Franka and both pulse archives have no direct sleep flag. Their
q/dq become exactly constant through the remaining samples from step 25
(ordinary Franka control) or step 34 (both pulse runs), consistent with—but
not direct proof of—sleep in those archives. Consequently the step-90 offset
mixes pulse response with different freeze times; the immediate step-10
contrast is the cleaner position-level comparison.

## Interpretation and limits

**Observation:** the prescribed contact-free impulse dispatch coincided with
small pulse-minus-control q5 displacements in both assemblies. Robotiq's
peak was larger by +5.91563e-6 rad in this single pair of runs, but neither
assembly reproduced the earlier +0.130094 rad collision-associated Robotiq
q5 shift. This establishes an observed position-level assembly contrast
under B1 conditions, not its statistical robustness or physical cause.

**Limits:** successful API dispatch does not independently verify the
delivered impulse vector or magnitude. The original q/dq inconsistency
persists in both assemblies; no sign, scaling, or index correction has been
validated. Both models emit the unresolved PhysX small-sphere inertia
fallback warning for `fr3_link8`, whose composed mass/inertia attributes
were unauthored/default in the preflights. Sleep truncates later motion.
B1 has no loaded fingertip/table contact and therefore cannot separate
contact geometry from contact dynamics. One execution per condition gives
no response distribution or repeatability bound. These data do not rule out
a CAD mounting error, an inertia issue, or a contact-constraint mechanism.

Two hypotheses remain open:

- **H-G — Geometry/kinematics:** an incorrect gripper mount transform,
  joint axis/origin or frame, or collision geometry changes the response
  during contact.
- **H-D — Assembly dynamics/contact:** different physical properties,
  articulation topology, or contact constraints change the response despite
  correct geometry.

The next investigation will examine **H-G first**, by read-only inspection
of the composed model and CAD correspondence. No model correction is implied
by B1 alone.

## Artifact index and reproducibility

These repository-relative paths were checked to exist at closeout. Except
for this Markdown document, they are under Git-ignored `outputs/` and are
**not contained in the tracked closeout**. Retain or archive the raw files,
console/startup logs, approval manifest, runner, protocol code, and analysis
separately to reproduce or re-audit the result. The SHA-256 values below
identify the key local inputs; they are not substitutes for those files.

| Artifact | Repository-relative path | SHA-256 |
| --- | --- | --- |
| Approval | `outputs/isaac_joint5_diagnostics/sim_p13g/b1_approval.json` | `5fc56c2eccf6afaa600e2b93ef94a1d03524cb931992aa1798c72e9d0285ae4d` |
| Franka control | `outputs/isaac_joint5_diagnostics/sim_p13g/b1_franka_control/raw.json` | `fdf1e5b76735e00d82f38ac1904e6e43033a81493be17fc4996ea7d087225da4` |
| Franka direct sleep qualification | `outputs/isaac_joint5_diagnostics/sim_p13g/b1_franka_control_sleep_qualification_v2/raw.json` | `bcffe7bf7749f873007ceca244af4ade2b682843bbea92fbac526684d26c1feb` |
| Franka pulse | `outputs/isaac_joint5_diagnostics/sim_p13g/b1_franka_pulse_v2/raw.json` | `b7575df34623e6627feb3077bda8c40e07ee2fa931232f4a0105d23826bbfc79` |
| Franka velocity qualification | `outputs/isaac_joint5_diagnostics/sim_p13g/b1_franka_velocity_qualification_v1/raw.json` | `3baf7f172de4b6b20bb0ab983081e85d7c80cdf08a7f0c624f5671d22f224ea1` |
| Robotiq control | `outputs/isaac_joint5_diagnostics/sim_p13g/b1_robotiq_control_velocity_v1/raw.json` | `d9805160e67e1ebdf806fbaeef9aa029dc922fbcfa566f5532056bbc1647a389` |
| Robotiq pulse | `outputs/isaac_joint5_diagnostics/sim_p13g/b1_robotiq_pulse_v1/raw.json` | `003c7d28738b94e96b470e9ee3779e5c7b3711144466e3ad7b3a29171f4f9909` |
| Four-run offline calculation | `outputs/isaac_joint5_diagnostics/sim_p13g/b1_closeout_analysis.json` | `0d9c71eb4f016b6ca554c70eed372c6bba51d5264aaba8b36ba9e66b5a49fd0b` |

Additional checked local evidence:

- `outputs/isaac_joint5_diagnostics/sim_p13g/b1_franka_offline_audit/report.md`
- `outputs/isaac_joint5_diagnostics/sim_p13g/stage_a_report.md`
- `outputs/isaac_joint5_diagnostics/sim_p13g/analysis.json` (Stage A numerical evidence)
- `outputs/isaac_joint5_diagnostics/sim_p13g/b1_preflight_franka.json`
- `outputs/isaac_joint5_diagnostics/sim_p13g/b1_preflight_robotiq.json`
- `outputs/isaac_joint5_diagnostics/sim_p13g/b1_franka_control_sleep_qualification/raw.json` (failed sleep-API attempt)
- `outputs/isaac_joint5_diagnostics/sim_p13g/b1_franka_pulse/raw.json` (failed force-API attempt)
- `outputs/isaac_joint5_diagnostics/sim_p13g/b1_protocol.py`
- `outputs/isaac_joint5_diagnostics/sim_p13g/run_common_wrench.py`
- `outputs/isaac_joint5_diagnostics/sim_p13g/analyze_b1.py`

The accepted pair calculations can be recomputed with `analyze_b1.py`
using the four indexed run directories and a **new** output JSON path. That
analyzer also emits raw logged-dq contrasts; those are not interpreted here
because the velocity source is unqualified. The all-joint between-assembly
table above uses the explicit difference of the two position contrasts.
