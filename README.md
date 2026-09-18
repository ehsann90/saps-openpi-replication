# SAPS–OpenPI Replication

This repository provides a reproducible reference implementation of **SAPS:
Shared Autonomy for Policy Steering by Blending Teleoperation with a Pretrained
VLA** using OpenPI π0.5, LIBERO, Robosuite, MuJoCo, and a SpaceMouse operator
interface.

It is not an official implementation from the SAPS or OpenPI authors. Its role
is to establish transparent shared-autonomy behavior and deployment
infrastructure: deterministic policy sampling, controlled perturbations,
explicit arbitration, responsive operator input, latency-aware execution,
separate policy/operator/executed-action logs, and reproducible analysis.

The completed LIBERO work is a simulation baseline for physical deployment. It
is not presented as a full numerical reproduction of SAPS or as an independent,
powered comparison of arbitration methods.

## Project status

| Stage | Status |
|---|---|
| π0.5/OpenPI deterministic LIBERO baseline | Complete and validated |
| Autonomous perturbation and runtime characterization | Complete |
| Keyboard and calibrated SpaceMouse input | Complete and physically validated |
| Hard takeover, Fixed, and Cosine arbitration | Complete and validated |
| Matched LIBERO shared-autonomy pilot | Complete: 60/60 outcomes, 20/20 exact triplets, analysis valid |
| π0.5-DROID offline physical-policy integration | M1 complete and validated |
| Manual FR3 FK/Jacobian and DROID finite-action mapping | Validated offline |
| Live FR3 observation and spnavd input | M3 complete and live-validated |
| FR3 measured-state-anchored policy execution | C1-C2 physically validated, including repeated 8-action chunks at 15 Hz |
| DROID binary gripper semantics and FR3 Move/Grasp realization | G1B physically validated, including unsupported grasp/hold/release |
| Physical task-directed π0.5 behavior | G2 pending; object-directed behavior remains unresolved |
| Physical shared autonomy and operator blending | Not yet implemented |
| Risk, collaboration, and intervention-learning research | Planned after the physical baseline |

The matched pilot contains 20 autonomous, 20 Fixed, and 20 Cosine outcomes
across four selected perturbation conditions. It is a descriptive excluded
pilot with one task, one operator, and five repetitions per condition-mode cell.
See the [canonical simulation-baseline archive](docs/simulation_saps_baseline.md)
and its [generated final report](results/gate2_shared_autonomy_pilot_v2/REPORT.md).

## SAPS reference baseline

The implemented action-level comparison conditions are:

- autonomous policy execution;
- pure teleoperation;
- hard takeover;
- Fixed blending with active-human autonomy weight `0.5`;
- Cosine blending with logistic gain `k = 6`.

Actions have six end-effector motion dimensions and one gripper dimension. For
Fixed and Cosine modes, motion follows:

```text
executed_motion = alpha * autonomous_motion + (1 - alpha) * human_motion
```

The gripper is arbitrated independently with the SAPS closing-biased `max()`
rule under this repository's `-1=open`, `+1=close` convention. The
[shared-autonomy runtime guide](docs/shared_autonomy.md) defines every mode,
wait state, boundary condition, and logging field.

## Reproducibility baseline

| Dependency | Frozen identity |
|---|---|
| OpenPI | `15a9616a00943ada6c20a0f158e3adb39df2ccac` |
| LIBERO | `f78abd68ee283de9f9be3c8f7e2a9ad60246e95c` |
| Policy configuration | `pi05_libero` |
| Checkpoint | `gs://openpi-assets/checkpoints/pi05_libero` |
| Task suite | `libero_object` |
| Task | `pick up the cream cheese and place it in the basket` |

OpenPI is pinned as `third_party/openpi`; LIBERO is pinned recursively below it.
Project-specific compatibility changes are documented patches, not silent edits
to either dependency.

Gripper-enabled FR3 execution additionally requires the clean
`frdedynamics/fr3_lab_stack` commit
`4bb6cdc58839dcdd94acbe1633b8f361676a4eb6`. The G1B integration inspected
local `franka_ros2` source revision
`1369a2cb200d0f7b3da11c7728c7ca2e6975ca00`; no upstream `franka_ros2`
modification is part of the SAPS G1B implementation.

## Simulation validation

The repository progressed from autonomous π0.5 deployment through controlled
robustness and latency characterization, then SpaceMouse integration, Fixed and
Cosine arbitration, and finally a matched descriptive pilot. The completed
pilot observed autonomous degradation at larger selected object-position
offsets and several recoveries under human steering. These observations validate
the shared-autonomy pipeline; they do not establish statistical superiority or
causal explanations for recovery.

Simulation policy waits pause robot/environment state and simulation time while
wall clock advances. For a conventional physical chunked-VLA baseline, the
robot may hold or stop while waiting, but wall clock and the external physical
environment continue. Continuous execution methods such as real-time chunking
are outside the current baseline.

## Physical deployment

The fixed-arm physical runtime now has separately validated arm and gripper
mechanisms.

C1-C2 physically validates ordinary chunked π0.5/DROID arm execution through the
persistent FR3 streaming impedance path: a discarded cold-start warm-up, fresh
measured-q pre-inference holds, native `[15,8]` responses, actions `0..7` at
independently scheduled 15 Hz deadlines, fresh measured-state anchoring, terminal
holds, delivery audits, readiness reacquisition, and Franka health checks.

G1B validates the DROID binary gripper contract and its FR3 realization. The
strict policy rule is `action[7] > 0.5` → CLOSED and otherwise OPEN. OPEN maps to
`Move(maximum_width)`; CLOSED maps to an object-agnostic `Grasp(width=0)` with
the fixed validated embodiment parameters. Object width is not supplied to the
adapter. The qualifying physical run captured and held an unsupported object for
approximately three seconds, then performed the explicit Stop → Move(open)
release sequence.

These are runtime-mechanism validations, not manipulation-task-success results.
The next gate is G2: the current physical π0.5 behavior does not yet reliably
approach and interact with the intended object. Task-level SUCCESS/FAILURE
detection and physical SAPS operator blending also remain future work.

Physical safety remains independent of learned confidence and shared autonomy,
using robot-native supervision, joint/workspace and timing checks, collision or
force/torque monitoring where available, and an emergency stop. See
[the C1-C2 runtime record](docs/physical_c1c2_runtime.md) and
[the G1B gripper record](docs/physical_g1b_gripper.md).

## Research extensions

After the physical baseline is stable, the research direction is:

1. short-horizon autonomous-continuation risk;
2. evidence of collaboration, recovery, assistance reduction, and autonomy
   resumption;
3. selective learning from intervention.

These are future research questions, not claims supported by the LIBERO pilot.

## Quick start

Clone the pinned repository and submodules:

```bash
git clone --recurse-submodules \
  https://github.com/ehsann90/saps-openpi-replication.git
cd saps-openpi-replication
git submodule status
```

Apply the documented compatibility patch, build the validated Docker images,
and run automated checks:

```bash
make apply-patch
make build-images
make check
```

For autonomous or shared-autonomy development runs, start the policy server in
one terminal:

```bash
make policy-server
```

Then use a unique output identity in another terminal, for example:

```bash
make autonomous-smoke CONDITION=nominal
make takeover CONDITION=nominal TRIAL=0
make fixed-blend CONDITION=nominal TRIAL=0 FIXED_AUTONOMY_WEIGHT=0.5
make cosine-blend CONDITION=nominal TRIAL=0 COSINE_GAIN=6.0
```

For supervised FR3 work, start the required robot/controller, cameras, and
π0.5/DROID policy server first. The convenience targets preserve ROS's
`PYTHONPATH` and keep arm-only, isolated-gripper, and arm+gripper outputs
separate:

```bash
# Re-run the stationary-arm G1B mechanism validation.
make physical-g1b-grasp-release

# Validated C1-C2 arm-only policy execution.
make physical-c1c2 \
  PHYSICAL_RUN_ID=<unique> \
  PHYSICAL_PROMPT='<instruction>'

# Explicit opt-in arm + physical gripper execution.
make physical-c1c2-gripper \
  PHYSICAL_RUN_ID=<unique> \
  PHYSICAL_PROMPT='<instruction>'
```

`physical-c1c2-gripper` uses the same validated fixed Grasp parameters as G1B
and requires the clean pinned `fr3_lab_stack` revision. It is available as the
integrated runtime path, but current task-level policy behavior remains a G2
investigation; do not treat a run as manipulation-task success merely because
the transport, arm, and gripper mechanisms execute correctly.

The completed matched-pilot roots are frozen. Do not reuse their experiment IDs
or output directories. Regenerate the read-only derived archive with:

```bash
make gate2-analysis
```

## Documentation

Project overview and archive:

- [Simulation SAPS baseline](docs/simulation_saps_baseline.md)
- [Environment and dependency baseline](docs/environment-baseline.md)
- [Repository structure and data policy](docs/repository_structure.md)
- [Branch inventory at simulation closeout](docs/branch_inventory.md)

Reference implementation:

- [Installation and environment setup](docs/setup.md)
- [Command runbook](docs/runbook.md)
- [Shared-autonomy semantics and runtime](docs/shared_autonomy.md)
- [Keyboard and SpaceMouse input](docs/human_input.md)
- [Testing and validation](docs/testing.md)
- [Offline π0.5-DROID physical milestone M1](docs/physical_pi05_droid.md)
- [Validated FR3 kinematics and DROID action mapping](docs/physical_fr3_embodiment.md)
- [Live observation and SpaceMouse milestone M3](docs/physical_m3_inputs.md)
- [Repeated physical arm runtime C1-C2](docs/physical_c1c2_runtime.md)
- [DROID-to-FR3 physical gripper integration G1B](docs/physical_g1b_gripper.md)
- [G1B qualifying physical validation archive](docs/validation/2026-09-18_g1b_grasp_release/README.md)
- [Analysis tools and interpretation limits](docs/analysis.md)

Archived lower-level records:

- [Matched-pilot frozen protocol](docs/gate2_operator_pilot.md)
- [Latency and scheduler characterization](docs/gate1_rtx5080_ac_performance.md)
- [Autonomous perturbations and deterministic sampling](docs/phase1_libero_perturbations_and_determinism.md)
- [Reusable operator-session protocol](docs/experiment_protocol.md)

Historical experiment identifiers such as `gate1`, `gate2`, and `gate2_v2` are
retained in immutable protocol, command, and output paths for provenance. New
documentation uses descriptive stage names.

## Output and archive policy

Raw episodes live below `outputs/` and remain outside Git. The completed frozen
roots are:

```text
outputs/gate2_shared_autonomy_pilot_v2
outputs/gate2_autonomous_pilot_v2
```

Small validated derived tables and reports for the completed baseline are
tracked below `results/gate2_shared_autonomy_pilot_v2`. Raw physical runs also
remain under `outputs/`; selected qualifying evidence may be copied verbatim
into `docs/validation/` with checksums and a scope-limited validation record.
Other generated analysis products remain ignored unless deliberately selected
as a reviewable archive. Raw outputs must never be edited to change an outcome
or provenance record.

## Citation

When using this replication, cite the original SAPS paper and identify the
exact repository milestone, collection commit, analysis commit, submodule
revisions, checkpoint, and frozen protocol hashes. A machine-readable citation
template is provided in [CITATION.cff](CITATION.cff).
