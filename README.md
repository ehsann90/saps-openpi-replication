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
| DROID joint-to-FR3 Cartesian embodiment | M2 complete and validated offline |
| Live FR3 observation and spnavd input | M3 complete and live-validated without actuation |
| Fixed physical-robot SAPS baseline | Later milestone; not yet implemented |
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

The next target is ordinary chunked π0.5/SAPS deployment on a fixed physical
robot. Planned components include a fixed-arm interface, available fixed and
wrist/external cameras, SpaceMouse Cartesian correction, Fixed `alpha = 0.5`,
Cosine `k = 6`, SAPS-consistent gripper arbitration, and complete latency,
policy-wait, operator, policy, and executed-action logging.

Offline policy inference, FR3 embodiment kinematics, and non-actuating live
input diagnostics are implemented. A second D435I is temporarily assigned as
the exterior camera for M3 acceptance; this is not a permanent scientific
camera-role assumption. Actuation is not implemented.
Physical safety must be independent of
learned confidence and shared autonomy, using robot-native supervision,
workspace and velocity limits, collision or force/torque monitoring where
available, and an emergency stop. The concrete baseline specification is in
[Simulation SAPS Baseline](docs/simulation_saps_baseline.md#physical-saps-next-stage).

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
- [Offline DROID-to-FR3 embodiment milestone M2](docs/physical_fr3_embodiment.md)
- [Live observation and SpaceMouse milestone M3](docs/physical_m3_inputs.md)
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
tracked below `results/gate2_shared_autonomy_pilot_v2`. Other generated analysis
products remain ignored unless deliberately selected as a reviewable archive.
Raw outputs must never be edited to change an outcome or provenance record.

## Citation

When using this replication, cite the original SAPS paper and identify the
exact repository milestone, collection commit, analysis commit, submodule
revisions, checkpoint, and frozen protocol hashes. A machine-readable citation
template is provided in [CITATION.cff](CITATION.cff).
