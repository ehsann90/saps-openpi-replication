# Repository Structure

```text
saps-openpi-replication/
├── CITATION.cff
├── Makefile
├── README.md
├── compose.yml
├── configs/
│   ├── gate2_operator_pilot_manifest.json
│   ├── gate2_shared_autonomy_pilot_manifest.json
│   ├── gate2_autonomous_pilot_protocol.json
│   ├── libero_cream_cheese_offsets.json
│   ├── operator_shared_autonomy_manifest.json
│   ├── operator_teleoperation_manifest.json
│   └── spacemouse_profile.json
├── docs/
│   ├── analysis.md
│   ├── environment-baseline.md
│   ├── experiment_protocol.md
│   ├── gate1_rtx5080_ac_performance.md
│   ├── gate2_operator_pilot.md
│   ├── human_input.md
│   ├── phase1_libero_perturbations_and_determinism.md
│   ├── repository_structure.md
│   ├── runbook.md
│   ├── setup.md
│   ├── shared_autonomy.md
│   └── testing.md
├── patches/
│   ├── apply_openpi_patch.sh
│   └── openpi-libero-python38-build.patch
├── scripts/
│   ├── preflight_gate2_operator_pilot.py
│   ├── preflight_gate2_autonomous_pilot.py
│   ├── run_autonomous_sweep.py
│   ├── run_libero.py
│   ├── run_operator_experiment.py
│   ├── run_shared_autonomy_episode.py
│   ├── run_spacemouse_calibration.py
│   ├── run_teleoperation_episode.py
│   └── serve_seeded_policy.py
├── src/saps/
│   ├── arbitration/
│   ├── environments/
│   ├── evaluation/
│   ├── human_input/
│   └── policies/
├── tests/
│   ├── manual/
│   └── unit/
├── third_party/
│   └── openpi/                 # pinned Git submodule
└── tools/
    ├── analysis/
    ├── diagnostics/
    └── monitoring/
```

## Root files

- `Makefile`: stable entry points for setup, testing, experiments, diagnostics,
  and analysis.
- `compose.yml`: outer runtime and deterministic policy-server services.
- `.gitmodules`: pinned OpenPI dependency location.
- `.gitignore`: excludes generated data, caches, and local tooling files.
- `CITATION.cff`: citation metadata for the replication and original paper.

## Configuration

`configs/libero_cream_cheese_offsets.json` is the canonical source for the task,
object names, and nominal plus perturbed planar offsets. Experiment scripts
should read this configuration rather than duplicate the values.

The operator manifests define immutable teleoperation and shared-autonomy
schedules. Gate-2 v2 has separate shared and autonomous protocol files; the old
Gate-2 v1 manifest is superseded history. `configs/spacemouse_profile.json` is the
portable, physically validated SpaceMouse calibration; runtime device paths
remain outside it.

## Project scripts

- `serve_seeded_policy.py`: deterministic OpenPI server wrapper.
- `preflight_gate2_operator_pilot.py`: non-launching fixed-protocol validation
  and deterministic Gate-2 v2 shared/matched-design report.
- `preflight_gate2_autonomous_pilot.py`: non-launching 20-row autonomous
  protocol report.
- `run_libero.py`: one autonomous episode.
- `run_autonomous_sweep.py`: resumable autonomous condition sweep.
- `run_operator_experiment.py`: manifest-driven, resumable operator sessions.
- `run_teleoperation_episode.py`: pure browser-controlled LIBERO episode.
- `run_shared_autonomy_episode.py`: autonomous, takeover, fixed, or cosine
  arbitration through the asynchronous shared-control runtime.
- `run_spacemouse_calibration.py`: disposable graphical SpaceMouse calibration
  and profile writer.

## Python package

### `src/saps/arbitration`

Pure action-level arbitration and structured result logging.

### `src/saps/environments`

LIBERO task creation, object identification, perturbations, and observation
handling.

### `src/saps/evaluation`

Autonomous and operator-assisted episode loops, output helpers, asynchronous
policy scheduling, and shared-control state transitions.

### `src/saps/human_input`

Keyboard mapping, SpaceMouse discovery and processing, calibration profiles,
normalized human-input samples, and the browser operator server.

### `src/saps/policies`

OpenPI client integration, deterministic seeding, action chunks, and the
asynchronous policy worker.

## Tests

- `tests/unit/`: non-interactive regression and integration tests.
- `tests/manual/`: interactive browser/operator smoke tests.

Formal experiment episodes are not unit tests and do not belong in the Git
history.

## Tools

- `tools/diagnostics/`: SpaceMouse inspection, scene inspection, perturbation
  previews, and deterministic policy probes.
- `tools/analysis/`: stable analysis entry points.
- `tools/monitoring/`: development-oriented monitoring utilities. Files here are
  not automatically part of the supported public CLI.

## Generated data

```text
outputs/   # episodes, videos, schedules, manifests
results/   # tables, validation reports, plots
```

Both directories are ignored by Git. Formal datasets should be archived with
checksums and provenance outside the working clone.
