# Repository Structure

```text
saps-openpi-replication/
├── CITATION.cff
├── Makefile
├── README.md
├── compose.yml
├── configs/
│   └── libero_cream_cheese_offsets.json
├── docs/
│   ├── analysis.md
│   ├── experiment_protocol.md
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
│   ├── run_autonomous_sweep.py
│   ├── run_libero.py
│   ├── run_shared_autonomy_episode.py
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

## Project scripts

- `serve_seeded_policy.py`: deterministic OpenPI server wrapper.
- `run_libero.py`: one autonomous episode.
- `run_autonomous_sweep.py`: resumable autonomous condition sweep.
- `run_teleoperation_episode.py`: pure browser-controlled LIBERO episode.
- `run_shared_autonomy_episode.py`: autonomous, takeover, fixed, or cosine
  arbitration through the asynchronous shared-control runtime.

The future Phase 3 operator-session runner will also live in `scripts/` and will
orchestrate these existing episode entry points rather than duplicate their
control logic.

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

Keyboard mapping and the browser operator server.

### `src/saps/policies`

OpenPI client integration, deterministic seeding, action chunks, and the
asynchronous policy worker.

## Tests

- `tests/unit/`: non-interactive regression and integration tests.
- `tests/manual/`: interactive browser/operator smoke tests.

Formal experiment episodes are not unit tests and do not belong in the Git
history.

## Tools

- `tools/diagnostics/`: scene inspection, perturbation previews, and deterministic
  policy probes.
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
