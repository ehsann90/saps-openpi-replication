# Repository Guidelines

## Project Purpose

This repository implements shared-autonomy policy arbitration and evaluation for
robotic manipulation, including autonomous control, operator takeover,
fixed-weight blending, and cosine-scheduled blending.

Prioritize:

1. Correct and explicit arbitration behavior.
2. Deterministic and reproducible evaluation.
3. Safe handling of operator input and stale policy results.
4. Minimal, reviewable changes.
5. Preservation of the validated Docker-based runtime.

Do not alter experimental protocols, evaluation semantics, arbitration
equations, seed handling, or output schemas without explicitly identifying the
impact.

## Project Structure

* `src/saps/`: core Python package.

  * Keep arbitration, environments, evaluation, human input, and policy clients
    in their existing domains.
* `scripts/`: user-facing entry points and experiment launchers.
* `tools/`: analysis, diagnostics, migration, and maintenance utilities.
* `tests/unit/`: automated `unittest` tests named `test_*.py`.
* `tests/manual/`: interactive or hardware-dependent checks.
* `docs/`: architecture, protocols, operation, and testing documentation.
* `configs/`: version-controlled runtime and experiment configuration.
* `patches/`: project-specific compatibility changes for pinned dependencies.
* `third_party/openpi/`: pinned OpenPI submodule.
* `outputs/`: generated run artifacts.
* `results/`: derived analyses and reports.

Do not place reusable application logic in `scripts/` or `tools/` when it belongs
in `src/saps/`.

Treat `third_party/openpi/` as externally owned and pinned. Do not silently edit
files inside it. Express project-specific upstream changes as documented patches
under `patches/`, unless the task explicitly requires updating the submodule.

Do not treat files in `outputs/` or `results/` as source code. Avoid modifying or
committing generated artifacts unless the task specifically requires a small,
reviewable fixture or representative result.

## Working Method

Before editing:

1. Read the relevant implementation, tests, configuration, and documentation.
2. Inspect neighboring code for established patterns.
3. Identify whether the change affects runtime behavior, experimental protocol,
   reproducibility, output formats, or third-party compatibility.
4. Check the working tree and preserve unrelated user changes.

During implementation:

* Make the smallest coherent change that satisfies the task.
* Do not refactor unrelated code.
* Preserve public interfaces and output schemas unless a change is required.
* Update tests and documentation alongside behavioral changes.
* Prefer fixing the cause of an error over suppressing validation or warnings.
* Do not introduce dependencies without explaining why the existing stack is
  insufficient.
* Do not commit, push, open pull requests, or modify remote resources unless
  explicitly requested.

When requirements are ambiguous and the alternatives would materially affect
behavior, experimental validity, or compatibility, stop and ask for direction.

## Build and Development Commands

Most commands run inside Docker to preserve the validated dependency stack.

* `make help`: list supported commands, overrides, and diagnostics.
* `make build-images`: build the pinned LIBERO runtime and OpenPI server images.
* `make policy-server`: start the deterministic policy server.
* `make policy-stop`: stop the policy server.
* `make compile`: compile-check project Python files.
* `make unit-test`: run `unittest` discovery under `tests/unit/`.
* `make check`: run unit tests and compile checks.
* `make autonomous-smoke CONDITION=nominal`: run one autonomous validation.
* `make operator-smoke DURATION=60`: validate browser input without LIBERO.
* `make takeover`: run operator-takeover shared autonomy.
* `make fixed-blend`: run fixed-weight blending.
* `make cosine-blend`: run cosine-scheduled blending.

Use unique `TRIAL` or `SHARED_OUTPUT` values for interactive or experimental
runs. Never overwrite an existing run unless explicitly instructed.

Do not automatically run `make build-images`, start persistent services, invoke
GPU-dependent tasks, or launch interactive shared-autonomy modes merely to
validate a small source change. These actions may be expensive, stateful, or
require human supervision. Ask first when their necessity or environment
availability is unclear.

If starting the policy server for an approved task, stop it afterward with
`make policy-stop`, unless the user asks to leave it running.

## Verification Strategy

Use the narrowest adequate verification first:

1. Run focused unit tests for the changed behavior.
2. Run `make compile` for Python source changes.
3. Run `make check` before considering a normal code change complete.
4. Run autonomous, operator, GPU-dependent, or interactive smoke tests only
   when relevant and when the required environment is available.
5. Follow additional procedures in `docs/testing.md`.

If a required check cannot run, report:

* The exact command attempted.
* The relevant error or environmental limitation.
* What was verified successfully.
* What remains unverified.

Do not claim a smoke test, manual check, or experiment succeeded unless it was
actually executed and its result inspected.

## Coding Style

Use:

* Python 3 type annotations for new and modified interfaces.
* Four-space indentation.
* `snake_case` for modules, functions, methods, and variables.
* `PascalCase` for classes.
* `UPPER_CASE` for constants.
* Concise docstrings that explain intent, assumptions, or non-obvious behavior.
* Small, single-purpose functions.
* Readable lines, generally around 79 characters.

No standalone formatter or linter is configured. Match neighboring code and
avoid formatting unrelated lines.

Preserve the repository's explicit validation, structured logging, error
handling, and configuration patterns. Avoid broad exception handling, hidden
fallbacks, and silent coercion of invalid inputs.

Use comments to explain domain reasoning or safety constraints, not to restate
the code.

## Shared-Autonomy and Arbitration Requirements

Treat arbitration and state-transition logic as safety- and protocol-sensitive.

When changing these areas:

* State the governing equation or transition rule explicitly.
* Preserve units, ranges, clipping behavior, and boundary conditions.
* Verify behavior at blend weights `0` and `1` where applicable.
* Test transitions into and out of operator control.
* Test stale, missing, delayed, or invalid policy results.
* Preserve deterministic seed propagation.
* Avoid reliance on wall-clock timing when deterministic simulation time is
  available.
* Do not change defaults silently.
* Document any change that could alter recorded trajectories or evaluation
  outcomes.

Separate policy outputs, arbitration decisions, operator inputs, and environment
actions clearly. Do not obscure which component produced the executed action.

## Testing Guidelines

Tests use the standard-library `unittest` framework.

* Name test files `test_*.py`.
* Name test methods `test_<behavior>`.
* Keep unit tests deterministic and independent.
* Avoid network, browser, Docker, GPU, or human-input requirements in unit tests.
* Add regression coverage for every corrected defect.

When relevant, cover:

* Arbitration equations and boundary values.
* State-machine transitions.
* Deterministic seeds and repeatability.
* Stale-result rejection.
* Invalid inputs and configuration.
* Timeouts and delayed policy responses.
* Operator takeover and release behavior.
* Output-schema compatibility.

Interactive checks belong in `tests/manual/` and must not be presented as
automated unit coverage.

Pilot or smoke-test output is diagnostic evidence, not a formal experimental
result. Do not generalize from it or place it in `results/` as validated research
evidence.

## Reproducibility and Data Integrity

Treat configuration, seed, condition, trial identity, software version, and
output location as part of an experiment's provenance.

* Do not reuse a trial identifier for a different configuration.
* Do not edit generated run data to make a result appear successful.
* Do not delete or overwrite outputs unless explicitly requested and the target
  has been verified.
* Keep raw outputs separate from derived reports.
* Record transformations used to produce files under `results/`.
* Flag changes that make new output incomparable with previous runs.
* Do not label exploratory, pilot, smoke-test, or manually altered data as a
  formal result.

When modifying configuration defaults, explain both the previous and new
behavior and identify affected commands or protocols.

## Documentation Requirements

Update documentation when changing:

* User-facing commands or configuration.
* Environment or Docker requirements.
* Arbitration behavior.
* Experimental procedures.
* Output schemas or directory conventions.
* Manual testing steps.
* OpenPI patches or compatibility assumptions.

Documentation must describe implemented behavior. Do not document planned
behavior as if it already exists.

## Secrets and Sensitive Data

Never commit or expose:

* API keys, access tokens, or credentials.
* Populated `.env` files.
* Private SSH keys.
* Machine-specific caches.
* Participant-identifiable or otherwise sensitive research data.
* Unreviewed large run outputs.

Do not print secret-bearing environment variables or copy credentials into
logs, patches, issues, or chat responses. Use placeholders in examples.

## Commit and Pull Request Guidelines

Use short, imperative commit subjects, for example:

```text
Add fixed-weight shared autonomy blending
```

Keep each commit focused. Include tests and documentation with behavioral
changes.

A pull request should include:

* Motivation and scope.
* Summary of the implementation.
* Verification commands and results.
* Known limitations or unverified behavior.
* Reproducibility, protocol, or output-schema impacts.
* Related issues.
* Screenshots or representative artifact paths for browser, visualization, or
  experiment-output changes.

Do not include generated bulk output in a pull request unless it is explicitly
required and reviewed.

## Completion Checklist

Before reporting a task complete:

* Confirm the requested behavior is implemented.
* Review the final diff for unrelated changes.
* Run focused tests and `make check`, when available and relevant.
* Confirm no credentials or unintended artifacts were added.
* Update documentation for user-visible or protocol-level changes.
* Identify reproducibility or compatibility implications.
* Report modified files, verification performed, and remaining limitations.
