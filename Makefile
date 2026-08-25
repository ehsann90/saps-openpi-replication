SHELL := /bin/bash

LOCAL_UID := $(shell id -u)
LOCAL_GID := $(shell id -g)

COMPOSE := LOCAL_UID=$(LOCAL_UID) LOCAL_GID=$(LOCAL_GID) docker compose -f compose.yml
RUNTIME := $(COMPOSE) run --rm --no-deps runtime

CONDITION ?= nominal
CONDITION_IDS ?=
CONDITION_IDS_ARG = $(if $(strip $(CONDITION_IDS)),--condition-ids $(CONDITION_IDS),)

TRIAL ?= 0
INITIAL_STATE ?= 0
NUM_TRIALS ?= 1
TELEOP_MAX_STEPS ?= 1800
AUTONOMOUS_MAX_STEPS ?= 280
DURATION ?= 180
SPEED_MODE ?= fine
INPUT_SOURCE ?= keyboard
SPACEMOUSE_DEVICE ?=
SPACEMOUSE_TRANSLATION_GAIN ?= 0.30
SPACEMOUSE_ROTATION_GAIN ?= 0.08
SPACEMOUSE_DEADZONE ?= 0.08
SPACEMOUSE_AXIS_MAPPING ?= ABS_X,ABS_Y,ABS_Z,ABS_RX,ABS_RY,ABS_RZ
SPACEMOUSE_AXIS_SIGNS ?= 1,1,1,1,1,1
SPACEMOUSE_AXIS_MAXIMA ?= 350,350,350,350,350,350
SPACEMOUSE_STALE_TIMEOUT ?= 0.25
SPACEMOUSE_OPEN_BUTTON ?= 256
SPACEMOUSE_CLOSE_BUTTON ?= 257
SPACEMOUSE_REFRESH_HZ ?= 20
SPACEMOUSE_PROFILE ?=
CALIBRATION_PROFILE ?= configs/spacemouse_profile.json
CALIBRATION_TRANSLATION_GAIN ?= 0.30
CALIBRATION_ROTATION_GAIN ?= 0.08
SPACEMOUSE_DEVICE_RUNNER_ARG = $(if $(strip $(SPACEMOUSE_DEVICE)),--spacemouse-device-path $(SPACEMOUSE_DEVICE),)
SPACEMOUSE_DEVICE_DIAGNOSTIC_ARG = $(if $(strip $(SPACEMOUSE_DEVICE)),--device-path $(SPACEMOUSE_DEVICE),)
SPACEMOUSE_PROFILE_ARG = $(if $(strip $(SPACEMOUSE_PROFILE)),--spacemouse-profile-path $(SPACEMOUSE_PROFILE),)

HUMAN_INPUT_ARGS = --input-source $(INPUT_SOURCE) $(SPACEMOUSE_DEVICE_RUNNER_ARG) $(SPACEMOUSE_PROFILE_ARG) --translation-gain $(SPACEMOUSE_TRANSLATION_GAIN) --rotation-gain $(SPACEMOUSE_ROTATION_GAIN) --spacemouse-deadzone $(SPACEMOUSE_DEADZONE) --spacemouse-axis-mapping $(SPACEMOUSE_AXIS_MAPPING) --spacemouse-axis-signs $(SPACEMOUSE_AXIS_SIGNS) --spacemouse-axis-maxima $(SPACEMOUSE_AXIS_MAXIMA) --spacemouse-stale-input-timeout-seconds $(SPACEMOUSE_STALE_TIMEOUT) --spacemouse-open-button $(SPACEMOUSE_OPEN_BUTTON) --spacemouse-close-button $(SPACEMOUSE_CLOSE_BUTTON)
SESSION_INPUT_ARGS = --input-source $(INPUT_SOURCE) $(SPACEMOUSE_DEVICE_RUNNER_ARG) $(SPACEMOUSE_PROFILE_ARG) --spacemouse-deadzone $(SPACEMOUSE_DEADZONE) --spacemouse-axis-mapping $(SPACEMOUSE_AXIS_MAPPING) --spacemouse-axis-signs $(SPACEMOUSE_AXIS_SIGNS) --spacemouse-axis-maxima $(SPACEMOUSE_AXIS_MAXIMA) --spacemouse-stale-input-timeout-seconds $(SPACEMOUSE_STALE_TIMEOUT) --spacemouse-open-button $(SPACEMOUSE_OPEN_BUTTON) --spacemouse-close-button $(SPACEMOUSE_CLOSE_BUTTON)

ENVIRONMENT_SEED ?= 7
POLICY_BASE_SEED ?= 20260724

TELEOP_OUTPUT ?= outputs/teleoperation_smoke
AUTONOMOUS_OUTPUT ?= outputs/autonomous_sweep
PROBE_OUTPUT ?= outputs/seeded_policy_probe
SCENE_OUTPUT ?= outputs/scene_inspection
PREVIEW_OUTPUT ?= outputs/perturbation_preview
ANALYSIS_OUTPUT ?= results/analysis
COMPARISON_OUTPUT ?= results/saps_libero_current
AUTONOMOUS_RESULTS ?= outputs/autonomous_deterministic_n20_state0_v1
TELEOP_RESULTS ?= outputs/saps_libero_teleoperation_v1
SHARED_RESULTS ?= outputs/saps_libero_shared_autonomy_v1
REDO_EPISODES ?=
REDO_EPISODES_ARG = $(if $(strip $(REDO_EPISODES)),--redo-episode-ids $(REDO_EPISODES),)
MANIFEST ?= configs/operator_shared_autonomy_manifest.json
SESSION_OUTPUT ?= outputs/operator_experiment
REPOSITORY_COMMIT := $(shell git rev-parse HEAD)

DX ?= 0.0
DY ?= 0.0
LABEL ?= preview
SUMMARY ?= outputs/autonomous_sweep/sweep_summary.json

.PHONY: help
help:
	@echo "Main processes:"
	@echo "  make build-images"
	@echo "  make policy-server"
	@echo "  make autonomous-smoke"
	@echo "  make autonomous-sweep NUM_TRIALS=20"
	@echo "  make teleop CONDITION=nominal TRIAL=4"
	@echo "  make takeover CONDITION=nominal TRIAL=0"
	@echo "  make fixed-blend FIXED_AUTONOMY_WEIGHT=0.5"
	@echo "  make cosine-blend COSINE_GAIN=6.0"
	@echo "  make operator-session MANIFEST=<manifest.json>"
	@echo "  make teleoperation-session"
	@echo "  make shared-autonomy-session"
	@echo
	@echo "Tests:"
	@echo "  make check"
	@echo "  make unit-test"
	@echo "  make operator-smoke"
	@echo "  make compile"
	@echo
	@echo "Diagnostics:"
	@echo "  make seeded-probe"
	@echo "  make scene-inspect"
	@echo "  make perturbation-preview DX=0.10 DY=0.08 LABEL=p02"
	@echo "  make spacemouse-diagnostic"
	@echo "  make spacemouse-calibrate"
	@echo "  make analyze SUMMARY=<sweep_summary.json>"
	@echo "  make analyze-comparison"
	@echo
	@echo "Common overrides:"
	@echo "  CONDITION, CONDITION_IDS, TRIAL, INITIAL_STATE"
	@echo "  NUM_TRIALS, TELEOP_MAX_STEPS, AUTONOMOUS_MAX_STEPS, SPEED_MODE"
	@echo "  INPUT_SOURCE, SPACEMOUSE_DEVICE, SPACEMOUSE_PROFILE"
	@echo "  TELEOP_OUTPUT, AUTONOMOUS_OUTPUT, SHARED_OUTPUT"
	@echo "  FIXED_AUTONOMY_WEIGHT, COSINE_GAIN"

.PHONY: apply-patch
apply-patch:
	./patches/apply_openpi_patch.sh

.PHONY: build-images
build-images:
	cd third_party/openpi && \
		docker compose -f examples/libero/compose.yml \
		build runtime openpi_server

.PHONY: policy-server
policy-server:
	$(COMPOSE) up openpi_server

.PHONY: policy-stop
policy-stop:
	docker compose -f compose.yml stop openpi_server

.PHONY: operator-smoke
operator-smoke:
	$(COMPOSE) run --rm --no-deps \
		-e SAPS_SCRIPT=/workspace/tests/manual/keyboard_operator_smoke.py \
		-e SAPS_RUNTIME_ARGS="--duration-seconds $(DURATION)" \
		runtime

.PHONY: operator-session
operator-session:
	@test -z "$$(git status --porcelain)" || \
		{ echo "Formal collection requires a clean repository."; exit 1; }
	$(COMPOSE) run --rm --no-deps \
		-e SAPS_SCRIPT=/workspace/scripts/run_operator_experiment.py \
		-e SAPS_RUNTIME_ARGS="--manifest-path $(MANIFEST) --repository-commit $(REPOSITORY_COMMIT) --output-dir $(SESSION_OUTPUT) $(SESSION_INPUT_ARGS) $(REDO_EPISODES_ARG)" \
		runtime

.PHONY: teleoperation-session
teleoperation-session:
	$(MAKE) operator-session \
		MANIFEST=configs/operator_teleoperation_manifest.json \
		SESSION_OUTPUT=outputs/saps_libero_teleoperation_v2

.PHONY: shared-autonomy-session
shared-autonomy-session:
	$(MAKE) operator-session \
		MANIFEST=configs/operator_shared_autonomy_manifest.json \
		SESSION_OUTPUT=outputs/saps_libero_shared_autonomy_v2

.PHONY: teleop
teleop:
	$(COMPOSE) run --rm --no-deps \
		-e SAPS_SCRIPT=/workspace/scripts/run_teleoperation_episode.py \
		-e SAPS_RUNTIME_ARGS="--condition-id $(CONDITION) --trial-index $(TRIAL) --initial-state-index $(INITIAL_STATE) --environment-seed $(ENVIRONMENT_SEED) --policy-base-seed $(POLICY_BASE_SEED) --max-steps $(TELEOP_MAX_STEPS) --default-speed-mode $(SPEED_MODE) $(HUMAN_INPUT_ARGS) --output-dir $(TELEOP_OUTPUT)" \
		runtime

.PHONY: spacemouse-diagnostic
spacemouse-diagnostic:
	$(COMPOSE) run --rm --no-deps \
		-e SAPS_SCRIPT=/workspace/tools/diagnostics/inspect_spacemouse_input.py \
		-e SAPS_RUNTIME_ARGS="$(SPACEMOUSE_DEVICE_DIAGNOSTIC_ARG) --translation-gain $(SPACEMOUSE_TRANSLATION_GAIN) --rotation-gain $(SPACEMOUSE_ROTATION_GAIN) --deadzone $(SPACEMOUSE_DEADZONE) --axis-mapping $(SPACEMOUSE_AXIS_MAPPING) --axis-signs $(SPACEMOUSE_AXIS_SIGNS) --axis-maxima $(SPACEMOUSE_AXIS_MAXIMA) --stale-input-timeout-seconds $(SPACEMOUSE_STALE_TIMEOUT) --open-button $(SPACEMOUSE_OPEN_BUTTON) --close-button $(SPACEMOUSE_CLOSE_BUTTON) --refresh-frequency-hz $(SPACEMOUSE_REFRESH_HZ)" \
		runtime

.PHONY: spacemouse-calibrate
spacemouse-calibrate:
	$(COMPOSE) run --rm --no-deps \
		-e SAPS_SCRIPT=/workspace/scripts/run_spacemouse_calibration.py \
		-e SAPS_RUNTIME_ARGS="$(SPACEMOUSE_DEVICE_DIAGNOSTIC_ARG) --profile-path $(CALIBRATION_PROFILE) --translation-gain $(CALIBRATION_TRANSLATION_GAIN) --rotation-gain $(CALIBRATION_ROTATION_GAIN) --deadzone $(SPACEMOUSE_DEADZONE) --stale-input-timeout-seconds $(SPACEMOUSE_STALE_TIMEOUT) --open-button $(SPACEMOUSE_OPEN_BUTTON) --close-button $(SPACEMOUSE_CLOSE_BUTTON)" \
		runtime

.PHONY: autonomous-smoke
autonomous-smoke:
	$(COMPOSE) run --rm --no-deps \
		-e SAPS_SCRIPT=/workspace/scripts/run_autonomous_sweep.py \
		-e SAPS_RUNTIME_ARGS="--condition-ids $(CONDITION) --num-trials 1 --initial-state-index $(INITIAL_STATE) --policy-base-seed $(POLICY_BASE_SEED) --output-dir $(AUTONOMOUS_OUTPUT)" \
		runtime

.PHONY: autonomous-sweep
autonomous-sweep:
	$(COMPOSE) run --rm --no-deps \
		-e SAPS_SCRIPT=/workspace/scripts/run_autonomous_sweep.py \
		-e SAPS_RUNTIME_ARGS="$(CONDITION_IDS_ARG) --num-trials $(NUM_TRIALS) --initial-state-index $(INITIAL_STATE) --policy-base-seed $(POLICY_BASE_SEED) --max-steps $(AUTONOMOUS_MAX_STEPS) --output-dir $(AUTONOMOUS_OUTPUT)" \
		runtime

.PHONY: seeded-probe
seeded-probe:
	$(COMPOSE) run --rm --no-deps \
		-e SAPS_SCRIPT=/workspace/tools/diagnostics/probe_seeded_policy.py \
		-e SAPS_RUNTIME_ARGS="--condition-id $(CONDITION) --trial-index $(TRIAL) --initial-state-index $(INITIAL_STATE) --base-seed $(POLICY_BASE_SEED) --environment-seed $(ENVIRONMENT_SEED) --output-dir $(PROBE_OUTPUT)" \
		runtime

.PHONY: scene-inspect
scene-inspect:
	$(COMPOSE) run --rm --no-deps \
		-e SAPS_SCRIPT=/workspace/tools/diagnostics/inspect_libero_scene.py \
		-e SAPS_RUNTIME_ARGS="--initial-state-index $(INITIAL_STATE) --seed $(ENVIRONMENT_SEED) --output-dir $(SCENE_OUTPUT)" \
		runtime

.PHONY: perturbation-preview
perturbation-preview:
	$(COMPOSE) run --rm --no-deps \
		-e SAPS_SCRIPT=/workspace/tools/diagnostics/preview_libero_perturbation.py \
		-e SAPS_RUNTIME_ARGS="--initial-state-index $(INITIAL_STATE) --seed $(ENVIRONMENT_SEED) --delta-x $(DX) --delta-y $(DY) --label $(LABEL) --output-root $(PREVIEW_OUTPUT)" \
		runtime

.PHONY: analyze
analyze:
	$(RUNTIME) /bin/bash -lc \
		'source /.venv/bin/activate && python /workspace/tools/analysis/analyze_autonomous_results.py --output-dir "$(ANALYSIS_OUTPUT)" "$(SUMMARY)"'

.PHONY: analyze-comparison
analyze-comparison:
	$(COMPOSE) run --rm --no-deps runtime /bin/bash -lc \
		'source /.venv/bin/activate && python /workspace/tools/analysis/analyze_operator_comparison.py --autonomous-root $(AUTONOMOUS_RESULTS) --teleoperation-root $(TELEOP_RESULTS) --shared-autonomy-root $(SHARED_RESULTS) --output-dir $(COMPARISON_OUTPUT)'

.PHONY: unit-test
unit-test:
	$(RUNTIME) /bin/bash -lc \
		'source /.venv/bin/activate && python -m unittest discover -s /workspace/tests/unit -p "test_*.py" -v'

.PHONY: compile
compile:
	$(RUNTIME) /bin/bash -lc \
		'source /.venv/bin/activate && python -m compileall -q /workspace/src /workspace/scripts /workspace/tests /workspace/tools && echo "Compilation passed"'


.PHONY: check
check: unit-test compile

.PHONY: help-teleop
help-teleop:
	$(RUNTIME) /bin/bash -lc \
		'source /.venv/bin/activate && python /workspace/scripts/run_teleoperation_episode.py --help'

.PHONY: help-autonomous
help-autonomous:
	$(RUNTIME) /bin/bash -lc \
		'source /.venv/bin/activate && python /workspace/scripts/run_autonomous_sweep.py --help'

.PHONY: help-operator
help-operator:
	$(RUNTIME) /bin/bash -lc \
		'source /.venv/bin/activate && python /workspace/tests/manual/keyboard_operator_smoke.py --help'

.PHONY: help-probe
help-probe:
	$(RUNTIME) /bin/bash -lc \
		'source /.venv/bin/activate && python /workspace/tools/diagnostics/probe_seeded_policy.py --help'

.PHONY: clean-python
clean-python:
	find . \
		-path ./third_party -prune -o \
		-type d -name __pycache__ -exec rm -rf {} +
	find . \
		-path ./third_party -prune -o \
		-type f \( -name "*.pyc" -o -name "*.pyo" \) -delete

# Shared-autonomy live runner
SHARED_MAX_STEPS ?= 1200
SHARED_OUTPUT ?= outputs/shared_autonomy_smoke
FIXED_AUTONOMY_WEIGHT ?= 0.5
COSINE_GAIN ?= 6.0

define RUN_SHARED_AUTONOMY
	$(COMPOSE) run --rm --no-deps \
		-e SAPS_SCRIPT=/workspace/scripts/run_shared_autonomy_episode.py \
		-e SAPS_RUNTIME_ARGS="--arbitration-mode $(1) --fixed-autonomy-weight $(FIXED_AUTONOMY_WEIGHT) --cosine-gain $(COSINE_GAIN) --condition-id $(CONDITION) --trial-index $(TRIAL) --initial-state-index $(INITIAL_STATE) --environment-seed $(ENVIRONMENT_SEED) --policy-base-seed $(POLICY_BASE_SEED) --max-steps $(SHARED_MAX_STEPS) --default-speed-mode $(SPEED_MODE) $(HUMAN_INPUT_ARGS) --output-dir $(SHARED_OUTPUT)" \
		runtime

endef

.PHONY: takeover
takeover:
	$(call RUN_SHARED_AUTONOMY,takeover)

.PHONY: fixed-blend
fixed-blend:
	$(call RUN_SHARED_AUTONOMY,fixed_blend)

.PHONY: cosine-blend
cosine-blend:
	$(call RUN_SHARED_AUTONOMY,cosine_blend)

.PHONY: help-shared-autonomy
help-shared-autonomy:
	$(COMPOSE) run --rm --no-deps runtime /bin/bash -lc \
		'source /.venv/bin/activate && python /workspace/scripts/run_shared_autonomy_episode.py --help'
