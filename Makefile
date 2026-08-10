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
OPERATOR_SMOKE_OUTPUT ?= outputs/operator_smoke/input_events.jsonl
SPEED_MODE ?= fine

ENVIRONMENT_SEED ?= 7
POLICY_BASE_SEED ?= 20260724

TELEOP_OUTPUT ?= outputs/teleoperation_smoke
AUTONOMOUS_OUTPUT ?= outputs/autonomous_sweep
PROBE_OUTPUT ?= outputs/seeded_policy_probe
SCENE_OUTPUT ?= outputs/scene_inspection
PREVIEW_OUTPUT ?= outputs/perturbation_preview
ANALYSIS_OUTPUT ?= results/analysis

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
	@echo "  make analyze SUMMARY=<sweep_summary.json>"
	@echo
	@echo "Common overrides:"
	@echo "  CONDITION, CONDITION_IDS, TRIAL, INITIAL_STATE"
	@echo "  NUM_TRIALS, TELEOP_MAX_STEPS, AUTONOMOUS_MAX_STEPS, SPEED_MODE"
	@echo "  TELEOP_OUTPUT, AUTONOMOUS_OUTPUT, SHARED_OUTPUT"
	@echo "  FIXED_AUTONOMY_WEIGHT, COSINE_GAIN"
	@echo "  CONTROL_FREQUENCY_HZ, SCHEDULER_MODE, REPLAN_STEPS"
	@echo "  PREFETCH_REMAINING_ACTIONS"

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
		-e SAPS_RUNTIME_ARGS="--duration-seconds $(DURATION) --output-path $(OPERATOR_SMOKE_OUTPUT)" \
		runtime

.PHONY: teleop
teleop:
	$(COMPOSE) run --rm --no-deps \
		-e SAPS_SCRIPT=/workspace/scripts/run_teleoperation_episode.py \
		-e SAPS_RUNTIME_ARGS="--condition-id $(CONDITION) --trial-index $(TRIAL) --initial-state-index $(INITIAL_STATE) --environment-seed $(ENVIRONMENT_SEED) --policy-base-seed $(POLICY_BASE_SEED) --max-steps $(TELEOP_MAX_STEPS) --default-speed-mode $(SPEED_MODE) --output-dir $(TELEOP_OUTPUT)" \
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
SCHEDULER_MODE ?= strict_pause
REPLAN_STEPS ?= 5
PREFETCH_REMAINING_ACTIONS ?= 12
MOCK_OPERATOR_TRACE ?=
MOCK_OPERATOR_TRACE_ARG = $(if $(strip $(MOCK_OPERATOR_TRACE)),--mock-operator-trace $(MOCK_OPERATOR_TRACE),)
MAX_PLAN_AGE_SECONDS ?= 1.5
MAX_PLAN_TRANSLATION_M ?= 0.15
MAX_PLAN_ROTATION_RADIANS ?= 0.75
MAX_PLAN_GRIPPER_DELTA ?= 0.5
HANDOFF_STEPS ?= 3
EXHAUSTION_FALLBACK ?= pause
CONTROL_FREQUENCY_HZ ?= 20.0

define RUN_SHARED_AUTONOMY
	$(COMPOSE) run --rm --no-deps \
		-e SAPS_SCRIPT=/workspace/scripts/run_shared_autonomy_episode.py \
		-e SAPS_RUNTIME_ARGS="--arbitration-mode $(1) --fixed-autonomy-weight $(FIXED_AUTONOMY_WEIGHT) --cosine-gain $(COSINE_GAIN) --scheduler-mode $(SCHEDULER_MODE) --replan-steps $(REPLAN_STEPS) --prefetch-remaining-actions $(PREFETCH_REMAINING_ACTIONS) --max-plan-age-seconds $(MAX_PLAN_AGE_SECONDS) --max-plan-translation-m $(MAX_PLAN_TRANSLATION_M) --max-plan-rotation-radians $(MAX_PLAN_ROTATION_RADIANS) --max-plan-gripper-delta $(MAX_PLAN_GRIPPER_DELTA) --handoff-steps $(HANDOFF_STEPS) --exhaustion-fallback $(EXHAUSTION_FALLBACK) --control-frequency-hz $(CONTROL_FREQUENCY_HZ) --condition-id $(CONDITION) --trial-index $(TRIAL) --initial-state-index $(INITIAL_STATE) --environment-seed $(ENVIRONMENT_SEED) --policy-base-seed $(POLICY_BASE_SEED) --max-steps $(SHARED_MAX_STEPS) --default-speed-mode $(SPEED_MODE) $(MOCK_OPERATOR_TRACE_ARG) --output-dir $(SHARED_OUTPUT)" \
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
