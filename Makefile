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
SPACEMOUSE_REFRESH_HZ ?= 20
SPACEMOUSE_PROFILE ?= configs/spacemouse_profile.json
SPACEMOUSE_DEVICE_RUNNER_ARG = $(if $(and $(filter spacemouse,$(strip $(INPUT_SOURCE))),$(strip $(SPACEMOUSE_DEVICE))),--spacemouse-device-path $(SPACEMOUSE_DEVICE),)
SPACEMOUSE_DEVICE_DIAGNOSTIC_ARG = $(if $(strip $(SPACEMOUSE_DEVICE)),--device-path $(SPACEMOUSE_DEVICE),)
SPACEMOUSE_PROFILE_ARG = $(if $(and $(filter spacemouse,$(strip $(INPUT_SOURCE))),$(strip $(SPACEMOUSE_PROFILE))),--spacemouse-profile-path $(SPACEMOUSE_PROFILE),)
SPACEMOUSE_DIAGNOSTIC_PROFILE_ARG = $(if $(strip $(SPACEMOUSE_PROFILE)),--profile-path $(SPACEMOUSE_PROFILE),)

HUMAN_INPUT_ARGS = --input-source $(INPUT_SOURCE) $(SPACEMOUSE_DEVICE_RUNNER_ARG) $(SPACEMOUSE_PROFILE_ARG)
SESSION_INPUT_ARGS = --input-source $(INPUT_SOURCE) $(SPACEMOUSE_DEVICE_RUNNER_ARG) $(SPACEMOUSE_PROFILE_ARG)

ENVIRONMENT_SEED ?= 7
POLICY_BASE_SEED ?= 20260724

TELEOP_OUTPUT ?= outputs/teleoperation_smoke
AUTONOMOUS_OUTPUT ?= outputs/autonomous_sweep
PROBE_OUTPUT ?= outputs/seeded_policy_probe
SCENE_OUTPUT ?= outputs/scene_inspection
PREVIEW_OUTPUT ?= outputs/perturbation_preview
ANALYSIS_OUTPUT ?= results/analysis
COMPARISON_OUTPUT ?= results/saps_libero_current
GATE2_ANALYSIS_OUTPUT ?= results/gate2_shared_autonomy_pilot_v2
GATE2_AUTONOMOUS_RESULTS ?= outputs/gate2_autonomous_pilot_v2
AUTONOMOUS_RESULTS ?= outputs/autonomous_deterministic_n20_state0_v1
TELEOP_RESULTS ?= outputs/saps_libero_teleoperation_v2
SHARED_RESULTS ?= outputs/saps_libero_shared_autonomy_v2
DROID_DATA_DIR ?= data/droid_m1
DROID_SAMPLE_CONFIG ?= configs/droid_m1_sample.json
DROID_OUTPUT ?= outputs/physical_pi05_droid_m1
DROID_RUN_ID ?= m1_$(shell date -u +%Y%m%dT%H%M%SZ)
DROID_NUM_SAMPLES ?= 3
DROID_REPEAT_COUNT ?= 2
DROID_POLICY_SEED ?= 20260827
M2_M1_RUN ?= outputs/physical_pi05_droid_m1/validation_final_20260827T1318Z/run.json
M2_OUTPUT ?= outputs/physical_pi05_droid_m2
M2_RUN_ID ?= m2_$(shell date -u +%Y%m%dT%H%M%SZ)
M3_OUTPUT ?= outputs/physical_pi05_droid_m3
M3_RUN_ID ?= m3_$(shell date -u +%Y%m%dT%H%M%SZ)
M3_RUN_DIR = $(M3_OUTPUT)/$(M3_RUN_ID)
M3_SPACEMOUSE_RUN_ID ?= spnav_$(shell date -u +%Y%m%dT%H%M%SZ)
M3_SPACEMOUSE_RUN_DIR = $(M3_OUTPUT)/$(M3_SPACEMOUSE_RUN_ID)
M3_EXTERIOR_SERIAL ?=
M3_PROMPT ?= pick up the object
M3_OBSERVATIONS ?= 5
M3_CAPTURE_TIMEOUT ?= 30
M3_WRIST_TOPIC ?= /wrist/wrist_camera/color/image_raw
M3_EXTERIOR_TOPIC ?= /exterior/exterior_camera/color/image_raw
M3_JOINT_TOPIC ?= /franka/joint_states
M3_GRIPPER_TOPIC ?= /franka_gripper/joint_states
M3_SPACEMOUSE_DURATION ?= 10
M3_SPACEMOUSE_OUTPUT ?=
M3_PROJECTION ?=
M3_COMPARISON_OUTPUT ?= $(M3_OUTPUT)/comparison_$(shell date -u +%Y%m%dT%H%M%SZ).json
M3_MAPPING_RUN ?= outputs/physical_pi05_droid_m3/m3_live_20260828T1548Z
ALLOW_LEGACY_M2 ?=
FRANKA_ROS2_WS ?= $(HOME)/franka_ros2_ws
FRANKA_DESCRIPTION_DIR ?= $(FRANKA_ROS2_WS)/src/franka_description
IGD_FR3_CONTROL_DIR ?= $(FRANKA_ROS2_WS)/src/igd_fr3_control
FRANKA_ROS2_INSTALL ?= $(FRANKA_ROS2_WS)/install
REDO_EPISODES ?=
REDO_EPISODES_ARG = $(if $(strip $(REDO_EPISODES)),--redo-episode-ids $(REDO_EPISODES),)
MANIFEST ?= configs/operator_shared_autonomy_manifest.json
SESSION_OUTPUT ?= outputs/operator_experiment
REPOSITORY_COMMIT := $(shell git rev-parse HEAD)
OPENPI_COMMIT := $(shell git -C third_party/openpi rev-parse HEAD)
DROID_DIRTY_ARG = $(if $(strip $(shell git status --porcelain)),--repository-dirty,)
override GATE2_MANIFEST := configs/gate2_shared_autonomy_pilot_manifest.json
override GATE2_AUTONOMOUS_PROTOCOL := configs/gate2_autonomous_pilot_protocol.json
override GATE2_PROFILE := configs/spacemouse_profile.json
override GATE2_OUTPUT := outputs/gate2_shared_autonomy_pilot_v2
override GATE2_AUTONOMOUS_OUTPUT := outputs/gate2_autonomous_pilot_v2
override GATE2_EXPERIMENT_ID := saps_libero_gate2_shared_autonomy_pilot_v2
override GATE2_AUTONOMOUS_EXPERIMENT_ID := saps_libero_gate2_autonomous_pilot_v2

DX ?= 0.0
DY ?= 0.0
LABEL ?= preview
SUMMARY ?= outputs/autonomous_sweep/sweep_summary.json

.PHONY: help
help:
	@echo "Main processes:"
	@echo "  make build-images"
	@echo "  make policy-server"
	@echo "  make droid-sample  # 13.9 MB genuine offline subset"
	@echo "  make droid-policy-server"
	@echo "  make droid-inference"
	@echo "  make validate-fr3-kinematics  # hand-derived FK/Jacobian"
	@echo "  make validate-droid-fr3-mapping  # accepted M3 saved run"
	@echo "  make droid-fr3-m2 ALLOW_LEGACY_M2=1  # superseded provenance"
	@echo "  make physical-m3-observation M3_EXTERIOR_SERIAL=<serial>"
	@echo "  make physical-m3-shadow-inference M3_RUN_ID=<captured-run>"
	@echo "  make physical-m3-spacemouse  # subscriber/spnavd log only"
	@echo "  make autonomous-smoke"
	@echo "  make autonomous-sweep NUM_TRIALS=20"
	@echo "  make teleop CONDITION=nominal TRIAL=0"
	@echo "  make teleop INPUT_SOURCE=spacemouse CONDITION=nominal TRIAL=0"
	@echo "  make takeover CONDITION=nominal TRIAL=0"
	@echo "  make fixed-blend FIXED_AUTONOMY_WEIGHT=0.5"
	@echo "  make cosine-blend COSINE_GAIN=6.0"
	@echo "  make operator-session MANIFEST=<manifest.json>"
	@echo "  make teleoperation-session"
	@echo "  make shared-autonomy-session"
	@echo
	@echo "Completed simulation archive (read-only analysis):"
	@echo "  make gate2-analysis"
	@echo "  make gate2-preflight SPACEMOUSE_DEVICE=/dev/input/by-id/..."
	@echo "  make gate2-autonomous-preflight"
	@echo
	@echo "Historical frozen collection targets (do not reuse completed roots):"
	@echo "  make gate2-session SPACEMOUSE_DEVICE=/dev/input/by-id/..."
	@echo "  make gate2-autonomous"
	@echo
	@echo "Tests:"
	@echo "  make check"
	@echo "  make unit-test"
	@echo "  make operator-smoke  # keyboard-only browser input"
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
	@echo "  GATE2_ANALYSIS_OUTPUT, GATE2_AUTONOMOUS_RESULTS"
	@echo "  FIXED_AUTONOMY_WEIGHT, COSINE_GAIN"
	@echo "  DROID_NUM_SAMPLES, DROID_REPEAT_COUNT, DROID_POLICY_SEED, DROID_RUN_ID"
	@echo "  M2_M1_RUN, M2_OUTPUT, M2_RUN_ID, FRANKA_ROS2_WS"
	@echo "  FRANKA_DESCRIPTION_DIR, IGD_FR3_CONTROL_DIR"
	@echo "  M3_RUN_ID, M3_MAPPING_RUN, M3_EXTERIOR_SERIAL, M3_PROMPT"
	@echo "  M3_OBSERVATIONS, FRANKA_ROS2_INSTALL, ALLOW_LEGACY_M2"

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

.PHONY: droid-sample
droid-sample:
	docker run --rm \
		-v $(CURDIR):/workspace \
		-w /workspace \
		--entrypoint /bin/bash \
		openpi_server:latest -lc \
		'source /.venv/bin/activate && python tools/datasets/prepare_droid_m1_sample.py --config-path $(DROID_SAMPLE_CONFIG) --output-dir $(DROID_DATA_DIR) && chown -R $(LOCAL_UID):$(LOCAL_GID) $(DROID_DATA_DIR)'

.PHONY: droid-policy-server
droid-policy-server:
	SAPS_SERVER_ARGS="--config-name pi05_droid --checkpoint-dir gs://openpi-assets/checkpoints/pi05_droid" \
		$(COMPOSE) up openpi_server

.PHONY: droid-inference
droid-inference:
	$(COMPOSE) run --rm --no-deps \
		-e SAPS_SCRIPT=/workspace/scripts/droid_sample_inference.py \
		-e SAPS_RUNTIME_ARGS="--sample-bundle-path $(DROID_DATA_DIR)/droid_m1_samples.npz --sample-metadata-path $(DROID_DATA_DIR)/droid_m1_samples.json --num-samples $(DROID_NUM_SAMPLES) --repeat-count $(DROID_REPEAT_COUNT) --policy-episode-seed $(DROID_POLICY_SEED) --repository-commit $(REPOSITORY_COMMIT) $(DROID_DIRTY_ARG) --openpi-commit $(OPENPI_COMMIT) --output-dir $(DROID_OUTPUT)/$(DROID_RUN_ID)" \
		runtime

.PHONY: droid-fr3-m2
droid-fr3-m2:
	@test "$(ALLOW_LEGACY_M2)" = "1" || \
		{ echo "Superseded M2 projection requires ALLOW_LEGACY_M2=1."; \
		  exit 2; }
	bash -lc 'source /opt/ros/jazzy/setup.bash && \
		source "$(FRANKA_ROS2_INSTALL)/setup.bash" && \
		cd $(CURDIR) && \
		export PYTHONPATH="$(CURDIR)/src:$${PYTHONPATH}" && \
		/usr/bin/python3 \
		tools/diagnostics/project_droid_m1_to_fr3.py \
		--m1-run-path $(M2_M1_RUN) \
		--franka-description-dir "$(FRANKA_DESCRIPTION_DIR)" \
		--igd-control-dir "$(IGD_FR3_CONTROL_DIR)" \
		--output-dir $(M2_OUTPUT)/$(M2_RUN_ID)'

.PHONY: validate-fr3-kinematics
validate-fr3-kinematics:
	bash -lc 'source /opt/ros/jazzy/setup.bash && \
		source "$(FRANKA_ROS2_INSTALL)/setup.bash" && \
		cd $(CURDIR) && \
		export PYTHONPATH="$(CURDIR)/src:$${PYTHONPATH}" && \
		/usr/bin/python3 \
		tools/diagnostics/validate_fr3_forward_kinematics.py \
		--xacro-path "$(FRANKA_DESCRIPTION_DIR)/robots/fr3/fr3.urdf.xacro"'

.PHONY: validate-droid-fr3-mapping
validate-droid-fr3-mapping:
	bash -lc 'source /opt/ros/jazzy/setup.bash && \
		source "$(FRANKA_ROS2_INSTALL)/setup.bash" && \
		cd $(CURDIR) && \
		export PYTHONPATH="$(CURDIR)/src:$${PYTHONPATH}" && \
		/usr/bin/python3 \
		tools/diagnostics/validate_droid_fr3_action_mapping.py \
		--run-dir $(M3_MAPPING_RUN) \
		--franka-description-dir "$(FRANKA_DESCRIPTION_DIR)"'

.PHONY: physical-m3-observation
physical-m3-observation:
	@test -n "$(strip $(M3_EXTERIOR_SERIAL))" || \
		{ echo "M3 requires an explicit M3_EXTERIOR_SERIAL."; \
		  echo "Temporary serial 244222076317 is not selected implicitly."; \
		  exit 2; }
	bash -lc 'source /opt/ros/jazzy/setup.bash && \
		source "$(FRANKA_ROS2_INSTALL)/setup.bash" && \
		cd $(CURDIR) && \
		export PYTHONPATH="$(CURDIR)/src:$${PYTHONPATH}" && \
		/usr/bin/python3 \
		tools/diagnostics/capture_physical_m3_observations.py \
		--output-dir $(M3_RUN_DIR) \
		--exterior-camera-serial $(M3_EXTERIOR_SERIAL) \
		--prompt "$(M3_PROMPT)" \
		--wrist-image-topic $(M3_WRIST_TOPIC) \
		--exterior-image-topic $(M3_EXTERIOR_TOPIC) \
		--joint-state-topic $(M3_JOINT_TOPIC) \
		--gripper-state-topic $(M3_GRIPPER_TOPIC) \
		--observation-count $(M3_OBSERVATIONS) \
		--timeout-seconds $(M3_CAPTURE_TIMEOUT) \
		--franka-description-dir "$(FRANKA_DESCRIPTION_DIR)" \
		--igd-control-dir "$(IGD_FR3_CONTROL_DIR)"'

.PHONY: physical-m3-shadow-inference
physical-m3-shadow-inference:
	$(COMPOSE) run --rm --no-deps \
		-e SAPS_SCRIPT=/workspace/scripts/physical_shadow_inference.py \
		-e SAPS_RUNTIME_ARGS="--run-dir $(M3_RUN_DIR) --policy-episode-seed $(DROID_POLICY_SEED)" \
		runtime

.PHONY: physical-m3-shadow-project
physical-m3-shadow-project:
	@test "$(ALLOW_LEGACY_M2)" = "1" || \
		{ echo "Superseded M2 projection requires ALLOW_LEGACY_M2=1."; \
		  exit 2; }
	bash -lc 'source /opt/ros/jazzy/setup.bash && \
		source "$(FRANKA_ROS2_INSTALL)/setup.bash" && \
		cd $(CURDIR) && \
		export PYTHONPATH="$(CURDIR)/src:$${PYTHONPATH}" && \
		/usr/bin/python3 \
		tools/diagnostics/project_physical_m3_shadow.py \
		--run-dir $(M3_RUN_DIR) \
		--franka-description-dir "$(FRANKA_DESCRIPTION_DIR)"'

.PHONY: physical-m3-shadow
physical-m3-shadow:
	@test "$(ALLOW_LEGACY_M2)" = "1" || \
		{ echo "Superseded M2 projection requires ALLOW_LEGACY_M2=1."; \
		  exit 2; }
	$(MAKE) physical-m3-observation M3_RUN_ID=$(M3_RUN_ID)
	$(MAKE) physical-m3-shadow-inference M3_RUN_ID=$(M3_RUN_ID)
	$(MAKE) physical-m3-shadow-project M3_RUN_ID=$(M3_RUN_ID)

.PHONY: physical-m3-spacemouse
physical-m3-spacemouse:
	bash -lc 'source /opt/ros/jazzy/setup.bash && \
		source "$(FRANKA_ROS2_INSTALL)/setup.bash" && \
		cd $(CURDIR) && \
		export PYTHONPATH="$(CURDIR)/src:$${PYTHONPATH}" && \
		/usr/bin/python3 \
		tools/diagnostics/inspect_physical_spnav.py \
		--output-dir $(M3_SPACEMOUSE_RUN_DIR) \
		--duration-seconds $(M3_SPACEMOUSE_DURATION) \
		--joint-state-topic $(M3_JOINT_TOPIC) \
		--franka-description-dir "$(FRANKA_DESCRIPTION_DIR)" \
		--igd-control-dir "$(IGD_FR3_CONTROL_DIR)"'

.PHONY: physical-m3-compare
physical-m3-compare:
	@test "$(ALLOW_LEGACY_M2)" = "1" || \
		{ echo "Superseded normalized comparison requires ALLOW_LEGACY_M2=1."; \
		  exit 2; }
	@test -n "$(strip $(M3_PROJECTION))" || \
		{ echo "Set M3_PROJECTION=<shadow_projection.json>."; exit 2; }
	@test -n "$(strip $(M3_SPACEMOUSE_OUTPUT))" || \
		{ echo "Set M3_SPACEMOUSE_OUTPUT=<spnav.json>."; exit 2; }
	$(RUNTIME) /bin/bash -lc \
		'source /.venv/bin/activate && python /workspace/tools/analysis/compare_physical_m3_actions.py --projection-path $(M3_PROJECTION) --spnav-path $(M3_SPACEMOUSE_OUTPUT) --output-path $(M3_COMPARISON_OUTPUT)'

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

.PHONY: gate2-preflight
gate2-preflight:
	@test -n "$(strip $(SPACEMOUSE_DEVICE))" || \
		{ echo "Gate-2 requires SPACEMOUSE_DEVICE=/dev/input/by-id/..."; exit 2; }
	$(COMPOSE) run --rm --no-deps \
		-e SAPS_SCRIPT=/workspace/scripts/preflight_gate2_operator_pilot.py \
		-e SAPS_RUNTIME_ARGS="--manifest-path $(GATE2_MANIFEST) --spacemouse-profile-path $(GATE2_PROFILE) --spacemouse-device-path $(SPACEMOUSE_DEVICE) --autonomous-protocol-path $(GATE2_AUTONOMOUS_PROTOCOL) --output-dir $(GATE2_OUTPUT)" \
		runtime

.PHONY: gate2-autonomous-preflight
gate2-autonomous-preflight:
	$(COMPOSE) run --rm --no-deps \
		-e SAPS_SCRIPT=/workspace/scripts/preflight_gate2_autonomous_pilot.py \
		-e SAPS_RUNTIME_ARGS="--protocol-path $(GATE2_AUTONOMOUS_PROTOCOL)" \
		runtime

.PHONY: gate2-autonomous
gate2-autonomous:
	@test -z "$$(git status --porcelain)" || \
		{ echo "Gate-2 collection requires a clean repository."; exit 1; }
	$(COMPOSE) run --rm --no-deps \
		-e SAPS_SCRIPT=/workspace/scripts/run_autonomous_sweep.py \
		-e SAPS_RUNTIME_ARGS="--config-path configs/libero_cream_cheese_offsets.json --condition-ids nominal,p02,p06,p09 --num-trials 5 --initial-state-index 0 --resume --deterministic-policy --policy-base-seed 20260724 --seed 7 --resolution 256 --resize-size 224 --replan-steps 5 --num-steps-wait 10 --max-steps 280 --control-frequency-hz 20.0 --video-fps 10 --output-dir $(GATE2_AUTONOMOUS_OUTPUT) --required-protocol-id $(GATE2_AUTONOMOUS_EXPERIMENT_ID) --protocol-path $(GATE2_AUTONOMOUS_PROTOCOL) --repository-commit $(REPOSITORY_COMMIT)" \
		runtime

.PHONY: gate2-session
gate2-session:
	@test -n "$(strip $(SPACEMOUSE_DEVICE))" || \
		{ echo "Gate-2 requires SPACEMOUSE_DEVICE=/dev/input/by-id/..."; exit 2; }
	@test -z "$$(git status --porcelain)" || \
		{ echo "Gate-2 collection requires a clean repository."; exit 1; }
	$(COMPOSE) run --rm --no-deps \
		-e SAPS_SCRIPT=/workspace/scripts/run_operator_experiment.py \
		-e SAPS_RUNTIME_ARGS="--manifest-path $(GATE2_MANIFEST) --repository-commit $(REPOSITORY_COMMIT) --output-dir $(GATE2_OUTPUT) --required-protocol-id $(GATE2_EXPERIMENT_ID) --input-source spacemouse --spacemouse-device-path $(SPACEMOUSE_DEVICE) --spacemouse-profile-path $(GATE2_PROFILE) $(REDO_EPISODES_ARG)" \
		runtime

.PHONY: gate2-analysis
gate2-analysis:
	$(COMPOSE) run --rm --no-deps \
		-e SAPS_SCRIPT=/workspace/scripts/analyze_gate2_operator_pilot.py \
		-e SAPS_RUNTIME_ARGS="--session-root $(GATE2_OUTPUT) --autonomous-root $(GATE2_AUTONOMOUS_RESULTS) --output-dir $(GATE2_ANALYSIS_OUTPUT)" \
		runtime

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
		-e SAPS_RUNTIME_ARGS="$(SPACEMOUSE_DEVICE_DIAGNOSTIC_ARG) $(SPACEMOUSE_DIAGNOSTIC_PROFILE_ARG) --refresh-frequency-hz $(SPACEMOUSE_REFRESH_HZ)" \
		runtime

.PHONY: spacemouse-calibrate
spacemouse-calibrate:
	$(COMPOSE) run --rm --no-deps \
		-e SAPS_SCRIPT=/workspace/scripts/run_spacemouse_calibration.py \
		-e SAPS_RUNTIME_ARGS="$(SPACEMOUSE_DEVICE_DIAGNOSTIC_ARG) $(SPACEMOUSE_DIAGNOSTIC_PROFILE_ARG)" \
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
