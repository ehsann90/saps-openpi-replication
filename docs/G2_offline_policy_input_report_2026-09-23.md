# G2 offline policy-input diagnostics

**Date:** 2026-09-23  
**Status:** Offline input-isolation gates completed; runtime-wait changes deferred.  
**Scope:** Archived observations from `outputs/physical_c1c2_gripper/g2_red_t_gate3_chunks25_20260922`. These tests queried the policy server and did **not** issue robot commands.

## Provenance and controls

- Repository source snapshot: `92103ec14b6ae608c1517874539f84d48a6690d1`; pinned OpenPI: `15a9616a00943ada6c20a0f158e3adb39df2ccac`.
- Policy identity checked at the server: `pi05_droid`, checkpoint `gs://openpi-assets/checkpoints/pi05_droid`, native action shape `[15, 8]`, eight actions used per runtime chunk.
- Each intervention held one archived observation, episode seed `20260827`, replan index, and seeded-noise SHA-256 constant. Unmodified canonical input fields were checked byte for byte. The two requests have **different** replan indices and noise; compare interventions against their own request baseline.
- `request_0001`: replan `1`, originally open hand (`0.0`), noise `c5f8861ed84243f99488645a7e91a1d0dff81910f1ab64a3ae46b52289f05806`, archived action SHA-256 `c4cb1a1f7d94dd354593911841312151210f75d570907db4de3fd323c4d73eb1`.
- `request_0006`: replan `6`, originally open hand (`0.00002796`), noise `ea88250571d63c5c688a98f335c0f94924f8382aec0134dbb3fe42f41bedfc6a`, archived action SHA-256 `c1e8d409d0cd546f325a339d8c75e604af45873b0a728ee83768b14100967d40`.
- The command stages for `request_0001` repeated the prior replay and sweep before prompt and modality interventions. All reported stages passed. The raw `[15, 8]` float64 arrays, including all 15 gripper values for each variant, remain in their respective `actions.npz` files.

## Results

### 1. Exact replay

Three repeated baseline inferences for each tested request reproduced its archived action array exactly, including dtype and bytes. The reported noise SHA-256 matched its archive. This rules out observable replay variation **in these requests under this server setup**; it is not a claim of universal determinism.

### 2. Canonical gripper-state sweep

Only the input closure was set to `0, 0.25, 0.5, 0.75, 1.0`; both images, seven joints, and prompt remained fixed. The table gives the largest absolute first-eight arm-action change relative to the request's archived baseline, and the **ideal**, fixed-start-joints TCP endpoint shift. `CLOSE` means a raw gripper action strictly greater than `0.5`.

| Request | Input closure | CLOSE among executed first eight | Max first-eight arm-action change | Ideal TCP endpoint shift |
|:--|--:|--:|--:|--:|
| `0001` | 0.00 | 8/8 | 0.000 | 0 mm |
| `0001` | 0.25 | 8/8 | 0.182 | 63 mm |
| `0001` | 0.50 | 8/8 | 0.314 | 132 mm |
| `0001` | 0.75 | 8/8 | 0.199 | 67 mm |
| `0001` | 1.00 | 8/8 | 0.142 | 49 mm |
| `0006` | 0.00 | 8/8 | 0.000 | 0 mm |
| `0006` | 0.25 | 8/8 | 0.045 | 22 mm |
| `0006` | 0.50 | 8/8 | 0.050 | 23 mm |
| `0006` | 0.75 | 8/8 | 0.059 | 23 mm |
| `0006` | 1.00 | 8/8 | 0.055 | 14 mm |

**Finding:** Input gripper state affects continuous arm and gripper outputs. None of the five closure values changed the eight binary CLOSE decisions for either tested request. In particular, simply correcting the measured closure cannot explain or remedy CLOSE on the tested early request.

### 3. Isolated prompt comparison

The sole intervention was changing `Pick up the red object` to `Pick up the red T-shaped object`. Each alternate prompt was inferred three times, producing identical arrays within its request.

| Request | First-eight binary gripper decisions | Max first-eight arm-action change | Ideal TCP endpoint shift | Cosine between ideal cumulative translations |
|:--|:--|--:|--:|--:|
| `0001` | 8/8 CLOSE with either prompt | 0.497 | 210 mm | −0.581 |
| `0006` | 8/8 CLOSE with either prompt | 0.043 | 11.5 mm | 0.999 |

**Finding:** Wording changes the continuous policy output, markedly so for the early `request_0001` arm action, but leaves the first eight binary gripper decisions unchanged in both requests. On `request_0001`, the gripper decision differs at indices 9–14, which are outside the eight executed actions. These comparisons do not isolate the cause of the different *physical runs*, whose observations also changed.

### 4. Images versus proprioception, `request_0001`

The baseline remains the exact archived `request_0001` observation, prompt, seed, replan, and noise. With images fixed, joints and/or closure were substituted from adjacent requests. With proprioception fixed, the exterior and/or wrist images were substituted. The table reports changed binary decisions in the first eight actions relative to the baseline's 8/8 CLOSE.

| Adjacent source; fields substituted | CLOSE among first eight | Changed indices | Max first-eight arm-action change |
|:--|--:|:--|--:|
| `0000`; joints | 2/8 | 0, 1, 2, 3, 6, 7 | 0.487 |
| `0000`; exterior image | 8/8 | none | 0.132 |
| `0000`; wrist image | 6/8 | 0, 1 | 0.161 |
| `0000`; both images | 4/8 | 0, 1, 2, 7 | 0.159 |
| `0002`; joints | 8/8 | none | 0.212 |
| `0002`; closure | 8/8 | none | 0.142 |
| `0002`; joints + closure | 8/8 | none | 0.169 |
| `0002`; exterior image | 8/8 | none | 0.075 |
| `0002`; wrist image | 8/8 | none | 0.320 |
| `0002`; both images | 8/8 | none | 0.323 |

The `0000` gripper-only intervention was skipped because its closure was byte-identical to baseline; combined `0000` joints + gripper was skipped because only the joints differed. These are not failed tests.

**Finding:** At this observation, an adjacent earlier joint state *or* earlier wrist imagery can flip some CLOSE decisions. The exterior image substitutions did not flip any first-eight decisions. The direction of the temporal swap matters: the adjacent later substitutions did not flip first-eight decisions. Different perturbation sizes, camera/robot mismatch in synthetic combinations, and only one baseline observation prevent a general attribution of grasp timing as “primarily visual” or “primarily state-driven.”

## Interpretation and limits

1. The early CLOSE at `request_0001` persists for every tested closure and for both tested prompts; it is **not** invariant to all proprioception or imagery. The policy's early grasp decision is sensitive to the earlier joint vector and earlier wrist view.
2. Arm-action changes and ideal TCP differences describe counterfactual *policy predictions* and a perfect-target forward-kinematics calculation. No counterfactual trajectory was executed on the FR3. They do not establish physical reach direction, collision safety, object proximity, grasp success, or the actual cause of later behavior under replanning.
3. These are targeted diagnostics on two requests from one physical episode, not a statistical evaluation. The user's description of the object being far from the gripper was not converted into a calibrated distance measurement in these diagnostics.

## Source artifacts

The following attached files are the report's source evidence; paths below correspond to the user's original `outputs/` directories. The sha256 values identify the **uploaded files**, separate from the action-array hashes above.

| Gate / original output | `run.json` uploaded as (SHA-256 prefix) | `actions.npz` uploaded as (SHA-256 prefix) |
|:--|:--|:--|
| `0006` gripper, `outputs/g2_offline_gripper_sweep/request0006_20260923` | `run(20260923-070511).json` (`e83a40568c62`) | `actions(2).npz` (`3dc70055a386`) |
| `0006` prompt, `outputs/g2_offline_prompt_comparison/request0006_20260923` | `run(20260923-072334).json` (`40b037d5b122`) | `actions(3).npz` (`c31a307ad4b9`) |
| `0001` replay + gripper, `outputs/g2_offline_input_isolation/request0001_gripper_20260923` | `run(20260923-075100).json` (`278e20829dab`) | `actions(4).npz` (`226e79d598c2`) |
| `0001` prompt, `outputs/g2_offline_input_isolation/request0001_prompt_20260923` | `run(20260923-075239).json` (`8de314fe2b6f`) | `actions(5).npz` (`a39d0ca777d2`) |
| `0001` modalities, `outputs/g2_offline_input_isolation/request0001_modalities_20260923` | `run(20260923-075524).json` (`97833ab6a116`) | `actions(6).npz` (`beaf0dc5ef23`) |
