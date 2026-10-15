# Isaac policy video and archived gripper review

Source: `outputs/isaac_droid_robotiq_rollout/20261015T105024Z_78d660d1/`
(26 complete replans, 208 actions, 13.866667 s simulated policy execution).
This review uses the existing archive only. No policy or physics experiment was
run. Original archived observations and execution records remain unchanged.

## Video artifacts and reproduction

[Policy-observation MP4](../../outputs/isaac_droid_robotiq_rollout/20261015T105024Z_78d660d1/videos/policy_observations.mp4)
and [metadata](../../outputs/isaac_droid_robotiq_rollout/20261015T105024Z_78d660d1/videos/policy_observations.json)
form a **replan-sampled reconstruction**. It contains exactly the 26 complete
replans' archived observation image pairs in numeric order. The exporter
checks both decoded PNG arrays against the SHA-256 evidence saved for the
actual OpenPI request. Replan 26 has a saved policy request and response at
the final simulated state but no executed action. It is flagged in metadata
and excluded because it has zero simulated presentation interval. The video
metadata is labelled `partial` to make that omission explicit. The video
is 15/8 FPS: one frame per 8/15 s chunk. `ffprobe` found 26 frames and
13.866667 s duration. The last sample is displayed for one final 8/15 s
interval; the first-to-last sample timestamp span is 13.333334 s. No
intermediate visual frames were synthesized. Inference wall time is excluded.
There is no historical `simulation_motion.mp4`: action-boundary images were
not archived, so genuine 15 FPS motion cannot be reconstructed.

Export from an archived run that does not yet have this video:

```bash
python3 tools/analysis/export_isaac_policy_video.py \
  outputs/isaac_droid_robotiq_rollout/<run-id>
```

For a future manual rollout, add `--record-policy-video` to the existing
`sim/scripts/policy_rollout.py` command. It writes
`<run>/videos/policy_observations.mp4` (the exact request image pairs at
15/8 FPS) and `<run>/videos/simulation_motion.mp4` (initial state plus each
completed action at 15 FPS). Each is a synchronized side-by-side view. Raw
motion camera pairs remain under `<run>/videos/raw/`; request images remain
under `<run>/replans/`. JSON files beside each MP4 record simulation times,
frame mapping, resolution, frame loss, encoder and wall overhead. The 15 FPS
video's final image occupies one extra display interval; its simulated span
is always the final timestamp minus the initial timestamp. The GUI path
reuses the last physics-step render; headless mode renders once without a
physics step. A manual interrupt marks the motion recording partial and
preserves raw frames. Encoding occurs after action execution, so its wall
cost cannot change simulated action duration. The actual rollout below
exercised this path; the bounded smoke script remains available.

## Recorded position-reference validation

The manually run rollout at
`outputs/isaac_droid_robotiq_rollout/20261015T131423Z_f1f5e228/` has
`record_policy_video=true`, 20 complete replans, 160 actions, and
10.666667 s simulated control time. The user reports that both MP4s play.
Independent file inspection found H.264 640×180 streams: 20 observation
frames at 15/8 FPS (10.666667 s presentation) and 161 motion frames at
15 FPS (10.733333 s presentation, including one final display interval).
Motion metadata records a 10.666667 s initial-to-final simulated span,
four physics steps between adjacent action frames, 161 raw pairs, zero
dropped frames, and 3.903 s capture plus 3.562 s encoding wall time. The
initial motion pair matches the first saved policy observation. All 20
saved request image pairs match their recorded SHA-256 values. Replan 20
has an observation but no policy response or completed action; it is listed
as excluded from the partial observation video. The rollout ended by manual
interrupt, and both video metadata files are marked partial. These checks
inspect saved artifacts; they do not establish camera calibration or contact
fidelity.

## Gripper evidence

The action-level source is each replan's `execution.json`; observation poses
and images are in the corresponding `observation.json` and `*_raw.png`.
`contact_summary_after` is **cumulative**, so unchanged counts during a
replan mean no new reported contact. The monitor records only loaded pad/cube
contacts above 1e-6 N s, not table or internal mechanism contacts.

| Question | First sequence, replans 11–15 | Second sequence, replans 18–21 |
| --- | --- | --- |
| Closure | Replan 12: driver 0 → 0.8203 rad; measured opening 87.1 → 0 mm by action 4. Replan 13 remains closed. | Replan 19: all eight actions request CLOSED and target 0.8203 rad, but driver −0.00056 → −0.00017 rad and opening stays 87.08 mm. |
| Object between pads | Not established by saved fingertip poses. Full closure and no new loaded contact during replan 12 argue against enclosure. | Wrist image at replan 19 shows the cube between finger silhouettes, but projection alone does not establish 3D enclosure; opening never decreases. |
| Bilateral contact | None. Left-pad/cube counter reached 13 before replan 12; it did not increase during closure. Right remained zero. | None. Left counter remained 13 through replan 19; it rose to 20 during replan 20 OPEN and 28 during replan 21. Right remained zero. |
| Retained object | No. Cube stayed at z ≈ 0.420 m while the closed TCP moved away. | No. The gripper stayed open and cube z stayed ≈ 0.420 m. |
| Upward arm motion while closed | Replan 13 TCP z increased only 0.0095 m (0.4637 → 0.4732 m); replan 14 immediately commanded OPEN. The larger rise in replan 15 occurred open. | Replan 19 TCP z increased 0.0036 m while CLOSE was requested but opening stayed 87 mm. Replan 20 commanded OPEN; replan 21 TCP rose 0.225 m while open. |
| Sustained object lift | None in recorded object poses. | None in recorded object poses; later motion was lateral. |

At the start of replan 12, the grip-frame/TCP origin was approximately
`(0.424, 0.042, 0.477)` m and the cube center `(0.503, -0.001, 0.420)` m,
about 0.106 m apart. The closest action-end grip-frame/cube-center distance
during replan 12 was about 0.065 m. These are **grip-frame** distances, not
finger-pad clearances. The archive has no validated per-action fingertip world
poses, so exact enclosure cannot be reconstructed. The first close took place
while the object remained stationary and without new bilateral loaded contact.

At the start of replan 19, the grip-frame origin was approximately
`(0.515, 0.018, 0.415)` m, 0.023 m from the cube center. Its
[eight action rows](../../outputs/isaac_droid_robotiq_rollout/20261015T105024Z_78d660d1/replans/0019/execution.json)
show `CLOSED`, the 0.8203 rad target, `transition_active=true`, 32 transition
steps and no controller fault. The [gripper controller](../src/isaac_fr3/robotiq_gripper.py)
checks target readback on every physics step; a lost target would have aborted
the chunk. The authored [drive](../assets/fr3_robotiq_droid_historical_hg1.usda)
inherits stiffness 3, damping 0.0002 and maximum speed 146.46 deg/s, with a
10 N m force override. The joint is authored for 0–47 degrees. The first
closure's approximately 0.17 rad per action is consistent with that speed
limit. During replan 20's OPEN request, measured driver position later moved
*negative*, reaching −0.387 rad; it reached −0.686 rad early in replan 21
before returning toward zero. This abnormal opposite-direction excursion is
evidence of a disturbed articulation/constraint response. The archive lacks
driver torque, per-step driver velocity, joint constraint impulse, and
non-cube contact telemetry, so it cannot identify a blocked mechanism versus
a solver/constraint disturbance. The repeated target readback and empty fault
list make a software target-application failure unlikely for replan 19.
The action rows' `measured_dq_*` fields describe arm joints, not the gripper
driver; its velocity was not recorded.

**Smallest follow-up:** If a cause is required, replay the saved replan-19 arm
joint targets in an isolated, uniquely named short diagnostic, with the cube
and scene unchanged, logging driver q/dq, active target, drive effort or force
limit, mimic-joint errors, and gripper/cube/table contacts at every 60 Hz step.
Compare against the successful replan-12 close. This is a proposed diagnostic,
not authorization to run it or to alter the mechanism.

## Verification and limits

`python3 -m unittest discover -s tests/unit -p 'test_sim_policy_video.py' -v`
passed five tests for ordering, timing, archive loading, request-image hash
checks, inference-pause exclusion and finalization. Related rollout,
execution, observation and velocity-analysis tests also passed (20 tests).
Host compilation of project Python directories passed. `make check` ran 423
tests but failed in five unrelated existing
places: three Python-3.8 incompatibilities and two assertions contradicted by
pre-existing local scene changes.
`make compile` could not connect to the Docker socket in this session;
host `compileall` passed.
The actual rollout verifies action-boundary camera recording and encoding on
the user's Isaac setup. Camera geometry and contact fidelity remain
unqualified as described in the compatibility review.
