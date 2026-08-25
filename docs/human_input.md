# Human Input

Teleoperation and shared autonomy consume the same normalized
`HumanInputSample`. The selected motion source can be the browser keyboard or a
3Dconnexion SpaceMouse. The execution and arbitration loops do not contain
controller-specific branches.

The browser remains responsible for the camera view, explicit arming, abort,
runtime status, keyboard fallback, and gripper fallback. Browser connectivity
and SpaceMouse connectivity are recorded separately.

## SpaceMouse device and discovery

The verified lab device is a `3Dconnexion SpaceMouse Wireless` with USB ID
`256f:c62e`. Its stable cable-connected event path is:

```text
/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick
```

The backend selects devices in this order:

1. `--spacemouse-device-path`, when supplied;
2. sorted `/dev/input/by-id/*-event-joystick` nodes;
3. sorted `/dev/input/event*` nodes.

Every candidate must expose all of `ABS_X`, `ABS_Y`, `ABS_Z`, `ABS_RX`,
`ABS_RY`, and `ABS_RZ`. A device-name substring is never sufficient. The
runtime Compose service bind-mounts `/dev/input` so the stable host symlink and
its event node are available in the container.

Before reading motion, the backend acquires `EVIOCGRAB`. If exclusive access
fails, initialization fails clearly instead of producing an apparently valid
zero command. The diagnostic identifies `spacenavd` as a possible owner and
suggests inspecting the resolved event node with:

```bash
sudo fuser -v /dev/input/<event-node>
```

The known manual workaround is:

```bash
sudo systemctl stop spacenavd
```

The application never invokes `sudo`, stops a service, or kills a process.
`EVIOCGRAB` and the file descriptor are released on shutdown.

## Analog processing and defaults

The action convention remains:

```text
[dx, dy, dz, droll, dpitch, dyaw, gripper]
```

Each sample follows this testable sequence:

```text
raw / configured maximum
-> clip to [-1, 1]
-> component-wise rescaled deadzone
-> configured axis mapping and signs
-> translation or rotation gain
-> per-axis sensitivity scale and enable mask
-> final clip to [-1, 1]
```

For normalized magnitude `m` and deadzone `d`, values with `m <= d` become
zero. Other magnitudes become `(m - d) / (1 - d)`, preserving their sign.
There is no additional filtering.

The deadzone is applied independently to each raw axis around the neutral puck
position. With the default `d = 0.08`, the central 8% of travel in either
direction produces zero; values outside it are rescaled continuously so full
deflection still reaches normalized magnitude `1`. A larger deadzone suppresses
neutral drift and small accidental tilt, but also requires a larger deliberate
movement before an axis responds. The current profile has one shared deadzone
for all six axes, not separate translation and rotation deadzones.

Defaults are:

| Setting | Default |
|---|---|
| source | `keyboard` |
| translation gain | `0.14` |
| rotation gain | `0.18` |
| deadzone | `0.08` |
| mapping | `ABS_X,ABS_Y,ABS_Z,ABS_RX,ABS_RY,ABS_RZ` |
| signs | `1,1,1,1,1,1` |
| maxima | `350,350,350,350,350,350` |
| stale timeout | `0.25` seconds |
| open button | `256` (`BTN_0`) |
| close button | `257` (`BTN_1`) |

The mapping list is in application-axis order: its first entry supplies `dx`,
its second supplies `dy`, and so on. The identity mapping and positive signs are
the unchanged normal-runner defaults when no profile is supplied. Physical
shakedown established the following translation behavior:

- puck up/down is `ABS_Z`, inverted so puck up produces `+dz`;
- puck forward/back is `ABS_Y`, inverted so puck forward produces `+dx`;
- puck right/left is `ABS_X`, with puck right producing `+dy`.

Physical rotation shakedown subsequently established:

- the two planar rotation sources must be swapped;
- `roll` is supplied by inverted `ABS_RY`;
- `pitch` is supplied by `ABS_RX` with positive sign;
- `yaw` is supplied by `ABS_RZ` with its existing positive sign.

The physically validated profile uses mapping
`ABS_Y,ABS_X,ABS_Z,ABS_RY,ABS_RX,ABS_RZ` and signs
`-1,1,-1,-1,1,1`. It is stored at `configs/spacemouse_profile.json` with all six
outputs enabled. Calibration loads that profile by default. If it is absent or
loading is explicitly disabled, calibration starts with the same mapping and
signs but enables translation only for a safe staged shakedown. Normal runner
defaults remain unchanged until this profile is explicitly supplied.

The calibration-only translation-gain candidate is `0.30`, raised from `0.14`
after physical testing found both single-axis and mechanically coupled diagonal
motion too slow. This does not change the normal runner default or the analog
processing equation. Multi-axis SpaceMouse gestures commonly divide the puck's
physical deflection among axes, so increasing the linear gain compensates
without normalizing or otherwise reshaping the command vector.

The calibration-only rotation-gain candidate is `0.08`, reduced from `0.18`
after combined 6-DoF testing found that incidental puck tilt produced too much
gripper rotation during intended translation. This reduces both intentional and
incidental rotation proportionally; it does not introduce dominant-axis
suppression, filtering, or a different activation rule. The value remains
adjustable live while disarmed.

## Safety and gripper behavior

SpaceMouse motion is applied only while the browser is connected and armed,
abort is not active, the physical device is connected, and the most recent
axis event is not stale. Device loss immediately clears all six Cartesian
dimensions and button state. Reconnection does not restore authority: the
browser operator must intentionally arm again.

The SpaceMouse buttons and browser `Z`/`X` controls latch the same gripper
command. Close has deterministic priority if both configured buttons are
reported pressed in one sample. Shared-autonomy gripper arbitration is
unchanged.

## Diagnostic

The diagnostic starts no LIBERO environment, policy server, or robot:

```bash
make spacemouse-diagnostic \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick
```

It prints the selected name/path, six-axis capability ranges, exclusive-access
status, connection and stale state, raw axes, mapped axes, final 7-D action,
buttons, native event timestamp, and errors. Stop it with `Ctrl+C`.

After mapping and signs are physically verified, a teleoperation check can use:

```bash
make teleop \
  INPUT_SOURCE=spacemouse \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick \
  SPACEMOUSE_AXIS_MAPPING=ABS_X,ABS_Y,ABS_Z,ABS_RX,ABS_RY,ABS_RZ \
  SPACEMOUSE_AXIS_SIGNS=1,1,1,1,1,1 \
  TELEOP_OUTPUT=outputs/spacemouse_teleop_manual
```

Use a unique output directory. Shared-autonomy modes use the same input
overrides and still require the policy server.

## Graphical calibration shakedown

The graphical tool complements the console diagnostic. It starts the nominal
LIBERO scene and browser, but no policy server, arbitration, schedule, or
experiment logging:

```bash
make spacemouse-calibrate \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick
```

This is disposable calibration, not Gate-2 data. The browser shows connection,
selected device, arming and stale state, raw sliders for all six Linux axes,
mapped application-axis sliders, and the final 7-D action.

While disarmed, the operator can edit:

- the raw-axis source and sign for each of `dx`, `dy`, `dz`, `roll`, `pitch`,
  and `yaw`;
- translation gain, rotation gain, and deadzone;
- six dimensionless per-axis scales applied on top of those gains;
- six output enable checkboxes.

Use the three stages in order:

1. `Stage 1: Translation only`: verify forward/back, right/left, then up/down.
2. `Stage 2: Rotation only`: confirm the calibrated roll, inverted pitch, and
   unchanged yaw directions without translational motion.
3. `Stage 3: Enable all six`: verify combined motion and tune sensitivity.

Stage buttons edit the enable checkboxes but do not change live control until
`Apply` is clicked. Applying a configuration, resetting the nominal scene, or
saving a profile always disarms on the server. Applying also discards the
pre-change raw command, so a fresh device event is required. Review the values,
click Apply, release the puck to neutral, then explicitly re-arm.

During the rotation-only stage, record for each of the three gestures the
dominant raw axis and sign, dominant mapped output and sign, and observed
gripper rotation. Smaller simultaneous raw-axis values are physical coupling
and are intentionally prevented from reaching disabled translation outputs.

`Reset nominal scene` restores and settles the configured initial state without
restarting the container. `Abort episode` terminates the shakedown. Simulation
is paused while disarmed.

Calibration loads the committed `configs/spacemouse_profile.json` by default.
If the file is absent, it starts from the current gains/deadzone, all scales at
`1.0`, and the translation-only candidate described above. `Save profile`
atomically writes that path. The saved enable mask is explicit; all six values
must be `true` for the validated full 6-DoF configuration.

The versioned profile format is:

```json
{
  "schema_version": 1,
  "device_type": "3Dconnexion SpaceMouse Wireless",
  "axis_mapping": ["ABS_Y", "ABS_X", "ABS_Z", "ABS_RY", "ABS_RX", "ABS_RZ"],
  "axis_signs": [-1.0, 1.0, -1.0, -1.0, 1.0, 1.0],
  "axis_maxima": [350.0, 350.0, 350.0, 350.0, 350.0, 350.0],
  "translation_gain": 0.14,
  "rotation_gain": 0.08,
  "axis_scales": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
  "axis_enabled": [true, true, true, true, true, true],
  "deadzone": 0.08,
  "stale_input_timeout_seconds": 0.25,
  "open_button": 256,
  "close_button": 257
}
```

Profiles deliberately exclude `/dev/input/eventX` and the stable device path.
The latter remains a runtime argument. Existing single-episode teleoperation
and shared-autonomy runners can load a saved candidate without changing their
defaults:

```bash
make teleop \
  INPUT_SOURCE=spacemouse \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick \
  SPACEMOUSE_PROFILE=configs/spacemouse_profile.json \
  TELEOP_OUTPUT=outputs/spacemouse_profile_shakedown
```

This capability does not add the profile to any manifest or experimental
schedule.

## Logging

Existing per-step operator and arbitration fields are preserved. Each step and
scheduler wait additionally contains a `human_input` object with the source,
browser connection, physical connection, selected name/path, raw and mapped
axes, processed 7-D action, motion-active and stale state, gains, deadzone,
mapping, signs, maxima, configured and current buttons, device error, and native
event timestamp when available. Axis scales and the enable mask are also
recorded. Manifest-driven sessions freeze the runtime
selection in `human_input.json` without changing the manifest schema.
