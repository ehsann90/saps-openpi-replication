# FR3 + Robotiq 2F-85 + ZED Mini transforms

T_A_from_B maps column coordinates in B into A: p_A = T_A_from_B @ p_B. Translation in metres; matrices listed by rows.

## Frame definitions

- **flange**: Actual FR3 flange interface / standard Isaac fr3_link8 frame. STEP Link_Flange source frame translated +107 mm in its local Z.
- **hand_cad**: Source frame of fixed Robotiq body Defeatured 2F-85 PAD OPEN_base.step; not TCP, not Franka fr3_hand, and not a verified Robotiq USD or URDF frame. Local +Y points outward along flange +Z, local +X points along flange -Y, local +Z points along flange -X.
- **camera_cad**: Source frame of ZED M STEP, coincident with ZEDM in this file. Use the same source-local geometry origin when importing the camera.
- **robotiq_assembly_cad**: Parent Robotiq_2F85 / 2F85_Opened source assembly origin, offset from its body centreline; supplied separately for whole-assembly import.
- **left_optical**: Nominal LEFT optical centre; X image right, Y image down, Z forward. Prior optical registration reused.
- **usd_camera**: Native USD Camera frame: X right, Y up, -Z forward; same origin as left_optical.

## Results

### T_flange_from_hand_cad

```text
[[-0.        0.       -1.        0.      ]
 [-1.       -0.        0.       -0.      ]
 [-0.        1.        0.        0.013758]
 [ 0.        0.        0.        1.      ]]
```

### T_flange_from_camera_cad

```text
[[-0.           0.342529088 -0.939507224 -0.082109796]
 [-1.          -0.          -0.           0.00594922 ]
 [-0.           0.939507224  0.342529088 -0.004588467]
 [ 0.           0.           0.           1.         ]]
```

### T_camera_cad_from_left_optical_nominal

```text
[[ 0.999999997  0.000077464  0.000031391 -0.025981691]
 [-0.000031362 -0.000370025  0.999999931  0.007698006]
 [ 0.000077475 -0.999999929 -0.000370023  0.000019789]
 [ 0.           0.           0.           1.         ]]
```

### T_flange_from_left_optical_nominal

```text
[[-0.000083531  0.939380413  0.342876703 -0.079491597]
 [-0.999999997 -0.000077464 -0.000031391  0.031930911]
 [-0.000002927 -0.342876705  0.939380416  0.002650644]
 [ 0.           0.           0.           1.         ]]
```

### T_flange_from_usd_camera_nominal

```text
[[-0.000083531 -0.939380413 -0.342876703 -0.079491597]
 [-0.999999997  0.000077464  0.000031391  0.031930911]
 [-0.000002927  0.342876705 -0.939380416  0.002650644]
 [ 0.           0.           0.           1.         ]]
```

### T_hand_cad_from_camera_cad

```text
[[ 1.           0.           0.          -0.00594922 ]
 [-0.           0.939507224  0.342529088 -0.018346467]
 [-0.          -0.342529088  0.939507224  0.082109796]
 [ 0.           0.           0.           1.         ]]
```

### T_flange_from_robotiq_assembly_cad

```text
[[-0.           0.          -1.           0.093425734]
 [-1.          -0.           0.           0.         ]
 [-0.           1.           0.           0.013888956]
 [ 0.           0.           0.           1.         ]]
```

## Applying in Isaac Sim

Parent the camera under the flange / fr3_link8 frame and use T_flange_from_camera_cad for camera CAD geometry. For a native USD Camera prim use T_flange_from_usd_camera_nominal. The latter is already converted from optical axes.

The Robotiq hand matrix refers to the fixed base CAD component. The parent assembly origin differs: use T_flange_from_robotiq_assembly_cad only if importing the entire Robotiq assembly with its source origin preserved. Do not substitute either frame for the stock Franka fr3_hand.

The camera body translation is approximately (-82.110, +5.949, -4.588) mm in flange axes. The body tilt is 20.031 degrees relative to an untilted downward optical direction. The fixed Robotiq base CAD origin is on the flange centreline at +13.758 mm along flange Z.

The camera optical pose is nominal; the matrix precision describes CAD arithmetic, not physical calibration accuracy.

The base CAD frame is not necessarily the mounting face centre. Its axes and origin are taken directly from the fixed base STEP component; no invented hand/TCP frame is used.

Use the JSON for full precision matrices, inverses and position/quaternion values. See verification.json for independent extraction and source geometry checks.
