# FR3 / DROID ZED Mini camera transforms

Calculated from the uploaded `FR3_ZED_DROID_Reference.step` and NVIDIA Isaac Sim 5.1's standard `FrankaRobotics/FrankaFR3/fr3.usd`.

## Use

Read `fr3_zed_camera_transforms.json`. A matrix named `T_A_from_B` converts homogeneous column coordinates from B to A: `p_A = T_A_from_B @ p_B`. All translations are metres. Quaternion order is **w, x, y, z**.

- `T_fr3_hand_from_camera_cad`: mounted ZED STEP component origin expressed in the standard USD hand link frame.
- `T_camera_cad_from_left_optical_nominal`: nominal left optical frame expressed in the camera STEP frame.
- `T_fr3_hand_from_left_optical_nominal`: their composition. Optical axes: X right, Y down, Z forward.
- `T_fr3_hand_from_usd_camera_nominal`: same viewpoint, rotated for a native USD Camera: X right, Y up, -Z forward. Use this when authoring a Camera prim directly under `fr3_hand`. Do not also apply an optical-axis conversion.
- `T_sim_fr3_link8_from_*`: transforms relative to the standard USD flange link.
- Inverse matrices are included for converting in the other direction.

Left optical position in the standard simulation hand: **[-0.0787836815, -0.0336309449, +0.0026506438] m**.
Native USD Camera quaternion wxyz: **[0.1608413931, 0.9097548678, -0.3768770203, -0.0666321124]**.

## Important CAD versus simulation difference

The CAD hand geometry is clocked approximately **45 degrees differently** around the flange from the stock USD hand. The primary transforms preserve the bracket/camera relative to the CAD wrist and flange, and then express that pose in the stock USD hand frame. They do not rotate the camera mount to compensate for differently positioned simulation fingers.

To reproduce both the CAD wrist mount and the CAD finger view, the simulation hand must also be clocked to match the CAD hand. The JSON includes explicitly named `T_cad_hand_geometry_from_*` alternative transforms for that matched hand geometry. These alternatives are NOT poses in the unchanged stock USD `fr3_hand` frame.

The standard USD fixed hand joint has `T_fr3_link8_from_fr3_hand = Rz(-45 degrees)` and zero translation. The CAD flange component frame is equivalent to the USD link7 geometry frame; the flange interface/link8 origin is +107 mm along its local Z. Comparing corresponding flange surfaces checks this correspondence. The CAD hand origin is internal to the housing; its flange interface is at local Z=-26.9 mm, and its CAD opening direction is X rather than the USD hand's Y.

## Accuracy

Camera mounting pose is read from the STEP assembly's hierarchy and placement matrices. The optical pose is **nominal**, transferred from Stereolabs' URDF and ZED Mini mesh into the supplied camera CAD by rigid surface registration. It is not a physical hand-eye calibration or per-unit optical-center measurement.

Nominal stereo baseline: 63 mm. The official model's left optical point is approximately 0.98 mm laterally displaced from the exact left lens-cylinder axis in the supplied CAD; fine model details also differ. Treat optical placement as millimetre-scale nominal. Do not use the front lens face as the optical origin. Preserve the supplied full precision for computation without interpreting decimal digits as optical accuracy.

All output rotations passed orthogonality and determinant checks. Composition and inverse directions are explicit. SHA256 hashes, STEP placements, model registration, and coordinate conversion records are in the JSON.

## Sources

- NVIDIA FR3 asset documentation: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_robots.html
- NVIDIA asset bytes: https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Robots/FrankaRobotics/FrankaFR3/fr3.usd
- Stereolabs URDF: https://github.com/stereolabs/zed-ros2-description/blob/main/urdf/zed_macro.urdf.xacro
- Stereolabs mesh: https://github.com/stereolabs/zed-ros2-description/blob/main/meshes/zedm.stl
- Optical-frame definitions: https://docs.stereolabs.com/docs/development/zed-sdk/modules/positional-tracking/coordinate-frames
