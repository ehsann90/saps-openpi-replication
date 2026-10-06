# Updated FR3 / ZED Mini transforms

This package supersedes `FR3_ZED_transforms.zip`. It uses the newly supplied `FR3_ZED_DROID_Rotated.step`, preserving the revised ZED mount. RealSense transforms are excluded.

## Verified hand alignment

After converting the source CAD hand frame to the simulation convention, its flange-relative frame matches NVIDIA Isaac Sim 5.1's standard `/fr3/fr3_hand` frame: rotation difference 0.00000266 degrees, origin difference below 0.000001 mm. These small values reflect numerical precision, not physical manufacturing accuracy. The earlier 45-degree hand clocking mismatch is resolved. The stock USD fixed hand joint can remain unchanged.

The ZED mount and hand have rotated together about the flange. This preserves the camera-to-finger relationship of the earlier CAD assembly while changing its transform relative to the simulation hand used in the previous package.

## Matrix convention

`T_A_from_B` maps homogeneous column coordinates from B into A: `p_A = T_A_from_B @ p_B`. Translation units: metres. Matrices stored by rows. Quaternion order: w,x,y,z.

- `T_fr3_hand_from_camera_cad`: camera STEP component origin in Isaac's hand frame.
- `T_camera_cad_from_left_optical_nominal`: nominal left optical camera frame in the camera STEP component frame; reused because the camera source geometry is unchanged.
- `T_fr3_hand_from_left_optical_nominal`: composition of those two transforms. Optical axes X right, Y down, Z forward.
- `T_fr3_hand_from_usd_camera_nominal`: same position with native USD Camera axes X right, Y up, -Z forward. This already includes the optical-to-USD rotation.
- Flange (`fr3_link8`) transforms and inverses are also included.

## Native USD Camera placement under fr3_hand

Position, metres:
`[-0.079489144607, +0.031927806250, +0.002650643753]`

Quaternion, wxyz:
`[0.123099065614, 0.696279310194, -0.696337080774, -0.123111381195]`

For a direct USD Camera child, use the native USD camera pose. For an optical-frame API, use the optical pose instead and let that API convert axes exactly once. A USD transform matrix uses row-vector conventions, so transpose the supplied homogeneous column-vector matrix when constructing a Gf.Matrix4d directly. Do not pass the USD-native quaternion to an API that also applies a ROS/optical conversion.

## Accuracy and provenance

Mount placement is taken from the STEP representation hierarchy, with no image-based placement estimates. The transform parser was cross-checked against the previous OpenCascade assembly extraction (maximum matrix element difference 1.12e-16). Both flange-based and CAD-hand-based derivations agree. Rotations passed determinant/orthogonality checks, and composition directions were verified.

Optical placement remains nominal, from the official Stereolabs ZED Mini URDF and mesh registration to the supplied ZED CAD. It is not per-unit optical or physical hand-eye calibration. Model differences include approximately 0.98 mm lateral difference between the nominal optical point and the exact lens-cylinder axis in the supplied CAD. Treat optical positioning as millimetre-scale nominal; numerical precision does not imply physical accuracy.

The JSON contains the full matrices, quaternions, inverse transforms, input SHA256 hashes, frame definitions, source URLs, registration provenance, and alignment checks.
