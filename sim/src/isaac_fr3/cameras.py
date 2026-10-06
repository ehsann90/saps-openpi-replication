"""Virtual RGB cameras for the Isaac FR3 DROID-like baseline."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from isaacsim.sensors.camera import Camera


# USD camera axes: +X image right, +Y image up, -Z forward. Poses are
# scalar-first quaternions. Conventional optical axes are +X right, +Y down,
# +Z forward; image arrays use top-left origin without flips or mirroring.
USD_CAMERA_FRAME = "+X right, +Y up, -Z forward"
OPTICAL_FRAME = "+X right, +Y down, +Z forward"
VIRTUAL_HORIZONTAL_APERTURE_M = 0.036


@dataclass
class CameraRig:
    wrist: Camera
    external: Camera


def create_camera(spec: dict, name: str) -> Camera:
    """Create one RGB-only camera at the configured parent-relative USD pose."""
    prim_path = spec["prim_path"]
    parent = spec["parent_prim"]
    if not prim_path.startswith(parent.rstrip("/") + "/"):
        raise ValueError(f"Camera {prim_path} is not a child of {parent}")
    width, height = map(int, spec["resolution_wh"])
    if width <= 0 or height <= 0:
        raise ValueError("Camera resolution must be positive")
    fov = float(spec["horizontal_fov_deg"])
    if not 0.0 < fov < 180.0:
        raise ValueError("Camera horizontal FOV must be between 0 and 180 degrees")
    camera = Camera(prim_path=prim_path, name=name, resolution=(width, height))
    camera.set_local_pose(
        translation=np.asarray(spec["position_xyz_m"], dtype=np.float64),
        orientation=np.asarray(spec["orientation_wxyz"], dtype=np.float64),
        camera_axes="usd",
    )
    camera.set_horizontal_aperture(VIRTUAL_HORIZONTAL_APERTURE_M)
    camera.set_clipping_range(*spec["clipping_range_m"])
    camera.set_focal_length(
        VIRTUAL_HORIZONTAL_APERTURE_M / (2.0 * math.tan(math.radians(fov) / 2.0))
    )
    camera.initialize(attach_rgb_annotator=True)
    return camera


def create_cameras(config: dict) -> CameraRig:
    """Attach the wrist camera to the hand and the external camera to World."""
    specs = config["cameras"]
    return CameraRig(
        external=create_camera(specs["external"], "external_rgb"),
        wrist=create_camera(specs["wrist"], "wrist_rgb"),
    )


def capture_rgb(camera: Camera, resolution_wh: list[int]) -> np.ndarray:
    """Read a valid nonuniform RGB uint8 frame after rendering has warmed up."""
    rgb = camera.get_rgb()
    if rgb is None:
        raise RuntimeError(f"No RGB frame from {camera.prim_path}")
    rgb = np.asarray(rgb)
    width, height = resolution_wh
    if rgb.shape != (height, width, 3) or rgb.dtype != np.uint8:
        raise RuntimeError(
            f"Unexpected RGB frame from {camera.prim_path}: "
            f"shape={rgb.shape}, dtype={rgb.dtype}"
        )
    if not np.isfinite(rgb).all() or not np.any(rgb) or not np.any(rgb != rgb[0, 0]):
        raise RuntimeError(f"Empty or uniform RGB frame from {camera.prim_path}")
    return rgb.copy()


def camera_metadata(camera: Camera, spec: dict) -> dict:
    """Report configured and measured camera geometry at capture time."""
    local_pos, local_quat = camera.get_local_pose(camera_axes="usd")
    world_pos, world_quat = camera.get_world_pose(camera_axes="usd")
    return {
        "model": spec["model"],
        "calibration": spec["calibration"],
        "prim_path": spec["prim_path"],
        "parent_prim": spec["parent_prim"],
        "frame": USD_CAMERA_FRAME,
        "optical_frame": OPTICAL_FRAME,
        "parent_relative_position_xyz_m": np.asarray(local_pos).tolist(),
        "parent_relative_orientation_wxyz": np.asarray(local_quat).tolist(),
        "world_position_xyz_m": np.asarray(world_pos).tolist(),
        "world_orientation_wxyz": np.asarray(world_quat).tolist(),
        "resolution_wh": list(camera.get_resolution()),
        "horizontal_fov_deg": float(spec["horizontal_fov_deg"]),
        "focal_length_m": float(camera.get_focal_length()),
        "horizontal_aperture_m": float(camera.get_horizontal_aperture()),
        "vertical_aperture_m": float(camera.get_vertical_aperture()),
        "clipping_range_m": list(camera.get_clipping_range()),
        "intrinsics_matrix_pixels": np.asarray(camera.get_intrinsics_matrix()).tolist(),
    }
