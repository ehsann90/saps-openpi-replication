"""Independent Pinocchio validation backend for the lab FR3 description."""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any

import numpy as np

from saps.physical.embodiment import FR3_JOINT_NAMES


class Fr3PinocchioKinematics:
    """Independent FR3 Jacobian/FK checks with no robot connection.

    The returned geometric Jacobian describes the ``fr3_hand_tcp`` origin
    relative to ``fr3_link0``. Its first three rows are linear velocity in
    metres/second and its last three are angular velocity in radians/second;
    both are resolved along ``fr3_link0`` axes.
    """

    joint_names = FR3_JOINT_NAMES

    def __init__(
        self,
        model: Any,
        pinocchio_module: Any,
        *,
        urdf_xml: str,
        source_xacro_path: Path,
        base_frame: str = "fr3_link0",
        end_effector_frame: str = "fr3_hand_tcp",
    ) -> None:
        self._pin = pinocchio_module
        self._model = model
        self.base_frame = base_frame
        self.end_effector_frame = end_effector_frame
        self.source_xacro_path = source_xacro_path.resolve()
        self.urdf_sha256 = hashlib.sha256(
            urdf_xml.encode("utf-8")
        ).hexdigest()
        if model.nq != 7 or model.nv != 7:
            raise ValueError(
                "Reduced FR3 model must have exactly seven arm DoF; "
                f"received nq={model.nq}, nv={model.nv}."
            )
        actual_joint_names = tuple(model.names[1:])
        if actual_joint_names != self.joint_names:
            raise ValueError(
                "FR3 model joint order does not match the verified order: "
                f"{actual_joint_names!r}."
            )
        for frame in (base_frame, end_effector_frame):
            if not model.existFrame(frame):
                raise ValueError(f"FR3 model has no frame {frame!r}.")
        self._base_frame_id = model.getFrameId(base_frame)
        self._end_effector_frame_id = model.getFrameId(
            end_effector_frame
        )

    @classmethod
    def from_xacro(
        cls,
        xacro_path: str | Path,
        *,
        base_frame: str = "fr3_link0",
        end_effector_frame: str = "fr3_hand_tcp",
    ) -> "Fr3PinocchioKinematics":
        """Expand the actual lab xacro and lock the two finger joints."""

        try:
            import pinocchio as pin
            import xacro
        except ImportError as error:
            raise RuntimeError(
                "FR3 model validation requires ROS Jazzy's xacro and "
                "Pinocchio Python packages. Source /opt/ros/jazzy/setup.bash."
            ) from error

        source_path = Path(xacro_path).resolve()
        if not source_path.is_file():
            raise FileNotFoundError(f"FR3 xacro not found: {source_path}")
        urdf_xml = xacro.process_file(
            str(source_path),
            mappings={"hand": "true", "ros2_control": "false"},
        ).toxml()
        full_model = pin.buildModelFromXML(urdf_xml)
        finger_names = (
            "fr3_finger_joint1",
            "fr3_finger_joint2",
        )
        missing = [
            name
            for name in finger_names
            if not full_model.existJointName(name)
        ]
        if missing:
            raise ValueError(
                f"FR3 hand model is missing finger joints: {missing}."
            )
        reduced_model = pin.buildReducedModel(
            full_model,
            [full_model.getJointId(name) for name in finger_names],
            pin.neutral(full_model),
        )
        return cls(
            reduced_model,
            pin,
            urdf_xml=urdf_xml,
            source_xacro_path=source_path,
            base_frame=base_frame,
            end_effector_frame=end_effector_frame,
        )

    @property
    def backend_version(self) -> str:
        """Return the selected Pinocchio version."""

        return str(self._pin.__version__)

    @property
    def position_lower_rad(self) -> np.ndarray:
        """Return arm lower position limits from expanded URDF."""

        return _readonly(self._model.lowerPositionLimit)

    @property
    def position_upper_rad(self) -> np.ndarray:
        """Return arm upper position limits from expanded URDF."""

        return _readonly(self._model.upperPositionLimit)

    @property
    def velocity_limit_rad_s(self) -> np.ndarray:
        """Return arm velocity limits from expanded URDF."""

        return _readonly(self._model.velocityLimit)

    def jacobian(self, joint_position: np.ndarray) -> np.ndarray:
        """Return base-resolved [linear; angular] Jacobian at current q."""

        q = self._joint_position(joint_position)
        data = self._model.createData()
        self._pin.forwardKinematics(self._model, data, q)
        self._pin.updateFramePlacements(self._model, data)
        jacobian_world = self._pin.computeFrameJacobian(
            self._model,
            data,
            q,
            self._end_effector_frame_id,
            self._pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        world_rotation_base = data.oMf[self._base_frame_id].rotation
        rotate_to_base = world_rotation_base.T
        jacobian_base = np.empty((6, 7), dtype=np.float64)
        jacobian_base[:3] = rotate_to_base @ jacobian_world[:3]
        jacobian_base[3:] = rotate_to_base @ jacobian_world[3:]
        return _readonly(jacobian_base)

    def forward_kinematics(self, joint_position: np.ndarray) -> np.ndarray:
        """Return base-to-selected-frame homogeneous transform for q."""

        q = self._joint_position(joint_position)
        data = self._model.createData()
        self._pin.forwardKinematics(self._model, data, q)
        self._pin.updateFramePlacements(self._model, data)
        base_to_tcp = (
            data.oMf[self._base_frame_id].inverse()
            * data.oMf[self._end_effector_frame_id]
        )
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = base_to_tcp.rotation
        transform[:3, 3] = base_to_tcp.translation
        return _readonly(transform)

    def finite_step_displacement(
        self,
        joint_position: np.ndarray,
        delta_q_rad: np.ndarray,
    ) -> np.ndarray:
        """Return exact FK finite displacement in base-resolved coordinates."""

        q = self._joint_position(joint_position)
        delta = self._joint_position(delta_q_rad)
        initial = self.forward_kinematics(q)
        final = self.forward_kinematics(q + delta)
        translation = final[:3, 3] - initial[:3, 3]
        rotation = self._pin.log3(
            final[:3, :3] @ initial[:3, :3].T
        )
        return _readonly(np.concatenate((translation, rotation)))

    def _joint_position(self, value: np.ndarray) -> np.ndarray:
        result = np.asarray(value)
        if not np.issubdtype(result.dtype, np.floating):
            raise TypeError("FR3 joint vector must have a floating dtype.")
        if result.shape != (7,):
            raise ValueError(
                "FR3 joint vector must have shape (7,), received "
                f"{result.shape}."
            )
        if not np.all(np.isfinite(result)):
            raise ValueError("FR3 joint vector must contain finite values.")
        return np.array(result, dtype=np.float64, copy=True)


def _readonly(value: np.ndarray) -> np.ndarray:
    result = np.array(value, dtype=np.float64, copy=True)
    result.setflags(write=False)
    return result
