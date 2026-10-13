"""Read-only HG1-B1 composed-USD and scene-selection audit (no SimulationApp)."""

from __future__ import annotations

import json
import hashlib
from pathlib import Path

from pxr import Sdf, Usd, UsdGeom, UsdPhysics


ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "sim/assets"
BASE = ASSETS / "fr3_robotiq_droid_historical_hg1.usda"
CANDIDATE = ASSETS / "fr3_robotiq_droid_fusion_v13_candidate.usda"
BASE_CONFIG = (ROOT / "sim/configs/"
               "fr3_droid_robotiq_historical_hg1_scene.json")
CANDIDATE_CONFIG = (ROOT / "sim/configs/"
                    "fr3_droid_robotiq_fusion_v13_candidate_scene.json")
RECORD = (ASSETS / "FR3_2F85_ZED_transforms/"
          "fusion_v13_candidate.json")
LINK = "/fr3/fr3_link8"
MOUNT = LINK + "/Robotiq_2F_85"
ROBOT = MOUNT + "/Robotiq_2F_85"
BASE_LINK = ROBOT + "/base_link"
JOINT = LINK + "/robotiq_mount_joint"
FINGER_JOINTS = (
    "finger_joint", "right_outer_knuckle_joint",
    "left_outer_finger_joint", "right_outer_finger_joint",
    "left_inner_knuckle_joint", "right_inner_knuckle_joint",
    "left_inner_finger_joint", "right_inner_finger_joint",
    "left_inner_finger_knuckle_joint",
    "right_inner_finger_knuckle_joint",
)
ARM_JOINTS = tuple(
    f"/fr3/fr3_link{index - 1}/fr3_joint{index}"
    for index in range(1, 8)
)


def near(actual: object, expected: object, tol: float = 1e-9) -> bool:
    """Compare USD scalar/vector/matrix values without mutating the stage."""
    if hasattr(actual, "GetReal") and hasattr(actual, "GetImaginary"):
        actual = (actual.GetReal(), *actual.GetImaginary())
    if isinstance(expected, (list, tuple)):
        return len(actual) == len(expected) and all(
            near(a, b, tol) for a, b in zip(actual, expected)
        )
    return abs(float(actual) - float(expected)) <= tol


def value(stage: Usd.Stage, path: str, name: str) -> object:
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise AssertionError(f"Missing composed prim {path}")
    result = prim.GetAttribute(name).Get()
    if result is None:
        raise AssertionError(f"Missing composed attribute {path}.{name}")
    return result


def snapshot(stage: Usd.Stage, path: str) -> dict:
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise AssertionError(f"Missing composed prim {path}")
    return {
        "type": prim.GetTypeName(),
        "active": prim.IsActive(),
        "apis": sorted(prim.GetAppliedSchemas()),
        "attributes": {
            a.GetName(): str(a.Get()) for a in prim.GetAttributes()
        },
        "relationships": {
            r.GetName(): [str(p) for p in r.GetTargets()]
            for r in prim.GetRelationships()
        },
    }


def relative_pose(stage: Usd.Stage, child: str) -> list[list[float]]:
    cache = UsdGeom.XformCache()
    matrix, reset = cache.ComputeRelativeTransform(
        stage.GetPrimAtPath(child), stage.GetPrimAtPath(LINK)
    )
    if reset:
        raise AssertionError(f"Transform resets before {child}")
    # Gf matrices multiply row vectors; JSON CAD matrices multiply columns.
    return [[float(matrix[j][i]) for j in range(4)] for i in range(4)]


def main() -> None:
    historical = json.loads(BASE_CONFIG.read_text())
    candidate_config = json.loads(CANDIDATE_CONFIG.read_text())
    assert historical["robot"]["asset"] == (
        "assets/fr3_robotiq_droid_historical_hg1.usda"
    )
    assert candidate_config["robot"]["asset"] == (
        "assets/fr3_robotiq_droid_fusion_v13_candidate.usda"
    )
    expected_config = json.loads(BASE_CONFIG.read_text())
    expected_config["robot"]["asset"] = candidate_config["robot"]["asset"]
    assert candidate_config == expected_config, "Non-asset scene setting changed"

    layer = Sdf.Layer.FindOrOpen(str(CANDIDATE))
    assert layer is not None
    assert len(layer.rootPrims) == 1
    root_spec = layer.GetPrimAtPath("/fr3")
    assert len(root_spec.referenceList.prependedItems) == 1
    assert root_spec.referenceList.prependedItems[0].assetPath == (
        "./fr3_robotiq_droid_historical_hg1.usda"
    )
    # The only third opinion is the separately documented visual placement.
    authored = set()

    def visit(spec: Sdf.PrimSpec) -> None:
        for prop in spec.properties:
            authored.add((str(spec.path), prop.name))
        for child in spec.nameChildren:
            visit(child)

    visit(root_spec)
    assert authored == {
        (MOUNT, "xformOp:translate"), (JOINT, "physics:localPos0"),
        (LINK + "/GRP_CPL_062_visual", "xformOp:translate"),
    }, authored

    baseline = Usd.Stage.Open(str(BASE))
    candidate = Usd.Stage.Open(str(CANDIDATE))
    assert baseline and candidate
    record = json.loads(RECORD.read_text())
    assert record["source_fusion_design_version"] == 13
    assert record["historical_assembly_sha256"] == hashlib.sha256(
        BASE.read_bytes()).hexdigest()
    assert record["historical_scene_sha256"] == hashlib.sha256(
        BASE_CONFIG.read_bytes()).hexdigest()
    cad_root = ROOT / "sim/cad_reference/fr3_robotiq_zed"
    for field, source in (
        ("source_f3z_sha256", cad_root / "native/FR3_2F85_ZED.f3z"),
        ("source_step_sha256", cad_root / "step/FR3_2F85_ZED.step"),
        ("source_frames_sha256", cad_root / "measurements/"
         "fusion_frames_20261013_102402.json"),
        ("source_faces_sha256", cad_root / "measurements/"
         "hg1_fusion_faces_20261013T083300663567Z.json"),
    ):
        assert record[field] == hashlib.sha256(source.read_bytes()).hexdigest()
    assert record["face_qualification"] == (
        "plane_qualified_trim_containment_unresolved for all six v2 faces"
    )
    assert near(value(baseline, MOUNT, "xformOp:translate"),
                (0, 0, 0.013758))
    assert near(value(candidate, MOUNT, "xformOp:translate"),
                (0, 0, 0.010758))
    assert near(value(baseline, JOINT, "physics:localPos0"),
                (0, 0, 0.013758))
    assert near(value(candidate, JOINT, "physics:localPos0"),
                (0, 0, 0.010758))

    for path, excluded in ((MOUNT, {"xformOp:translate"}),
                           (JOINT, {"physics:localPos0"}),
                           (LINK + "/GRP_CPL_062_visual",
                            {"xformOp:translate"})):
        before, after = snapshot(baseline, path), snapshot(candidate, path)
        for name in excluded:
            before["attributes"].pop(name)
            after["attributes"].pop(name)
        assert before == after, f"Unexpected composed change at {path}"

    unchanged_paths = [
        ROBOT, BASE_LINK, BASE_LINK + "/visuals",
        BASE_LINK + "/grip_frame", ROBOT + "/left_fingertip",
        ROBOT + "/right_fingertip",
        *(ROBOT + "/Joints/" + name for name in FINGER_JOINTS),
        *(ROBOT + "/" + side + "/visuals/tip_standard/"
          "Defeatured_2F_85_PAD_OPEN_fingertipsstep"
          for side in ("left_fingertip", "right_fingertip")),
    ]
    for path in unchanged_paths:
        assert snapshot(baseline, path) == snapshot(candidate, path), path
    stock_arm_available = all(
        baseline.GetPrimAtPath(path).IsValid() and
        candidate.GetPrimAtPath(path).IsValid()
        for path in ARM_JOINTS
    )
    if stock_arm_available:
        for path in ARM_JOINTS:
            assert snapshot(baseline, path) == snapshot(candidate, path), path
            assert candidate.GetPrimAtPath(path).IsA(
                UsdPhysics.RevoluteJoint), path
        assert snapshot(baseline, LINK) == snapshot(candidate, LINK)
    assert (historical["cameras"]["wrist"] ==
            candidate_config["cameras"]["wrist"])
    assert candidate_config["cameras"]["wrist"]["parent_prim"] == (
        "/World/fr3/fr3_link8"
    )
    for path in (BASE_LINK, ROBOT + "/left_fingertip",
                 ROBOT + "/right_fingertip"):
        assert candidate.GetPrimAtPath(path).HasAPI(UsdPhysics.RigidBodyAPI)
        assert candidate.GetPrimAtPath(path).HasAPI(UsdPhysics.MassAPI)
    assert near(value(candidate, BASE_LINK + "/visuals", "xformOp:orient"),
                (0.5, 0.5, 0.5, 0.5))
    assert near(value(candidate, BASE_LINK + "/visuals", "xformOp:translate"),
                (0, 0, 0))
    for side in ("left_fingertip", "right_fingertip"):
        path = (ROBOT + "/" + side + "/visuals/tip_standard/"
                "Defeatured_2F_85_PAD_OPEN_fingertipsstep")
        prim = candidate.GetPrimAtPath(path)
        assert prim.HasAPI(UsdPhysics.CollisionAPI), path
        assert value(candidate, path, "physics:collisionEnabled") is True
    assert near(value(baseline, LINK + "/GRP_CPL_062_visual",
                      "xformOp:translate"), (0.00207, 0.06565, -0.20265))
    assert near(value(candidate, LINK + "/GRP_CPL_062_visual",
                      "xformOp:translate"), (0.00207, 0.06565, -0.20565))
    coupling = Usd.Stage.Open(str(ASSETS / "robotiq_2f85_upstream/"
                                   "GRP-CPL-062.usd"))
    assert coupling
    coupling_prims = list(coupling.Traverse())
    assert coupling_prims
    for prim in coupling_prims:
        assert not any(api.startswith(("Physics", "Physx"))
                       for api in prim.GetAppliedSchemas()), prim.GetPath()
        assert not any(attr.HasAuthoredValueOpinion() and
                       attr.GetName().startswith(("physics:", "physx:"))
                       for attr in prim.GetAttributes()), prim.GetPath()

    matrices = record["matrices_m"]
    base_pose = relative_pose(candidate, BASE_LINK)
    grip_pose = relative_pose(candidate, BASE_LINK + "/grip_frame")
    assert near(base_pose, matrices["T_flange_from_usd_base"])
    assert near(grip_pose, matrices["T_flange_from_usd_grip"])
    assert near(base_pose[2][3], 0.010758)
    assert near(grip_pose[2][3], 0.14447506760793903)
    assert near(value(candidate, JOINT, "physics:localPos1"), (0, 0, 0))
    assert [str(p) for p in candidate.GetPrimAtPath(JOINT).GetRelationship(
        "physics:body0").GetTargets()] == [LINK]
    assert [str(p) for p in candidate.GetPrimAtPath(JOINT).GetRelationship(
        "physics:body1").GetTargets()] == [BASE_LINK]
    assert near(value(candidate, JOINT, "physics:localRot0"), (0, 0, 0, 1))
    assert near(value(candidate, JOINT, "physics:localRot1"), (1, 0, 0, 0))
    assert near(value(candidate, MOUNT, "xformOp:orient"), (0, 0, 0, 1))

    # The joint anchor is authored in link-8 and base-link coordinates.
    # At authored zero pose, the relative base-link origin equals localPos0.
    assert near(base_pose[0][3:4] + base_pose[1][3:4]
                + base_pose[2][3:4], (0, 0, 0.010758))

    print(json.dumps({
        "status": "local_composed_checks_passed",
        "historical_asset": str(BASE),
        "candidate_asset": str(CANDIDATE),
        "authored_candidate_properties": sorted(authored),
        "base_pose_flange_column_matrix": base_pose,
        "grip_pose_flange_column_matrix": grip_pose,
        "finger_joint_paths_compared": len(FINGER_JOINTS),
        "fingertip_collision_meshes_compared": 2,
        "coupling_visual_prims_without_physics": len(coupling_prims),
        "other_unchanged_prim_paths_compared": len(unchanged_paths),
        "stock_fr3_arm_composed_and_compared": stock_arm_available,
        "wrist_camera_config_equal": True,
        "limitation": (None if stock_arm_available else
                       "Remote stock FR3 reference unresolved; seven-arm "
                       "composed prims cannot be read. Camera is authored "
                       "at runtime from unchanged scene configuration."),
    }, indent=2))


if __name__ == "__main__":
    main()
