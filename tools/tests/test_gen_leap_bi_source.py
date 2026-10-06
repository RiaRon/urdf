"""The LEAP source URDF: one connected tree, palm seated on the mount, RH56F1 orientation."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR))

import gen_leap_bi_source as gen  # noqa: E402
from audit_self_collision import UrdfModel, origin_transform  # noqa: E402


@pytest.fixture(scope="module")
def urdf(tmp_path_factory) -> Path:
    path = tmp_path_factory.mktemp("leap") / "openarm_bi_leap.urdf"
    gen.build().write(path, encoding="utf-8", xml_declaration=True)
    return path


def test_single_tree_with_both_hands(urdf: Path) -> None:
    model = UrdfModel(urdf)
    links = {link.get("name") for link in model.root.findall("link")}
    roots = links - set(model.parent_of)
    assert roots == {"world"}
    assert not any("gripper_mount" in name for name in links)
    for side in ("left", "right"):
        revolute = [j for j in model.root.findall("joint")
                    if j.get("name").startswith(f"leap_{side}_") and j.get("type") == "revolute"]
        assert len(revolute) == 16
    leap_meshes = [m.get("filename") for m in model.root.iter("mesh") if "/leap_hand/" in m.get("filename")]
    assert leap_meshes and all(Path(f.removeprefix("file://")).is_file() for f in leap_meshes)


@pytest.mark.parametrize("side, palm_normal_y", [("right", -1.0), ("left", 1.0)])
def test_palm_seated_and_oriented_like_rh56f1(urdf: Path, side: str, palm_normal_y: float) -> None:
    link7 = f"openarm_{side}_link7"
    p = f"leap_{side}_"

    def in_link7(model: UrdfModel, link: str) -> np.ndarray:
        rel = np.linalg.inv(model.world_transform(link7)) @ model.world_transform(link)
        return rel[:3, 3]

    rest = UrdfModel(urdf)
    palm = next(l for l in rest.root.findall("link") if l.get("name").startswith(p + "palm_lower"))
    assert rest.parent_of[palm.get("name")][0] == link7   # dex-urdf `base` frame folded away
    visual = palm.find("visual")
    obj = Path(visual.find("geometry/mesh").get("filename").removeprefix("file://"))
    verts = np.array([[float(v) for v in line.split()[1:4]]
                      for line in obj.read_text().splitlines() if line.startswith("v ")])
    to_link7 = (np.linalg.inv(rest.world_transform(link7)) @ rest.world_transform(palm.get("name"))
                @ origin_transform(visual))
    pts = verts @ to_link7[:3, :3].T + to_link7[:3, 3]
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    seat = [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]]   # palm bottom-center
    mount_xyz = [float(v) for v in gen.mount_origin(side)[0].split()]   # assembly xacro arg
    assert np.allclose(seat, mount_xyz, atol=1e-5)

    index_tip = in_link7(rest, p + "index_tip_head")
    assert index_tip[2] > mount_xyz[2] + 0.15   # fingers point along link7 +z
    assert index_tip[0] > 0.0                  # index toward link7 +x

    flexed = in_link7(UrdfModel(urdf, {p + "1": 1.0}), p + "index_tip_head")
    assert (flexed - index_tip)[1] * palm_normal_y > 0.05   # index curls toward the palm side


def test_ros_assembly_matches_rl_source(urdf: Path, tmp_path: Path) -> None:
    """assemblies/openarm_modular_dual_leap.xacro (ROS) and the RL source carry the same hands.

    Needs a sourced ROS 2 + this workspace's install/ (leap_hand_description built)."""
    xacro = pytest.importorskip("xacro")
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory("leap_hand_description")
    except Exception:
        pytest.skip("leap_hand_description is not built/sourced")
    ros_path = tmp_path / "openarm_dual_leap.urdf"
    ros_path.write_text(xacro.process_file(str(gen.ASSEMBLY)).toxml())
    ros, rl = UrdfModel(ros_path), UrdfModel(urdf)

    def rel(model: UrdfModel, a: str, b: str) -> np.ndarray:
        return np.linalg.inv(model.world_transform(a)) @ model.world_transform(b)

    for side in ("left", "right"):
        palm = next(l.get("name") for l in rl.root.findall("link")
                    if l.get("name").startswith(f"leap_{side}_palm_lower"))
        link7 = f"openarm_{side}_link7"
        assert np.allclose(rel(ros, link7, palm), rel(rl, link7, palm), atol=1e-6)

    def hand_joints(model: UrdfModel) -> dict:
        return {j.get("name"): (j.get("type"), j.find("limit").attrib if j.find("limit") is not None else None)
                for j in model.root.findall("joint")
                if j.get("name").startswith("leap_") and j.get("type") != "fixed"}
    assert hand_joints(ros) == hand_joints(rl) and len(hand_joints(rl)) == 32
