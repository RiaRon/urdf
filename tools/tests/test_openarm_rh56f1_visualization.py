from __future__ import annotations

import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
from urllib.parse import unquote, urlparse

import pytest
import yaml


TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR))

from openarm_rh56f1_visualization import (  # noqa: E402
    HAND_CONFIGURATIONS,
    PALM_CONTROL_FRAMES,
    build_robot_description,
    build_robot_element,
)


EXPECTED_MOUNTS = {
    "left": {
        "joint": "l_hj_mount",
        "parent": "l_al_7",
        "child": "l_hl_base",
        "xyz": (0.0, 0.0, 0.0595695),
        "rpy": (0.0, 0.0, 1.5707964),
    },
    "right": {
        "joint": "r_hj_mount",
        "parent": "r_al_7",
        "child": "r_hl_base",
        "xyz": (0.0, 0.0000003, 0.0595695),
        "rpy": (0.0, 0.0, -1.5707964),
    },
}

EXPECTED_PALM_SENSORS = {
    "left": {
        "joint": "l_hj_palm_sensor",
        "parent": "l_hl_palm_2",
        "link": "l_hl_palm_sensor",
        "xyz": (0.01594, -0.0013505, 0.073746),
        "rpy": (1.5708, 0.0, 1.5708),
    },
    "right": {
        "joint": "r_hj_palm_sensor",
        "parent": "r_hl_palm_2",
        "link": "r_hl_palm_sensor",
        "xyz": (0.0159401947506102, -0.00135045394126701, 0.0737460952299602),
        "rpy": (1.5707963267949, 0.0, 1.5707963267949),
    },
}

FABRIC_MANIFEST = (
    TOOLS_DIR.parent
    / "generated/fabric/openarm_rh56f1_bi/openarm_rh56f1_bi_manifest.yaml"
)


def _as_vector(text: str) -> tuple[float, float, float]:
    return tuple(float(value) for value in text.split())


def _assert_tree(root: ET.Element) -> None:
    links = [link.get("name") for link in root.findall("link")]
    joints = [joint.get("name") for joint in root.findall("joint")]
    assert None not in links
    assert None not in joints
    assert len(links) == len(set(links)), "duplicate link name"
    assert len(joints) == len(set(joints)), "duplicate joint name"

    link_set = set(links)
    children: dict[str, list[str]] = {name: [] for name in links}
    child_to_joint: dict[str, str] = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        assert parent in link_set
        assert child in link_set
        assert child not in child_to_joint, f"multiple parents for {child}"
        child_to_joint[child] = joint.get("name")
        children[parent].append(child)

    roots = link_set - set(child_to_joint)
    assert roots == {"body_root"}

    visited: set[str] = set()
    active: set[str] = set()

    def walk(link: str) -> None:
        assert link not in active, f"cycle at {link}"
        if link in visited:
            return
        active.add(link)
        for child in children[link]:
            walk(child)
        active.remove(link)
        visited.add(link)

    walk("body_root")
    assert visited == link_set, "disconnected links"


@pytest.mark.parametrize("configuration", HAND_CONFIGURATIONS)
def test_all_variants_generate_one_valid_tree(configuration: str) -> None:
    description = build_robot_description(configuration)
    assert description.strip().startswith("<?xml")
    root = ET.fromstring(description)
    _assert_tree(root)

    links = {link.get("name") for link in root.findall("link")}
    assert {"l_al_7", "r_al_7"} <= links
    assert ("l_hl_base" in links) is (configuration in {"left", "both"})
    assert ("r_hl_base" in links) is (configuration in {"right", "both"})


@pytest.mark.parametrize("configuration", ("left", "right", "both"))
def test_requested_hands_keep_authoritative_mounts_and_frames(
    configuration: str,
) -> None:
    root = build_robot_element(configuration)
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    links = {link.get("name") for link in root.findall("link")}

    enabled = {configuration} if configuration != "both" else {"left", "right"}
    for side in enabled:
        mount = EXPECTED_MOUNTS[side]
        joint = joints[mount["joint"]]
        assert joint.get("type") == "fixed"
        assert joint.find("parent").get("link") == mount["parent"]
        assert joint.find("child").get("link") == mount["child"]
        assert _as_vector(joint.find("origin").get("xyz")) == pytest.approx(
            mount["xyz"]
        )
        assert _as_vector(joint.find("origin").get("rpy")) == pytest.approx(
            mount["rpy"]
        )

        sensor = EXPECTED_PALM_SENSORS[side]
        sensor_joint = joints[sensor["joint"]]
        assert sensor["link"] in links
        assert sensor_joint.get("type") == "fixed"
        assert sensor_joint.find("parent").get("link") == sensor["parent"]
        assert sensor_joint.find("child").get("link") == sensor["link"]
        assert _as_vector(sensor_joint.find("origin").get("xyz")) == pytest.approx(
            sensor["xyz"]
        )
        assert _as_vector(sensor_joint.find("origin").get("rpy")) == pytest.approx(
            sensor["rpy"]
        )


@pytest.mark.parametrize("configuration", HAND_CONFIGURATIONS)
def test_only_selected_control_frames_are_present(configuration: str) -> None:
    root = build_robot_element(configuration)
    links = {link.get("name") for link in root.findall("link")}
    enabled = (
        {"left", "right"}
        if configuration == "both"
        else set()
        if configuration == "arm_only"
        else {configuration}
    )

    for side, frame in PALM_CONTROL_FRAMES.items():
        assert (frame in links) is (side in enabled)


def test_control_frame_names_match_fabric_manifest() -> None:
    manifest = yaml.safe_load(FABRIC_MANIFEST.read_text(encoding="utf-8"))

    assert manifest["palm_frames"] == [
        PALM_CONTROL_FRAMES["right"],
        PALM_CONTROL_FRAMES["left"],
    ]


def test_arm_only_removes_only_hand_subtrees() -> None:
    root = build_robot_element("arm_only")
    links = {link.get("name") for link in root.findall("link")}
    joints = {joint.get("name") for joint in root.findall("joint")}

    assert not any(name.startswith(("l_hl_", "r_hl_")) for name in links)
    assert not any(name.startswith(("l_hj_", "r_hj_")) for name in joints)
    assert {f"l_aj_{index}" for index in range(1, 8)} <= joints
    assert {f"r_aj_{index}" for index in range(1, 8)} <= joints


def test_hand_actuator_and_mimic_joint_contract() -> None:
    root = build_robot_element("both")
    for prefix in ("l_hj_", "r_hj_"):
        movable = [
            joint
            for joint in root.findall("joint")
            if joint.get("name").startswith(prefix) and joint.get("type") != "fixed"
        ]
        actuators = [joint for joint in movable if joint.find("mimic") is None]
        passive_mimics = [joint for joint in movable if joint.find("mimic") is not None]
        assert len(actuators) == 6
        assert len(passive_mimics) == 6

    joint_names = {joint.get("name") for joint in root.findall("joint")}
    for joint in root.findall("joint"):
        mimic = joint.find("mimic")
        if mimic is not None:
            assert mimic.get("joint") in joint_names


def test_all_movable_joints_have_nonzero_axes_and_valid_limits() -> None:
    root = build_robot_element("both")
    for joint in root.findall("joint"):
        if joint.get("type") == "fixed":
            continue
        axis = _as_vector(joint.find("axis").get("xyz"))
        assert math.sqrt(sum(value * value for value in axis)) > 0.0
        if joint.get("type") != "continuous":
            limit = joint.find("limit")
            assert limit is not None
            assert float(limit.get("lower")) < float(limit.get("upper"))


def test_every_resolved_mesh_exists() -> None:
    root = build_robot_element("both")
    meshes = root.findall(".//mesh")
    assert meshes
    for mesh in meshes:
        uri = mesh.get("filename")
        assert uri.startswith("file://")
        assert "/home/user/rl_ws/" not in uri
        path = Path(unquote(urlparse(uri).path))
        assert path.is_file(), f"missing mesh: {path}"


def _identity() -> tuple[tuple[float, ...], ...]:
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _multiply(a, b):
    return tuple(
        tuple(sum(a[row][k] * b[k][column] for k in range(4)) for column in range(4))
        for row in range(4)
    )


def _origin_transform(xyz, rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    result = [list(row) for row in _identity()]
    result[0][:3] = (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr)
    result[1][:3] = (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr)
    result[2][:3] = (-sp, cp * sr, cp * cr)
    result[0][3], result[1][3], result[2][3] = xyz
    return tuple(tuple(row) for row in result)


def _axis_angle(axis, angle):
    norm = math.sqrt(sum(value * value for value in axis))
    x, y, z = (value / norm for value in axis)
    c, s, d = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return (
        (c + x * x * d, x * y * d - z * s, x * z * d + y * s, 0.0),
        (y * x * d + z * s, c + y * y * d, y * z * d - x * s, 0.0),
        (z * x * d - y * s, z * y * d + x * s, c + z * z * d, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _inverse_rigid(transform):
    result = [list(row) for row in _identity()]
    for row in range(3):
        for column in range(3):
            result[row][column] = transform[column][row]
        result[row][3] = -sum(transform[column][row] * transform[column][3] for column in range(3))
    return tuple(tuple(row) for row in result)


def _fk(root: ET.Element, positions: dict[str, float]):
    by_parent: dict[str, list[ET.Element]] = {}
    child_links = set()
    for joint in root.findall("joint"):
        parent = joint.find("parent").get("link")
        child_links.add(joint.find("child").get("link"))
        by_parent.setdefault(parent, []).append(joint)
    root_link = next(
        link.get("name") for link in root.findall("link") if link.get("name") not in child_links
    )
    transforms = {root_link: _identity()}

    def visit(parent: str) -> None:
        for joint in by_parent.get(parent, []):
            origin = joint.find("origin")
            xyz = _as_vector(origin.get("xyz", "0 0 0")) if origin is not None else (0.0, 0.0, 0.0)
            rpy = _as_vector(origin.get("rpy", "0 0 0")) if origin is not None else (0.0, 0.0, 0.0)
            motion = _identity()
            if joint.get("type") in {"revolute", "continuous"}:
                value = positions.get(joint.get("name"), 0.0)
                mimic = joint.find("mimic")
                if mimic is not None:
                    value = (
                        positions.get(mimic.get("joint"), 0.0)
                        * float(mimic.get("multiplier", "1"))
                        + float(mimic.get("offset", "0"))
                    )
                motion = _axis_angle(_as_vector(joint.find("axis").get("xyz")), value)
            child = joint.find("child").get("link")
            transforms[child] = _multiply(
                transforms[parent], _multiply(_origin_transform(xyz, rpy), motion)
            )
            visit(child)

    visit(root_link)
    return transforms


def _different(a, b, tolerance=1e-9):
    return any(
        abs(a[row][column] - b[row][column]) > tolerance
        for row in range(4)
        for column in range(4)
    )


def _same(a, b, tolerance=1e-9):
    return not _different(a, b, tolerance)


@pytest.mark.parametrize(
    ("side", "arm_joint", "finger_joint", "finger_tip", "wrist"),
    (
        ("left", "l_aj_2", "l_hj_index_1", "l_hl_index_tip", "l_al_7"),
        ("right", "r_aj_2", "r_hj_index_1", "r_hl_index_tip", "r_al_7"),
    ),
)
def test_mock_fk_arm_hand_and_palm_sensor_behavior(
    side: str,
    arm_joint: str,
    finger_joint: str,
    finger_tip: str,
    wrist: str,
) -> None:
    root = build_robot_element("both")
    zero = _fk(root, {})
    arm_moved = _fk(root, {arm_joint: 0.25})
    finger_moved = _fk(root, {finger_joint: 0.25})
    sensor = PALM_CONTROL_FRAMES[side]

    # body_root -> sensor is computable and follows an upstream arm joint.
    assert all(math.isfinite(value) for row in zero[sensor] for value in row)
    assert _different(zero[sensor], arm_moved[sensor])

    # A hand actuator moves its finger, but cannot move the fixed palm frame.
    assert _different(zero[finger_tip], finger_moved[finger_tip])
    assert _same(zero[sensor], finger_moved[sensor])

    zero_sensor_from_wrist = _multiply(
        _inverse_rigid(zero[wrist]), zero[sensor]
    )
    moved_sensor_from_wrist = _multiply(
        _inverse_rigid(arm_moved[wrist]), arm_moved[sensor]
    )
    finger_sensor_from_wrist = _multiply(
        _inverse_rigid(finger_moved[wrist]), finger_moved[sensor]
    )

    # wrist -> sensor exists and is invariant to both upstream arm motion and
    # downstream finger motion.
    assert all(
        math.isfinite(value) for row in zero_sensor_from_wrist for value in row
    )
    assert _same(zero_sensor_from_wrist, moved_sensor_from_wrist)
    assert _same(zero_sensor_from_wrist, finger_sensor_from_wrist)
