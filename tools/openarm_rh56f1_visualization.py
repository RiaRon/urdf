#!/usr/bin/env python3
"""Build ROS/RViz-ready OpenArm + RH56F1 URDF variants.

The checked-in canonical bimanual URDF is the kinematic source of truth.  This
module only resolves its historical workspace-local mesh URIs and optionally
removes complete hand subtrees for visualization.  It never changes joint
origins, axes, limits, inertials, or mimic relationships.
"""

from __future__ import annotations

import argparse
import copy
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[1]
SOURCE_URDF = REPO_ROOT / "generated/rl/openarm_rh56f1_bi_rl.urdf"
LEGACY_MESH_ROOT = "file:///home/user/rl_ws/urdf/"
HAND_CONFIGURATIONS = ("arm_only", "left", "right", "both")

# Public frame contract for FK, state monitoring, and future control adapters.
# These are existing links in the canonical URDF, not newly synthesized TCPs.
PALM_CONTROL_FRAMES = {
    "left": "l_hl_palm_sensor",
    "right": "r_hl_palm_sensor",
}

_HAND_LINK_PREFIX = {"left": "l_hl_", "right": "r_hl_"}


def _enabled_sides(hand_configuration: str) -> set[str]:
    if hand_configuration not in HAND_CONFIGURATIONS:
        choices = ", ".join(HAND_CONFIGURATIONS)
        raise ValueError(
            f"unsupported hand_configuration {hand_configuration!r}; choose {choices}"
        )
    if hand_configuration == "both":
        return {"left", "right"}
    if hand_configuration == "arm_only":
        return set()
    return {hand_configuration}


def build_robot_element(
    hand_configuration: str = "both",
    *,
    source_urdf: Path = SOURCE_URDF,
    repo_root: Path = REPO_ROOT,
) -> ET.Element:
    """Return one connected URDF tree for the requested hand selection."""

    enabled_sides = _enabled_sides(hand_configuration)
    root = copy.deepcopy(ET.parse(source_urdf).getroot())

    removed_links = {
        link.get("name")
        for side, prefix in _HAND_LINK_PREFIX.items()
        if side not in enabled_sides
        for link in root.findall("link")
        if (link.get("name") or "").startswith(prefix)
    }

    for link in list(root.findall("link")):
        if link.get("name") in removed_links:
            root.remove(link)

    for joint in list(root.findall("joint")):
        parent = joint.find("parent")
        child = joint.find("child")
        if (
            parent is not None
            and parent.get("link") in removed_links
            or child is not None
            and child.get("link") in removed_links
        ):
            root.remove(joint)

    local_mesh_root = repo_root.resolve().as_uri().rstrip("/") + "/"
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename and filename.startswith(LEGACY_MESH_ROOT):
            mesh.set("filename", local_mesh_root + filename[len(LEGACY_MESH_ROOT) :])

    root.set("name", f"openarm_rh56f1_{hand_configuration}_visualization")
    return root


def build_robot_description(hand_configuration: str = "both") -> str:
    """Return the generated URDF as a string suitable for robot_description."""

    root = build_robot_element(hand_configuration)
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode", xml_declaration=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--hand-configuration",
        choices=HAND_CONFIGURATIONS,
        default="both",
        help="RH56F1 attachment selection (default: both)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="write the generated URDF to this path (stdout when omitted)",
    )
    args = parser.parse_args(argv)

    description = build_robot_description(args.hand_configuration)
    if args.output:
        args.output.write_text(description, encoding="utf-8")
    else:
        sys.stdout.write(description)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
