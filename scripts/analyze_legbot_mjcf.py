#!/usr/bin/env python3
"""Analyze and verify the LegBot MJCF model with MuJoCo.

This script keeps the source MJCF untouched.  It first reports why the source
scene may fail to load, then writes a temporary project-local scene with the
mesh path fixed and non-robot marker bodies removed, loads it through MuJoCo,
and prints the joint/actuator mapping needed by Cheetah-Software.
"""

from __future__ import annotations

import argparse
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SOURCE_XML = REPO_ROOT / "models" / "MJCF" / "legbot" / "legbot_mpc_scene.xml"
MESH_DIR_CANDIDATES = (
    REPO_ROOT / "models" / "legbot" / "xmls" / "meshes",
    REPO_ROOT / "models" / "legbot_description" / "meshes",
)
MARKER_BODIES = {"target_marker", "robot_heading_arrow", "desired_heading_arrow"}
LEG_TO_CHEETAH = {"FR": 0, "FL": 1, "RR": 2, "RL": 3}
JOINT_TO_CHEETAH = {"hip": 0, "thigh": 1, "calf": 2}


def find_mesh_dir(mesh_dir: Path | None) -> Path:
    candidates = (mesh_dir,) if mesh_dir else MESH_DIR_CANDIDATES
    for candidate in candidates:
        if candidate and (candidate / "base_link.STL").is_file():
            return candidate.resolve()
    searched = ", ".join(str(path) for path in candidates if path)
    raise FileNotFoundError(f"Could not find LegBot STL meshes. Searched: {searched}")


def make_fixed_xml(source_xml: Path, output_xml: Path, mesh_dir: Path) -> Path:
    tree = ET.parse(source_xml)
    root = tree.getroot()

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.set("angle", "radian")
    compiler.set("meshdir", str(mesh_dir))

    asset = root.find("asset")
    if asset is not None:
        for child in list(asset):
            if child.tag == "mesh" and child.get("name") == "arrow_cone":
                asset.remove(child)

    worldbody = root.find("worldbody")
    if worldbody is not None:
        for body in list(worldbody):
            if body.get("name") in MARKER_BODIES:
                worldbody.remove(body)

    ET.indent(tree, space="  ")
    tree.write(output_xml, encoding="utf-8", xml_declaration=True)
    return output_xml


def mujoco_name(model, obj_type, index: int) -> str:
    import mujoco

    return mujoco.mj_id2name(model, obj_type, index) or ""


def parse_legbot_joint_name(name: str) -> tuple[int, int]:
    leg, joint, suffix = name.split("_", 2)
    if suffix != "joint":
        raise ValueError(f"Unexpected joint name: {name}")
    return LEG_TO_CHEETAH[leg], JOINT_TO_CHEETAH[joint]


def print_model_report(model) -> None:
    import mujoco

    print(f"Loaded model: nq={model.nq}, nv={model.nv}, nu={model.nu}, nbody={model.nbody}, ngeom={model.ngeom}")

    print("\nBodies:")
    for body_id in range(model.nbody):
        name = mujoco_name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        if name:
            print(f"  {body_id:2d}: {name}")

    print("\nFoot/contact geoms:")
    for geom_id in range(model.ngeom):
        name = mujoco_name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
        if name in LEG_TO_CHEETAH or "foot" in name.lower() or "contact" in name.lower():
            body = mujoco_name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[geom_id]))
            print(f"  geom {geom_id:2d}: {name} body={body}")

    print("\nTable 1: MuJoCo joint -> Cheetah mapping")
    print("| MuJoCo joint name | MuJoCo qpos index | MuJoCo qvel index | Cheetah leg index | Cheetah joint index |")
    print("|---|---:|---:|---:|---:|")
    for joint_id in range(model.njnt):
        name = mujoco_name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if name.startswith(("FR_", "FL_", "RR_", "RL_")):
            leg_id, joint_idx = parse_legbot_joint_name(name)
            print(f"| `{name}` | {model.jnt_qposadr[joint_id]} | {model.jnt_dofadr[joint_id]} | {leg_id} | {joint_idx} |")

    print("\nTable 2: MuJoCo actuator -> LegController mapping")
    print("| MuJoCo actuator name | MuJoCo ctrl index | Cheetah leg index | Cheetah joint index | LegController command index |")
    print("|---|---:|---:|---:|---|")
    for actuator_id in range(model.nu):
        name = mujoco_name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
        leg_id, joint_idx = parse_legbot_joint_name(name)
        print(f"| `{name}` | {actuator_id} | {leg_id} | {joint_idx} | `commands[{leg_id}].*({joint_idx})` / flat {leg_id * 3 + joint_idx} |")


def try_load(label: str, xml_path: Path):
    import mujoco

    print(f"\nLoading {label}: {xml_path}")
    try:
        model = mujoco.MjModel.from_xml_path(str(xml_path))
    except Exception as exc:
        print(f"  FAILED: {exc}")
        return None
    print("  OK")
    return model


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--xml", type=Path, default=SOURCE_XML)
    parser.add_argument("--mesh-dir", type=Path, default=None)
    args = parser.parse_args()

    import mujoco  # noqa: F401

    source_xml = args.xml.resolve()
    mesh_dir = find_mesh_dir(args.mesh_dir)
    print(f"Source XML: {source_xml}")
    print(f"Selected mesh dir: {mesh_dir}")

    try_load("source XML", source_xml)

    with tempfile.TemporaryDirectory(prefix="legbot_mjcf_") as temp_dir:
        fixed_xml = Path(temp_dir) / "legbot_mpc_scene_fixed.xml"
        make_fixed_xml(source_xml, fixed_xml, mesh_dir)
        model = try_load("fixed XML", fixed_xml)
        if model is None:
            return 1
        print_model_report(model)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
