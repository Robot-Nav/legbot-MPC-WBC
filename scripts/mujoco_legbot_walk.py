#!/usr/bin/env python3
"""Baseline-only LegBot MuJoCo trot script.

The original legbot scene is kept as the source of truth.  Before running, this
script writes a project-local MuJoCo XML with the mesh directory fixed to the
checked-in model assets and with non-robot marker arrows removed so the walking
simulation only advances the quadruped.

Important: this file does not run Cheetah-Software's MIT_Controller,
ConvexMPCLocomotion, WBC, RobotRunner, LegController, or StateEstimator stack.
It is only a direct MuJoCo baseline controller.  Use the C++ two-process path
for the project algorithm stack:

  1. ./build-legbot-check/mujoco_bridge/mujoco_legbot_sim
  2. ./build-legbot-check/user/MIT_Controller/mit_ctrl v s
"""

from __future__ import annotations

import argparse
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE_XML = REPO_ROOT / "models" / "MJCF" / "legbot" / "legbot_mpc_scene.xml"
DEFAULT_GENERATED_XML = REPO_ROOT / "models" / "MJCF" / "legbot" / "legbot_mpc_scene_project.xml"
DEFAULT_COLLISION_XML = REPO_ROOT / "models" / "MJCF" / "legbot" / "legbot_mpc_scene_collision.xml"
MESH_DIR_CANDIDATES = (
    REPO_ROOT / "models" / "legbot" / "xmls" / "meshes",
    REPO_ROOT / "models" / "legbot_description" / "meshes",
)

LEGS = ("FR", "FL", "RR", "RL")
JOINTS = tuple(f"{leg}_{joint}_joint" for leg in LEGS for joint in ("hip", "thigh", "calf"))


def _strip_non_robot_markers(root: ET.Element) -> None:
    asset = root.find("asset")
    if asset is not None:
        for child in list(asset):
            if child.tag == "mesh" and child.get("name") == "arrow_cone":
                asset.remove(child)

    worldbody = root.find("worldbody")
    if worldbody is None:
        return
    for body in list(worldbody):
        if body.get("name") in {"target_marker", "robot_heading_arrow", "desired_heading_arrow"}:
            worldbody.remove(body)


def _remove_visual_meshes(root: ET.Element) -> None:
    asset = root.find("asset")
    if asset is not None:
        for child in list(asset):
            if child.tag == "mesh":
                asset.remove(child)

    for parent in root.iter():
        for child in list(parent):
            if child.tag == "geom" and child.get("type") == "mesh":
                parent.remove(child)

    _strip_non_robot_markers(root)

    compiler = root.find("compiler")
    if compiler is not None and "meshdir" in compiler.attrib:
        del compiler.attrib["meshdir"]


def _set_sim_options(root: ET.Element) -> None:
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", "0.001")
    option.set("integrator", "RK4")
    option.set("cone", "elliptic")
    option.set("impratio", "100")

    size = root.find("size")
    if size is None:
        size = ET.SubElement(root, "size")
    size.set("njmax", "1000")
    size.set("nconmax", "300")


def _scale_pair(value: str, scale: float) -> str:
    lo, hi = (float(part) for part in value.split())
    return f"{lo * scale:.6g} {hi * scale:.6g}"


def _scale_actuator_limits(root: ET.Element, scale: float) -> None:
    if abs(scale - 1.0) < 1e-9:
        return
    for elem in root.iter():
        for attr in ("forcerange", "actuatorfrcrange"):
            if attr in elem.attrib:
                elem.set(attr, _scale_pair(elem.get(attr, "0 0"), scale))


def write_collision_xml(source_xml: Path, output_xml: Path) -> Path:
    tree = ET.parse(source_xml)
    root = tree.getroot()
    _remove_visual_meshes(root)
    _set_sim_options(root)
    output_xml.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output_xml, encoding="utf-8", xml_declaration=True)
    return output_xml


def _find_mesh_dir(mesh_dir: Path | None) -> Path:
    candidates = (mesh_dir,) if mesh_dir is not None else MESH_DIR_CANDIDATES
    for candidate in candidates:
        if candidate and (candidate / "base_link.STL").is_file():
            return candidate.resolve()
    searched = ", ".join(str(path) for path in candidates if path)
    raise FileNotFoundError(f"Could not find legbot STL meshes. Searched: {searched}")


def write_project_xml(source_xml: Path, output_xml: Path, mesh_dir: Path | None, torque_scale: float) -> Path:
    tree = ET.parse(source_xml)
    root = tree.getroot()
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.set("angle", "radian")
    compiler.set("meshdir", str(_find_mesh_dir(mesh_dir)))
    _strip_non_robot_markers(root)
    _set_sim_options(root)
    _scale_actuator_limits(root, torque_scale)
    output_xml.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output_xml, encoding="utf-8", xml_declaration=True)
    return output_xml


def import_runtime():
    try:
        import mujoco
        import numpy as np
    except ModuleNotFoundError as exc:
        print(
            "Missing MuJoCo Python runtime. Install it with: python3 -m pip install mujoco numpy",
            file=sys.stderr,
        )
        raise SystemExit(2) from exc
    return mujoco, np


def quat_to_rpy(quat):
    w, x, y, z = quat
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class VbotRaibertTrot:
    def __init__(self, model, mujoco, np, speed: float, torque_scale: float, height: float):
        if not 0.5 <= speed <= 2.0:
            raise ValueError("speed must be in [0.5, 2.0] m/s")
        self.model = model
        self.mujoco = mujoco
        self.np = np
        self.speed = speed
        self.height = height
        self.l1 = 0.1985
        self.l2 = 0.214
        self.hip_x = {"FR": 0.18453, "FL": 0.18453, "RR": -0.18453, "RL": -0.18453}
        self.hip_y = {"FR": -0.1485, "FL": 0.1485, "RR": -0.1485, "RL": 0.1485}
        self.phase_offset = {"FR": 0.0, "RL": 0.0, "FL": 0.5, "RR": 0.5}
        freejoint_id = model.joint("joint_fixed_world").id
        self.base_qpos_addr = int(model.jnt_qposadr[freejoint_id])
        self.base_qvel_addr = int(model.jnt_dofadr[freejoint_id])
        self.qpos_addr = [model.jnt_qposadr[model.joint(name).id] for name in JOINTS]
        self.qvel_addr = [model.jnt_dofadr[model.joint(name).id] for name in JOINTS]
        self.ctrl_addr = [model.actuator(name).id for name in JOINTS]
        self.foot_body_id = {leg: model.body(f"{leg}_foot").id for leg in LEGS}
        self.nominal_q = self.np.array([self.ik(leg, 0.0, self.height) for leg in LEGS]).reshape(12)
        self.mass = float(self.np.sum(model.body_mass))
        self.kp = self.np.array([36.0, 48.0, 54.0] * 4)
        self.kd = self.np.array([1.8, 2.2, 2.4] * 4)
        self.torque_limit = torque_scale * self.np.array([16.0, 16.0, 32.0] * 4)

    def period(self) -> float:
        return max(0.24, 0.46 - 0.12 * (self.speed - 0.5))

    def step_length(self) -> float:
        return min(0.49, max(0.14, self.speed * self.period() * 0.98))

    def ik(self, leg: str, x_rel: float, z_down: float):
        side = -1.0 if leg in ("FR", "RR") else 1.0
        q_abad = 0.04 * side
        x = x_rel
        z = z_down
        r2 = x * x + z * z
        c2 = (r2 - self.l1 * self.l1 - self.l2 * self.l2) / (2.0 * self.l1 * self.l2)
        c2 = max(-0.98, min(0.98, c2))
        q_knee = -math.acos(c2)
        q_thigh = math.atan2(x, z) - math.atan2(self.l2 * math.sin(q_knee), self.l1 + self.l2 * math.cos(q_knee))
        return q_abad, q_thigh, q_knee

    def phase(self, leg: str, t: float) -> float:
        return ((t / self.period()) + self.phase_offset[leg]) % 1.0

    def foot_target(self, leg: str, t: float, base_pitch: float, vx: float):
        period = self.period()
        phase = self.phase(leg, t)
        duty = 0.55
        step = self.step_length()
        vx_error = self.speed - vx
        pitch_comp = max(-0.035, min(0.035, 0.07 * base_pitch))
        speed_comp = max(-0.025, min(0.025, 0.025 * vx_error))
        center = pitch_comp + speed_comp

        if phase < duty:
            s = phase / duty
            x_rel = center + step * (s - 0.5)
            z_down = self.height
        else:
            s = (phase - duty) / (1.0 - duty)
            smooth = 0.5 - 0.5 * math.cos(math.pi * s)
            x_rel = center + step * (0.5 - smooth)
            clearance = 0.045 + 0.02 * min(1.0, self.speed / 2.0)
            z_down = self.height - clearance * math.sin(math.pi * s)
        return x_rel, z_down

    def command(self, data, t: float):
        q = self.np.array([data.qpos[i] for i in self.qpos_addr])
        dq = self.np.array([data.qvel[i] for i in self.qvel_addr])
        base_qpos = data.qpos[self.base_qpos_addr : self.base_qpos_addr + 7]
        base_qvel = data.qvel[self.base_qvel_addr : self.base_qvel_addr + 6]
        roll, pitch, _ = quat_to_rpy(base_qpos[3:7])
        vx = base_qvel[0]
        vz = base_qvel[2]

        q_des = []
        stance_legs = []
        for leg in LEGS:
            x_rel, z_down = self.foot_target(leg, t, pitch, vx)
            q_des.extend(self.ik(leg, x_rel, z_down))
            if self.phase(leg, t) < 0.55:
                stance_legs.append(leg)
        q_des = self.np.array(q_des)

        q_des[0::3] += self.np.clip(-0.10 * roll, -0.08, 0.08)
        tau = self.kp * (q_des - q) - self.kd * dq

        if stance_legs:
            height_error = self.height + 0.015 - base_qpos[2]
            total_fz = self.mass * (9.81 + 70.0 * height_error - 5.0 * vz)
            total_fz = float(self.np.clip(total_fz, 0.45 * self.mass * 9.81, 1.55 * self.mass * 9.81))
            total_fx = float(self.np.clip(65.0 * (self.speed - vx), -110.0, 110.0))
            pitch_moment = float(self.np.clip(-8.0 * pitch - 1.0 * base_qvel[4], -5.0, 5.0))
            roll_force = float(self.np.clip(-7.0 * roll - 0.8 * base_qvel[3], -7.0, 7.0))

            for leg in stance_legs:
                leg_i = LEGS.index(leg)
                jacp = self.np.zeros((3, self.model.nv))
                jacr = self.np.zeros((3, self.model.nv))
                self.mujoco.mj_jacBody(self.model, data, jacp, jacr, self.foot_body_id[leg])
                dofs = self.qvel_addr[3 * leg_i : 3 * leg_i + 3]
                hip_x = self.hip_x[leg]
                side = -1.0 if leg in ("FR", "RR") else 1.0
                f_world = self.np.array(
                    [
                        total_fx / len(stance_legs),
                        side * roll_force / len(stance_legs),
                        total_fz / len(stance_legs) + pitch_moment * hip_x,
                    ]
                )
                tau[3 * leg_i : 3 * leg_i + 3] -= jacp[:, dofs].T @ f_world

        tau = self.np.clip(tau, -self.torque_limit, self.torque_limit)
        for ctrl_i, value in zip(self.ctrl_addr, tau):
            data.ctrl[ctrl_i] = value
        return roll, pitch, vx

    def base_rpy(self, data):
        base_qpos = data.qpos[self.base_qpos_addr : self.base_qpos_addr + 7]
        return quat_to_rpy(base_qpos[3:7])

    def reset(self, data):
        data.qpos[:] = 0.0
        data.qvel[:] = 0.0
        data.qpos[self.base_qpos_addr : self.base_qpos_addr + 3] = (0.0, 0.0, self.height + 0.025)
        data.qpos[self.base_qpos_addr + 3 : self.base_qpos_addr + 7] = (1.0, 0.0, 0.0, 0.0)
        for addr, value in zip(self.qpos_addr, self.nominal_q):
            data.qpos[addr] = value

    def base_x(self, data) -> float:
        return float(data.qpos[self.base_qpos_addr])

    def base_height(self, data) -> float:
        return float(data.qpos[self.base_qpos_addr + 2])


def run_once(args, xml_path: Path):
    mujoco, np = import_runtime()
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    physics_hz = 1.0 / model.opt.timestep
    control_decimation = max(1, round(physics_hz / args.control_hz))
    actual_control_hz = physics_hz / control_decimation
    controller = VbotRaibertTrot(model, mujoco, np, args.speed, args.torque_scale, args.height)
    controller.reset(data)
    mujoco.mj_forward(model, data)

    viewer = None
    if args.viewer:
        try:
            from mujoco import viewer as mujoco_viewer

            viewer = mujoco_viewer.launch_passive(model, data)
        except Exception as exc:
            print(f"Viewer unavailable, continuing headless: {exc}", file=sys.stderr)

    start_x = controller.base_x(data)
    max_abs_roll = 0.0
    max_abs_pitch = 0.0
    min_height = controller.base_height(data)
    t0 = time.time()
    step_count = 0

    while data.time < args.duration:
        if step_count % control_decimation == 0:
            controller.command(data, data.time)
        mujoco.mj_step(model, data)
        roll, pitch, _ = controller.base_rpy(data)
        max_abs_roll = max(max_abs_roll, abs(roll))
        max_abs_pitch = max(max_abs_pitch, abs(pitch))
        min_height = min(min_height, controller.base_height(data))
        step_count += 1
        if viewer is not None and viewer.is_running() and step_count % args.render_every == 0:
            viewer.sync()
            if args.realtime:
                sleep_time = data.time - (time.time() - t0)
                if sleep_time > 0.0:
                    time.sleep(sleep_time)

    distance = controller.base_x(data) - start_x
    avg_speed = distance / max(args.duration, 1e-6)
    healthy = 0.45 <= avg_speed <= 2.15 and min_height > 0.20 and max_abs_pitch < 0.60 and max_abs_roll < 0.55
    return {
        "cmd_speed": args.speed,
        "avg_speed": avg_speed,
        "distance": distance,
        "min_height": min_height,
        "max_roll": max_abs_roll,
        "max_pitch": max_abs_pitch,
        "physics_hz": physics_hz,
        "control_hz": actual_control_hz,
        "render_every": args.render_every,
        "height": args.height,
        "healthy": healthy,
    }


def print_result(result) -> None:
    status = "OK" if result["healthy"] else "FAIL"
    print(
        f"{status} cmd={result['cmd_speed']:.2f}m/s avg={result['avg_speed']:.2f}m/s "
        f"dist={result['distance']:.2f}m min_z={result['min_height']:.3f}m "
        f"max_roll={math.degrees(result['max_roll']):.1f}deg "
        f"max_pitch={math.degrees(result['max_pitch']):.1f}deg"
    )


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-xml", type=Path, default=DEFAULT_SOURCE_XML)
    parser.add_argument("--xml", type=Path, default=DEFAULT_GENERATED_XML)
    parser.add_argument("--mesh-dir", type=Path, default=None, help="directory containing base_link.STL and leg STL meshes")
    parser.add_argument("--torque-scale", type=float, default=1.8, help="scale motor force limits in the generated MuJoCo XML")
    parser.add_argument("--height", type=float, default=0.2775, help="nominal base/COM height target in meters")
    parser.add_argument("--control-hz", type=float, default=1000.0, choices=(500.0, 1000.0), help="controller update frequency")
    parser.add_argument("--render-every", type=int, default=16, help="viewer sync period in physics steps")
    parser.add_argument("--speed", type=float, default=1.0, help="forward speed command in m/s, valid range [0.5, 2.0]")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--viewer", action="store_true", help="open MuJoCo passive viewer")
    parser.add_argument("--realtime", action="store_true", help="pace viewer simulation in wall time")
    parser.add_argument("--prepare-xml", action="store_true", help="only generate the MuJoCo XML used for simulation")
    parser.add_argument("--collision-only", action="store_true", help="remove visual meshes and run on collision geoms only")
    parser.add_argument("--scan", action="store_true", help="run 0.5, 1.0, 1.5 and 2.0 m/s headless")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    xml_path = (
        write_collision_xml(args.source_xml, args.xml if args.xml != DEFAULT_GENERATED_XML else DEFAULT_COLLISION_XML)
        if args.collision_only
        else write_project_xml(args.source_xml, args.xml, args.mesh_dir, args.torque_scale)
    )
    print(f"Generated MuJoCo XML: {xml_path}")
    print(
        f"Command: speed={args.speed:.2f}m/s height={args.height:.4f}m "
        f"control_hz={args.control_hz:.0f} render_every={args.render_every} "
        f"torque_scale={args.torque_scale:.2f}"
    )
    if args.prepare_xml:
        return 0

    if args.scan:
        ok = True
        for speed in (0.5, 1.0, 1.5, 2.0):
            scan_args = argparse.Namespace(**vars(args))
            scan_args.speed = speed
            scan_args.viewer = False
            result = run_once(scan_args, xml_path)
            print_result(result)
            ok = ok and result["healthy"]
        return 0 if ok else 1

    result = run_once(args, xml_path)
    print_result(result)
    return 0 if result["healthy"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
