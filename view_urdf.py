"""Open a URDF in viser: link meshes, joint sliders, link frames, and centers of mass.

The assembled SolidWorks pose is joint value zero. Drag a slider to move that joint
about the axis stored in the URDF.

    uv run python view_urdf.py output/s3_arm/s3_arm.urdf
"""

from __future__ import annotations

import argparse
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

# Names used by the S3 arm. Other links take the palette in URDF order.
LINK_COLOR = {
    "base_link": (0.45, 0.47, 0.50, 1.0),
    "pitch_link": (0.20, 0.45, 0.85, 1.0),
    "roll_link": (0.20, 0.65, 0.40, 1.0),
    "catch_link": (0.90, 0.55, 0.15, 1.0),
    "catch_l_link": (0.85, 0.25, 0.25, 1.0),
    "catch_r_link": (0.90, 0.80, 0.20, 1.0),
}
PALETTE = (
    (0.45, 0.47, 0.50, 1.0),
    (0.20, 0.45, 0.85, 1.0),
    (0.20, 0.65, 0.40, 1.0),
    (0.95, 0.55, 0.20, 1.0),
    (0.55, 0.35, 0.75, 1.0),
    (0.15, 0.55, 0.55, 1.0),
    (0.85, 0.25, 0.25, 1.0),
    (0.90, 0.80, 0.20, 1.0),
)


def _se3_from_xyz_rpy(xyz: list[float], rpy: list[float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    rx = np.array(
        [[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]]
    )
    ry = np.array(
        [[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]]
    )
    rz = np.array(
        [[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
    )
    transform = np.eye(4)
    transform[:3, :3] = rz @ ry @ rx
    transform[:3, 3] = xyz
    return transform


def _load_urdf(path: Path):
    root = ET.parse(path).getroot()
    joints = []
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        if origin is None:
            raise ValueError(f"joint {joint.attrib.get('name')} has no origin")
        xyz = [float(v) for v in origin.attrib["xyz"].split()]
        rpy = [float(v) for v in origin.attrib["rpy"].split()]
        axis = joint.find("axis")
        limit = joint.find("limit")
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            raise ValueError(f"joint {joint.attrib.get('name')} is missing parent or child")
        joints.append(
            {
                "axis": np.array([float(v) for v in axis.attrib["xyz"].split()])
                if axis is not None
                else np.array([0.0, 0.0, 1.0]),
                "name": joint.attrib["name"],
                "type": joint.attrib["type"],
                "parent": parent.attrib["link"],
                "child": child.attrib["link"],
                "origin": _se3_from_xyz_rpy(xyz, rpy),
                "lower": float(limit.attrib["lower"]) if limit is not None else 0.0,
                "upper": float(limit.attrib["upper"]) if limit is not None else 0.0,
            }
        )
    visuals: dict[str, list[tuple[np.ndarray, trimesh.Trimesh]]] = {}
    coms: dict[str, np.ndarray] = {}
    link_names: list[str] = []
    for link in root.findall("link"):
        name = link.attrib["name"]
        link_names.append(name)
        items = []
        for visual in link.findall("visual"):
            origin = visual.find("origin")
            mesh = visual.find("geometry/mesh")
            if origin is None or mesh is None:
                continue
            xyz = [float(v) for v in origin.attrib["xyz"].split()]
            rpy = [float(v) for v in origin.attrib["rpy"].split()]
            filename = path.parent / mesh.attrib["filename"]
            loaded = trimesh.load(filename, force="mesh", process=False)
            items.append((_se3_from_xyz_rpy(xyz, rpy), loaded))
        visuals[name] = items
        inertial = link.find("inertial/origin")
        if inertial is not None:
            xyz = [float(v) for v in inertial.attrib["xyz"].split()]
            rpy = [float(v) for v in inertial.attrib["rpy"].split()]
            coms[name] = _se3_from_xyz_rpy(xyz, rpy)
    return joints, visuals, coms, link_names


def _link_color(link_name: str, link_names: list[str]) -> tuple[float, float, float, float]:
    if link_name in LINK_COLOR:
        return LINK_COLOR[link_name]
    index = link_names.index(link_name) if link_name in link_names else 0
    return PALETTE[index % len(PALETTE)]


def _wxyz(rotation: np.ndarray) -> tuple[float, float, float, float]:
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    quat = trimesh.transformations.quaternion_from_matrix(matrix)
    return float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])


def _motion(joint_type: str, value: float, axis: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    if joint_type == "revolute":
        transform = trimesh.transformations.rotation_matrix(value, axis)
    elif joint_type == "prismatic":
        transform[:3, 3] = value * axis / np.linalg.norm(axis)
    return transform


def view(urdf_path: Path, port: int = 8080) -> None:
    import viser

    joints, visuals, coms, link_names = _load_urdf(urdf_path)
    children: dict[str, list[dict]] = {}
    for joint in joints:
        children.setdefault(joint["parent"], []).append(joint)
    server = viser.ViserServer(port=port)
    server.scene.add_grid("grid", width=2.0, height=2.0, cell_size=0.1)

    def fk(q: dict[str, float]) -> dict[str, np.ndarray]:
        poses = {"base_link": np.eye(4)}
        stack = ["base_link"]
        while stack:
            parent = stack.pop()
            for joint in children.get(parent, []):
                motion = _motion(joint["type"], q.get(joint["name"], 0.0), joint["axis"])
                poses[joint["child"]] = poses[parent] @ joint["origin"] @ motion
                stack.append(joint["child"])
        return poses

    # base_link is the URDF root, so its frame stays on the grid origin.
    zero = fk({})

    handles = {}
    link_labels = {}
    com_handles = {}
    com_labels = {}
    for link_name in zero:
        pose = zero[link_name].copy()
        handles[link_name] = server.scene.add_frame(
            f"/{link_name}",
            show_axes=True,
            axes_length=0.08,
            axes_radius=0.003,
            origin_radius=0.006,
            position=tuple(pose[:3, 3]),
            wxyz=_wxyz(pose[:3, :3]),
        )
        link_labels[link_name] = server.scene.add_label(
            f"/{link_name}/label",
            link_name,
            position=(0.0, 0.0, 0.09),
        )
        com = coms.get(link_name)
        if com is not None:
            com_handles[link_name] = server.scene.add_frame(
                f"/{link_name}/com",
                show_axes=True,
                axes_length=0.035,
                axes_radius=0.002,
                origin_radius=0.008,
                origin_color=(255, 255, 255),
                position=tuple(com[:3, 3]),
                wxyz=_wxyz(com[:3, :3]),
            )
            com_labels[link_name] = server.scene.add_label(
                f"/{link_name}/com/label",
                "com",
                position=(0.02, 0.0, 0.02),
            )
        for index, (origin, mesh) in enumerate(visuals.get(link_name, [])):
            colored = mesh.copy()
            colored.apply_transform(origin)
            rgb = tuple(int(channel * 255) for channel in _link_color(link_name, link_names)[:3])
            server.scene.add_mesh_simple(
                f"/{link_name}/visual_{index}",
                vertices=np.asarray(colored.vertices, dtype=np.float32),
                faces=np.asarray(colored.faces, dtype=np.uint32),
                color=rgb,
            )

    sliders = {}
    for joint in joints:
        if joint["type"] == "fixed":
            continue
        sliders[joint["name"]] = server.gui.add_slider(
            joint["name"],
            min=joint["lower"],
            max=joint["upper"],
            step=0.01,
            initial_value=0.0,
        )
    show_links = server.gui.add_checkbox("link frames", initial_value=True)
    show_coms = server.gui.add_checkbox("CoM frames", initial_value=True)
    reset = server.gui.add_button("reset")

    def reset_pose(_event=None) -> None:
        for slider in sliders.values():
            slider.value = 0.0
        update()

    reset.on_click(reset_pose)

    def toggle_frames(_event=None) -> None:
        for link_name, handle in handles.items():
            handle.show_axes = show_links.value
            link_labels[link_name].visible = show_links.value
        for link_name, handle in com_handles.items():
            handle.visible = show_coms.value
            com_labels[link_name].visible = show_coms.value

    show_links.on_update(toggle_frames)
    show_coms.on_update(toggle_frames)

    def update(_event=None) -> None:
        q = {name: slider.value for name, slider in sliders.items()}
        poses = fk(q)
        for link_name, pose in poses.items():
            handles[link_name].position = tuple(pose[:3, 3])
            handles[link_name].wxyz = _wxyz(pose[:3, :3])

    for slider in sliders.values():
        slider.on_update(update)
    print(f"viser http://localhost:{server.get_port()}")
    print("zero pose is the assembled SolidWorks configuration; drag the joint sliders to move it")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        return


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", type=Path, help="URDF to open")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    view(args.urdf, port=args.port)


if __name__ == "__main__":
    main()
