"""Build a URDF for the S3 arm from the SolidWorks assembly.

SolidWorks is not required. The assembly stores component transforms and the
URDF coordinate systems (Origin_global, Origin_pitch, Origin_roll,
Origin_catch, Origin_catch_L, Origin_catch_R). Part geometry is the tessellation
saved in each .SLDPRT (decoded with cadmpeg), placed on those frames. Joint axes
come from the motor shafts and gripper racks.

Joint limits are placeholders. Mass is the assembly mass from SolidWorks,
spread uniformly by mesh volume. Parts that cannot move relative to each
other are written as one mesh per link, and the fixed catch joint is baked
into the roll link.
"""

from __future__ import annotations

import argparse
import math
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET
import zlib
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import trimesh

_SRC = Path(__file__).resolve().parent / "src"
sys.path.insert(0, str(_SRC))
from conversion.mesh_part import cadmpeg_binary, mesh_from_display_list, mesh_from_step  # noqa: E402
from conversion.sw_math import cs13_to_se3, part_axis, sw16_to_se3  # noqa: E402

NS = "{http://www.solidworks.com/sw2003/schema}"
ASSEMBLY_MASS_KG = 3.8939075156825225

# Child link, parent link, joint type, frame name. S3 frames are SolidWorks
# coordinate systems. Other models fill this in from the drive shafts.
LINKS: tuple[tuple[str, str | None, str | None, str], ...] = (
    ("base_link", None, None, "Origin_global"),
    ("pitch_link", "base_link", "revolute", "Origin_pitch"),
    ("roll_link", "pitch_link", "revolute", "Origin_roll"),
    ("catch_link", "roll_link", "fixed", "Origin_catch"),
    ("catch_l_link", "catch_link", "prismatic", "Origin_catch_L"),
    ("catch_r_link", "catch_link", "prismatic", "Origin_catch_R"),
)
JOINT_LIMITS = {
    "revolute": (-math.pi, math.pi),
    "prismatic": (-0.02, 0.02),
}
LINK_COLOR = {
    "base_link": (0.45, 0.47, 0.50, 1.0),
    "pitch_link": (0.20, 0.45, 0.85, 1.0),
    "roll_link": (0.20, 0.65, 0.40, 1.0),
    "catch_link": (0.90, 0.55, 0.15, 1.0),
    "catch_l_link": (0.85, 0.25, 0.25, 1.0),
    "catch_r_link": (0.90, 0.80, 0.20, 1.0),
}
# Parts this far from a jaw centerline stay on the gripper body.
JAW_SPLIT_M = 0.015
# The Origin_* frames do not put the joint axis on z. Axes come from the drive
# hardware instead: a motor shaft runs along its part's local y, a rack along local x.
SHAFT_PART = "电机轴"
RACK_PART = "齿条"
# The active link table. S3 keeps LINKS; S5 and S7 replace it for one build.
_ACTIVE_LINKS = LINKS


@dataclass
class Instance:
    name: str
    source: Path
    world: np.ndarray
    under_gripper: bool
    gripper_world: np.ndarray | None
    # Name of the top-level component this part belongs to. Assemblies that
    # have no Origin_* frames are grouped by this instead.
    group: str = ""


@dataclass
class Visual:
    link: str
    mesh_file: str
    origin: np.ndarray
    mesh: trimesh.Trimesh


def _default_assembly() -> Path:
    return Path(__file__).resolve().parents[1] / "s3机械臂_1" / "s3机械臂.SLDASM"


def rpy_from_matrix(rotation: np.ndarray) -> tuple[float, float, float]:
    """URDF roll-pitch-yaw, fixed-axis XYZ."""
    sy = math.hypot(rotation[0, 0], rotation[1, 0])
    if sy > 1e-8:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def _nibble_swap(data: bytes) -> bytes:
    return bytes(((byte & 0x0F) << 4 | (byte & 0xF0) >> 4) for byte in data)


def _section_name(preamble: bytes) -> str:
    return _nibble_swap(preamble).split(b"\x00", 1)[0].decode("utf-8", "replace")


def iter_sld_blocks(data: bytes):
    marker = b"\x14\x00\x06\x00\x08\x00"
    pos = 0
    while True:
        start = data.find(marker, pos)
        if start < 0 or start + 26 > len(data):
            return
        crc = int.from_bytes(data[start + 10 : start + 14], "little")
        comp_sz = int.from_bytes(data[start + 14 : start + 18], "little")
        uncomp_sz = int.from_bytes(data[start + 18 : start + 22], "little")
        pre_sz = int.from_bytes(data[start + 22 : start + 26], "little")
        if uncomp_sz and crc == 2 * uncomp_sz and comp_sz == uncomp_sz // 2 and 0 < pre_sz < 500:
            pos = start + 26 + pre_sz
            continue
        if (
            pre_sz > 10_000_000
            or comp_sz > 80_000_000
            or start + 26 + pre_sz + comp_sz > len(data)
        ):
            pos = start + 6
            continue
        name = _section_name(data[start + 26 : start + 26 + pre_sz])
        payload = data[start + 26 + pre_sz : start + 26 + pre_sz + comp_sz]
        try:
            decoded = zlib.decompressobj(-15).decompress(payload)
            ok = len(decoded) == uncomp_sz
        except zlib.error:
            ok = False
            decoded = b""
        if ok:
            yield name, decoded
            pos = start + 26 + pre_sz + comp_sz
        else:
            pos = start + 6


def _msgpack_unpack(buf: bytes, index: int):
    code = buf[index]
    if code <= 0x7F:
        return code, index + 1
    if 0xE0 <= code <= 0xFF:
        return code - 256, index + 1
    if 0xA0 <= code <= 0xBF:
        length = code - 0xA0
        return buf[index + 1 : index + 1 + length].decode("latin-1"), index + 1 + length
    if 0x90 <= code <= 0x9F:
        count = code - 0x90
        index += 1
        values = []
        for _ in range(count):
            value, index = _msgpack_unpack(buf, index)
            values.append(value)
        return values, index
    if 0x80 <= code <= 0x8F:
        count = code - 0x80
        index += 1
        items = []
        for _ in range(count):
            key, index = _msgpack_unpack(buf, index)
            value, index = _msgpack_unpack(buf, index)
            items.append((key, value))
        return items, index
    if code == 0xC0:
        return None, index + 1
    if code == 0xC2:
        return False, index + 1
    if code == 0xC3:
        return True, index + 1
    if code == 0xCC:
        return buf[index + 1], index + 2
    if code == 0xCD:
        return int.from_bytes(buf[index + 1 : index + 3], "big"), index + 3
    if code == 0xCE:
        return int.from_bytes(buf[index + 1 : index + 5], "big"), index + 5
    if code == 0xCB:
        return struct.unpack(">d", buf[index + 1 : index + 9])[0], index + 9
    if code == 0xCA:
        return struct.unpack(">f", buf[index + 1 : index + 5])[0], index + 5
    if code == 0xD9:
        length = buf[index + 1]
        return buf[index + 2 : index + 2 + length].decode("latin-1"), index + 2 + length
    if code == 0xC4:
        length = buf[index + 1]
        return buf[index + 2 : index + 2 + length], index + 2 + length
    raise ValueError(f"unsupported msgpack byte {code:#x} at {index}")


def coordinate_systems(assembly: Path) -> dict[str, np.ndarray]:
    blob = None
    for name, payload in iter_sld_blocks(assembly.read_bytes()):
        if name == "Contents/3DExperienceExchange2":
            blob = payload
            break
    if blob is None:
        raise RuntimeError(f"{assembly} has no coordinate-system database")
    signature = b"\x8d\xa5class\xacmoCoordSys_c"
    frames: dict[str, np.ndarray] = {}
    start = 0
    while True:
        found = blob.find(signature, start)
        if found < 0:
            break
        items, _ = _msgpack_unpack(blob, found)
        fields: dict[str, list] = {}
        for key, value in items:
            fields.setdefault(key, []).append(value)
        name = fields.get("name", [None])[-1]
        transform_values = fields.get("transform:", [])
        elems = None
        for value in transform_values:
            if isinstance(value, list):
                nested: dict[str, list] = {}
                for key, item in value:
                    nested.setdefault(key, []).append(item)
                candidate = nested.get("elems", [None])[-1]
                if isinstance(candidate, list) and len(candidate) >= 13:
                    elems = candidate
        if isinstance(name, str) and name.startswith("Origin_") and elems is not None:
            frames[name] = cs13_to_se3([float(v) for v in elems[:13]])
        start = found + 1
    missing = [spec[3] for spec in LINKS if spec[3] not in frames]
    if missing:
        raise RuntimeError(f"assembly is missing coordinate systems: {missing}")
    return frames


def _active_model_id(root: ET.Element) -> str:
    for config in root.iter(NS + "swConfiguration"):
        if config.attrib.get("swMostRecentConfiguration") == "YES":
            return config.attrib["swModelRef"]
    raise RuntimeError("assembly has no active configuration")


def _resolve_file(folders: list[Path], stored_path: str) -> Path | None:
    base = stored_path.replace("\\", "/").split("/")[-1]
    wanted = base.casefold()
    # A configured part is stored as "NAME(configuration).SLDPRT" while the
    # file on disk is often just "NAME.SLDPRT".
    stem = Path(base).stem
    prefixes = [stem.casefold()]
    if "(" in stem:
        prefixes.append(stem.split("(", 1)[0].strip().casefold())
    found: list[Path] = []
    for folder in folders:
        exact = folder / base
        if exact.is_file():
            return exact
        if not folder.is_dir():
            continue
        for candidate in folder.iterdir():
            if not candidate.is_file():
                continue
            if candidate.name.casefold() == wanted:
                return candidate
            if candidate.stem.casefold() in prefixes:
                found.append(candidate)
    if len(found) == 1:
        return found[0]
    return None


def extract_virtual_parts(folder: Path, cache: Path) -> int:
    """Pull in-context parts embedded in the assemblies out to real files."""
    cache.mkdir(parents=True, exist_ok=True)
    written = 0
    for assembly in folder.glob("*.SLDASM"):
        for name, payload in iter_sld_blocks(assembly.read_bytes()):
            if not (name.startswith("ImportedComp/") or name.startswith("VirtualComp/")):
                continue
            base = Path(name.split("/", 1)[1]).name
            if not base.lower().endswith(".sldprt"):
                continue
            destination = cache / base
            if destination.is_file() and destination.stat().st_size == len(payload):
                continue
            destination.write_bytes(payload)
            written += 1
    return written


def collect_instances(
    assembly: Path, extra_dirs: list[Path]
) -> tuple[str, list[Instance], list[str], list[tuple]]:
    root = ET.fromstring(
        next(
            payload
            for name, payload in iter_sld_blocks(assembly.read_bytes())
            if name == "swXmlContents/COMPINSTANCETREE"
        )
    )
    models = {el.attrib["id"]: el for el in root.iter(NS + "swModel")}
    files = {el.attrib["id"]: el.attrib.get("swPath", "") for el in root.iter(NS + "swFile")}
    active_id = _active_model_id(root)
    config_name = models[active_id].attrib.get("swConfigurationName", "")
    folders = [assembly.parent, *extra_dirs]
    warnings: list[str] = []
    instances: list[Instance] = []
    # (step file, world, under_gripper, gripper_world, descendant instance indexes)
    groups: list[tuple] = []

    def walk(
        model_id: str,
        parent_world: np.ndarray,
        under_gripper: bool,
        gripper_world: np.ndarray | None,
        group: str,
    ) -> None:
        model = models[model_id]
        for ref in model.findall(NS + "swReference"):
            if ref.attrib.get("swSuppressed") == "YES" or ref.attrib.get("swHidden") == "YES":
                continue
            local = sw16_to_se3([float(v) for v in ref.attrib["swTransform"].split()])
            world = parent_world @ local
            child = models[ref.attrib["swModelRef"]]
            child_name = child.attrib.get("swName", ref.attrib.get("swName", ""))
            this_group = group or child_name
            # Only the gripper assembly itself defines the jaw frame. Every
            # descendant keeps that frame; a part whose group name contains
            # 夹爪V2 must not replace it.
            is_gripper = "夹爪V2" in child_name
            next_gripper = world if is_gripper else gripper_world
            next_under = under_gripper or is_gripper
            if child.findall(NS + "swReference"):
                before = len(instances)
                walk(ref.attrib["swModelRef"], world, next_under, next_gripper, this_group)
                standin = _step_standin(folders, files.get(child.attrib.get("swFileRef", ""), ""))
                if standin is not None:
                    groups.append((standin, world, next_under, next_gripper, child_name, this_group, list(range(before, len(instances)))))
                continue
            file_id = child.attrib.get("swFileRef")
            stored = files.get(file_id, "")
            source = _resolve_file(folders, stored) if stored else None
            if source is None:
                warnings.append(f"missing file for {child_name} ({stored})")
                continue
            instances.append(
                Instance(
                    name=child_name,
                    source=source,
                    world=world,
                    under_gripper=next_under,
                    gripper_world=next_gripper,
                    group=this_group,
                )
            )

    walk(active_id, np.eye(4), False, None, "")
    return config_name, instances, warnings, groups


def _step_standin(folders: list[Path], stored_path: str) -> Path | None:
    """Use a sibling STEP when an assembly file is only a container.

    A second instance is often saved as ``NAME.STEP-1.SLDASM``. The geometry
    is still ``NAME.STEP``.
    """
    base = stored_path.replace("\\", "/").split("/")[-1]
    stem = Path(base).stem
    lowered = stem.lower()
    for suffix in (".step", ".stp"):
        found = lowered.rfind(suffix)
        if found > 0:
            return _resolve_file(folders, stem[: found + len(suffix)])
    return None


def _slug(name: str) -> str:
    chars = []
    for char in name:
        chars.append(char if char.isalnum() or char in "._-" else "_")
    slug = "".join(chars).strip("_")
    return slug[:80] or "part"


def _step_path(source: Path, step_dir: Path) -> Path:
    if source.suffix.lower() in {".step", ".stp"}:
        return source
    return step_dir / f"{_slug(source.stem)}.step"


def convert_steps(sources: list[Path], step_dir: Path) -> dict[Path, str]:
    step_dir.mkdir(parents=True, exist_ok=True)
    cadmpeg = cadmpeg_binary()
    failures: dict[Path, str] = {}

    def one(source: Path) -> tuple[Path, str | None]:
        destination = _step_path(source, step_dir)
        if source.suffix.lower() in {".step", ".stp"}:
            return source, None
        if destination.is_file() and destination.stat().st_size > 0:
            return source, None
        result = subprocess.run(
            [str(cadmpeg), "convert", "--allow-invalid", str(source), "-o", str(destination)],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0 or not destination.is_file():
            tail = (result.stderr or result.stdout or "cadmpeg failed").strip().splitlines()
            return source, tail[-1] if tail else "cadmpeg failed"
        return source, None

    with ThreadPoolExecutor(max_workers=4) as pool:
        futures = [pool.submit(one, source) for source in sources]
        for future in as_completed(futures):
            source, error = future.result()
            print(f"  step {source.name}" + (f" FAILED: {error}" if error else ""))
            if error:
                failures[source] = error
    return failures


def joint_axes_world(
    instances: list[Instance], frames: dict[str, np.ndarray]
) -> dict[str, np.ndarray]:
    """Assembly-space axis of every moving joint, keyed by child link."""
    shafts = [instance.world for instance in instances if SHAFT_PART in instance.name]
    racks = [instance.world for instance in instances if RACK_PART in instance.name]
    axes: dict[str, np.ndarray] = {}
    for link_name, _parent, joint_type, frame_name in LINKS:
        if joint_type != "revolute":
            continue
        if not shafts:
            raise RuntimeError(f"no motor shaft found for joint {link_name}")
        origin = frames[frame_name][:3, 3]
        shaft = min(shafts, key=lambda world: float(np.linalg.norm(world[:3, 3] - origin)))
        axes[link_name] = shaft[:3, 1] / np.linalg.norm(shaft[:3, 1])
    if not racks:
        raise RuntimeError("no gripper rack found for the jaw joints")
    jaw = racks[0][:3, 0] / np.linalg.norm(racks[0][:3, 0])
    # Positive motion opens both jaws.
    axes["catch_l_link"] = jaw
    axes["catch_r_link"] = -jaw
    return axes


def assign_links(
    instances: list[Instance],
    frames: dict[str, np.ndarray],
    axes: dict[str, np.ndarray],
    centroids: dict[Path, np.ndarray],
) -> dict[int, str]:
    """Pick each part's link.

    Body links use the mesh centroid, since part origins can sit far from the
    geometry. Jaw parts use the part origin: racks and jaw bases reach across the
    gripper, so their centroids sit near its center even though they ride on a jaw.
    """
    body_frames = ("Origin_global", "Origin_pitch", "Origin_roll", "Origin_catch")
    frame_of = {spec[3]: spec[0] for spec in LINKS}
    body_points = {name: frames[name][:3, 3] for name in body_frames}
    catch_l_axis_world = axes["catch_l_link"]
    assigned: dict[int, str] = {}
    for index, instance in enumerate(instances):
        point = instance.world[:3, 3]
        if instance.under_gripper and instance.gripper_world is not None:
            local = np.linalg.inv(instance.gripper_world) @ np.append(point, 1.0)
            jaw_axis = instance.gripper_world[:3, :3].T @ catch_l_axis_world
            jaw_axis = jaw_axis / np.linalg.norm(jaw_axis)
            along = float(np.dot(local[:3], jaw_axis))
            if abs(along) > JAW_SPLIT_M:
                assigned[index] = "catch_l_link" if along > 0 else "catch_r_link"
                continue
        centroid = centroids.get(instance.source.resolve(), np.zeros(3))
        point = (instance.world @ np.append(centroid, 1.0))[:3]
        nearest = min(body_frames, key=lambda name: float(np.linalg.norm(point - body_points[name])))
        assigned[index] = frame_of[nearest]
    return assigned


# A visible component whose solid sits this far from the rest of the assembly
# is a leftover placement (one countersunk screw in the revised S3 sits at
# z = 7.7 m). Its bounding box is enough volume to drag the link CoM out of
# the body. The buoyancy shell's part origin is far away too, but its solid
# is on the arm, so the mesh centroid keeps it.
STRAY_SOLID_M = 1.0


def _drop_stray_instances(
    instances: list[Instance], meshes: dict[Path, trimesh.Trimesh]
) -> list[Instance]:
    points: list[np.ndarray | None] = []
    for instance in instances:
        mesh = meshes.get(instance.source.resolve())
        if mesh is None or len(mesh.vertices) == 0:
            points.append(None)
            continue
        local = np.append(np.asarray(mesh.centroid, dtype=float), 1.0)
        points.append((instance.world @ local)[:3])
    known = [point for point in points if point is not None]
    if len(known) < 3:
        return instances
    median = np.median(np.asarray(known), axis=0)
    kept: list[Instance] = []
    for instance, point in zip(instances, points):
        if point is None:
            kept.append(instance)
            continue
        distance = float(np.linalg.norm(point - median))
        if distance > STRAY_SOLID_M:
            print(f"  skip {instance.name}: solid is {distance:.2f} m from the arm")
            continue
        kept.append(instance)
    return kept


def _solid_mesh(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    if mesh.is_watertight and abs(float(mesh.volume)) > 0:
        return mesh
    extents = np.maximum(mesh.bounds[1] - mesh.bounds[0], 1e-6)
    box = trimesh.creation.box(extents=extents)
    box.apply_translation(mesh.centroid)
    return box


def build(
    assembly: Path,
    output: Path,
    extra_dirs: list[Path] | None = None,
    kinematics=None,
    robot_name: str = "s3_arm",
    mass_kg: float | None = ASSEMBLY_MASS_KG,
) -> None:
    global _ACTIVE_LINKS
    print(f"assembly {assembly}")
    virtual_dir = output / "virtual"
    extracted = extract_virtual_parts(assembly.parent, virtual_dir)
    print(f"embedded parts extracted: {extracted}")
    config, instances, warnings, groups = collect_instances(assembly, [virtual_dir, *(extra_dirs or [])])
    print(f"configuration {config}: {len(instances)} visible parts")
    for warning in warnings:
        print(f"  skip {warning}")
    if not instances:
        raise RuntimeError("no visible parts were resolved")

    step_dir = output / "step"
    mesh_dir = output / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    unique_sources = list({instance.source.resolve(): instance.source for instance in instances}.values())
    print(f"converting {len(unique_sources)} parts to STEP")
    failures = convert_steps(unique_sources, step_dir)

    meshes: dict[Path, trimesh.Trimesh] = {}
    mesh_names: dict[Path, str] = {}
    for source in unique_sources:
        step = _step_path(source, step_dir)
        print(f"  mesh {source.name}")
        mesh = None
        # cadmpeg's STEP export can emit untrimmed surfaces (large shards), so a
        # converted STEP is only used when the part has no saved tessellation.
        if source.suffix.lower() not in {".step", ".stp"}:
            try:
                mesh = mesh_from_display_list(source, output / "cadir")
            except Exception as exc:
                print(f"    display list failed: {exc}")
        if mesh is None and source not in failures and step.is_file():
            print("    using STEP tessellation")
            try:
                mesh = mesh_from_step(step)
            except Exception as exc:  # cad kernel failures should not abort the arm
                print(f"    step mesh failed: {exc}")
        if mesh is None:
            failures[source] = failures.get(source, "empty mesh")
            print(f"    skipped: {failures[source]}")
            continue
        stem = _slug(source.stem)
        while stem in mesh_names.values():
            stem += "_b"
        meshes[source.resolve()] = mesh
        mesh_names[source.resolve()] = stem

    for standin, world, under_gripper, gripper_world, name, group_name, members in groups:
        if any(instances[index].source.resolve() in meshes for index in members):
            continue
        if standin.resolve() not in meshes:
            print(f"  stand-in mesh {standin.name}")
            try:
                standin_mesh = mesh_from_step(standin)
            except Exception as exc:
                print(f"    failed: {exc}")
                standin_mesh = None
            if standin_mesh is None:
                continue
            stem = _slug(standin.stem)
            while stem in mesh_names.values():
                stem += "_b"
            meshes[standin.resolve()] = standin_mesh
            mesh_names[standin.resolve()] = stem
        instances.append(
            Instance(
                name=name,
                source=standin,
                world=world,
                under_gripper=under_gripper,
                gripper_world=gripper_world,
                group=group_name,
            )
        )

    instances = _drop_stray_instances(instances, meshes)
    centroids = {key: np.asarray(mesh.centroid) for key, mesh in meshes.items()}
    mass_note = ""
    if kinematics is None:
        frames = coordinate_systems(assembly)
        _ACTIVE_LINKS = LINKS
        axes = joint_axes_world(instances, frames)
        assignment = assign_links(instances, frames, axes, centroids)
    else:
        frames, axes, assignment, _ACTIVE_LINKS, mass_note = kinematics(instances, centroids, meshes)
    visuals: list[Visual] = []
    for index, instance in enumerate(instances):
        mesh = meshes.get(instance.source.resolve())
        if mesh is None:
            continue
        link = assignment[index]
        origin = np.linalg.inv(frames[next(spec[3] for spec in _ACTIVE_LINKS if spec[0] == link)]) @ instance.world
        visuals.append(Visual(link, mesh_names[instance.source.resolve()], origin, mesh.copy()))

    counts: dict[str, int] = {}
    for visual in visuals:
        counts[visual.link] = counts.get(visual.link, 0) + 1
    print("link visuals:", ", ".join(f"{name}={counts.get(name, 0)}" for name, *_ in _ACTIVE_LINKS))

    volumes = []
    for visual in visuals:
        placed = visual.mesh.copy()
        placed.apply_transform(visual.origin)
        volumes.append(_solid_mesh(placed).volume)
    total_volume = float(sum(abs(v) for v in volumes)) or 1.0
    if mass_kg is None:
        density = 2700.0
    else:
        density = mass_kg / total_volume

    visuals, solids = merge_rigid_meshes(visuals, frames, mesh_dir)
    print("merged meshes:", ", ".join(visual.link for visual in visuals))
    urdf = _write_urdf(
        output / f"{robot_name}.urdf",
        config,
        frames,
        axes,
        visuals,
        density,
        solids,
        robot_name=robot_name,
        mass_note=mass_note,
    )
    print(f"wrote {urdf}")


def _feature_axis_column(mesh: trimesh.Trimesh) -> int:
    """Local axis a revolute joint follows.

    A worm wheel is a disk: the joint is its thin axis. A shaft is long: the
    joint is its long axis. Column 1 is not that axis on the S5 parts.
    """
    extent = np.asarray(mesh.extents, dtype=float)
    order = np.argsort(extent)
    if extent[order[0]] < 0.6 * extent[order[1]]:
        return int(order[0])
    return int(order[2])


def _chain_link(point: np.ndarray, stations: list[np.ndarray], owners: list[str]) -> str:
    """Link whose segment of the arm polyline is closest to ``point``."""
    best_distance = float("inf")
    best_link = owners[0]
    best_rank = -1
    for index in range(len(stations) - 1):
        start = stations[index]
        stop = stations[index + 1]
        span = stop - start
        length2 = float(np.dot(span, span))
        if length2 < 1e-12:
            toward = 0.0
            projection = start
        else:
            toward = float(np.clip(np.dot(point - start, span) / length2, 0.0, 1.0))
            projection = start + toward * span
        distance = float(np.linalg.norm(point - projection))
        # A point sitting on a joint station belongs to the child segment.
        rank = index if toward > 1e-6 else index - 1
        if distance < best_distance - 1e-6 or (abs(distance - best_distance) <= 1e-6 and rank > best_rank):
            best_distance = distance
            best_link = owners[index]
            best_rank = rank
    return best_link


def _s5_kinematics(
    instances: list[Instance],
    centroids: dict[Path, np.ndarray],
    meshes: dict[Path, trimesh.Trimesh],
):
    """S5 has no Origin_* frames.

    Each worm is the motor, spinning beside the joint. The arm joint is the
    worm wheel (its thin axis). The reducer shaft and the gripper pin are
    shafts, so their joint is the long axis. The worm motor stays on the
    parent link.
    """
    wheels = [instance for instance in instances if "蜗轮40" in instance.name]
    if len(wheels) != 2:
        raise RuntimeError(f"S5 expected 2 worm wheels, found {len(wheels)}")
    wheels.sort(key=lambda instance: -float(instance.world[2, 3]))
    worms = [instance for instance in instances if "蜗杆" in instance.name]
    wrist = next(
        instance
        for instance in instances
        if instance.group == "电机减速机装配总成" and "减速机连接轴" in instance.name
    )
    gripper = next(
        instance
        for instance in instances
        if instance.group == "电机夹爪装配总成" and "夹爪连接轴" in instance.name
    )
    frames = {
        "base": np.eye(4),
        "proximal": wheels[0].world.copy(),
        "distal": wheels[1].world.copy(),
        "wrist": wrist.world.copy(),
        "gripper": gripper.world.copy(),
    }
    links = (
        ("base_link", None, None, "base"),
        ("proximal_link", "base_link", "revolute", "proximal"),
        ("distal_link", "proximal_link", "revolute", "distal"),
        ("wrist_link", "distal_link", "revolute", "wrist"),
        ("gripper_link", "wrist_link", "revolute", "gripper"),
    )
    outputs = {
        "proximal_link": wheels[0],
        "distal_link": wheels[1],
        "wrist_link": wrist,
        "gripper_link": gripper,
    }
    axes = {
        link: part_axis(part.world, _feature_axis_column(meshes[part.source.resolve()]))
        for link, part in outputs.items()
    }
    stations = [
        np.array([0.02, 0.0, -0.02]),
        wheels[0].world[:3, 3].copy(),
        wheels[1].world[:3, 3].copy(),
        wrist.world[:3, 3].copy(),
        gripper.world[:3, 3].copy(),
    ]
    owners = ["base_link", "proximal_link", "distal_link", "wrist_link"]
    parent_of_wheel = {id(wheels[0]): "base_link", id(wheels[1]): "proximal_link"}
    assigned: dict[int, str] = {}
    for index, instance in enumerate(instances):
        if instance.group == "电机夹爪装配总成":
            assigned[index] = "gripper_link"
        elif instance.group == "电机减速机装配总成":
            assigned[index] = "wrist_link"
        elif instance.group == "电机蜗杆装配总成":
            nearest = min(wheels, key=lambda wheel: float(np.linalg.norm(instance.world[:3, 3] - wheel.world[:3, 3])))
            assigned[index] = parent_of_wheel[id(nearest)]
        elif "蜗轮" in instance.name:
            nearest = min(wheels, key=lambda wheel: float(np.linalg.norm(instance.world[:3, 3] - wheel.world[:3, 3])))
            assigned[index] = "proximal_link" if nearest is wheels[0] else "distal_link"
        else:
            centroid = centroids.get(instance.source.resolve(), np.zeros(3))
            point = (instance.world @ np.append(centroid, 1.0))[:3]
            assigned[index] = _chain_link(point, stations, owners)
    note = (
        "No Origin_* coordinate systems. Pitch joints are the worm wheels; "
        "the worms are motors and stay on the parent link. "
        "Mass uses an aluminum density of 2700 kg/m^3."
    )
    return frames, axes, assigned, links, note


def _s7_kinematics(
    instances: list[Instance],
    centroids: dict[Path, np.ndarray],
    meshes: dict[Path, trimesh.Trimesh] | None = None,
):
    """S7 has no Origin_* frames.

    Shoulder and elbow are the hinge pin and the bevel. The forearm then has
    two more motors, both named 电机轴2: one along X, then one along Z, before
    the gripper roll.
    """
    del meshes
    # The shoulder housings turn on 固定轴1. The elbow is the bevel at the end
    # of the upper arm. 电机轴2 is a pair of motors on the forearm, not that hinge.
    upper = next(instance for instance in instances if "固定轴1" in instance.name)
    elbow = next(instance for instance in instances if "锥齿轮SMS2-25L" in instance.name)
    forearm_shafts = [instance for instance in instances if "电机轴2" in instance.name]
    if len(forearm_shafts) != 2:
        raise RuntimeError(f"S7 expected 2 电机轴2, found {len(forearm_shafts)}")
    wrist_pitch = max(forearm_shafts, key=lambda instance: abs(float(part_axis(instance.world, 1)[0])))
    wrist_yaw = max(forearm_shafts, key=lambda instance: abs(float(part_axis(instance.world, 1)[2])))
    roll = next(instance for instance in instances if SHAFT_PART in instance.name and "夹爪" in instance.group)
    racks = [instance for instance in instances if RACK_PART in instance.name]
    if len(racks) != 2:
        raise RuntimeError(f"S7 expected 2 racks, found {len(racks)}")
    gripper_world = next(instance.gripper_world for instance in racks if instance.gripper_world is not None)
    jaw = part_axis(racks[0].world, 0)

    def along(instance: Instance) -> float:
        local = np.linalg.inv(gripper_world) @ np.append(instance.world[:3, 3], 1.0)
        axis = gripper_world[:3, :3].T @ jaw
        axis = axis / np.linalg.norm(axis)
        return float(np.dot(local[:3], axis))

    left = max(racks, key=along)
    right = min(racks, key=along)
    frames = {
        "base": np.eye(4),
        "upper": upper.world.copy(),
        "forearm": elbow.world.copy(),
        "wrist_pitch": wrist_pitch.world.copy(),
        "wrist_yaw": wrist_yaw.world.copy(),
        "roll": roll.world.copy(),
        "catch_l": left.world.copy(),
        "catch_r": right.world.copy(),
    }
    links = (
        ("base_link", None, None, "base"),
        ("upper_link", "base_link", "revolute", "upper"),
        ("forearm_link", "upper_link", "revolute", "forearm"),
        ("wrist_pitch_link", "forearm_link", "revolute", "wrist_pitch"),
        ("wrist_yaw_link", "wrist_pitch_link", "revolute", "wrist_yaw"),
        ("roll_link", "wrist_yaw_link", "revolute", "roll"),
        ("catch_l_link", "roll_link", "prismatic", "catch_l"),
        ("catch_r_link", "roll_link", "prismatic", "catch_r"),
    )
    axes = {
        "upper_link": part_axis(upper.world, 1),
        "forearm_link": part_axis(elbow.world, 1),
        "wrist_pitch_link": part_axis(wrist_pitch.world, 1),
        "wrist_yaw_link": part_axis(wrist_yaw.world, 1),
        "roll_link": part_axis(roll.world, 1),
        "catch_l_link": part_axis(left.world, 0),
        "catch_r_link": part_axis(right.world, 0),
    }
    if float(np.dot(axes["catch_l_link"], axes["catch_r_link"])) > 0:
        axes["catch_r_link"] = -axes["catch_r_link"]
    groups = {
        "WLT-ARM-02 大臂装配": "upper_link",
        "小臂连接": "upper_link",
    }
    # The X-axis 电机轴2 is mounted on the elbow side of the forearm. Parts
    # further toward the gripper, including the vertical 电机轴2, pitch about it.
    pitch_station = float(wrist_pitch.world[1, 3])
    assigned: dict[int, str] = {}
    for index, instance in enumerate(instances):
        if instance.under_gripper or "夹爪V2" in instance.group:
            value = along(instance)
            if abs(value) > JAW_SPLIT_M:
                assigned[index] = "catch_l_link" if value > 0 else "catch_r_link"
            else:
                assigned[index] = "roll_link"
            continue
        if instance.group == "WLT-ARM-04 小臂装配":
            centroid = centroids.get(instance.source.resolve(), np.zeros(3))
            point = (instance.world @ np.append(centroid, 1.0))[:3]
            assigned[index] = "forearm_link" if float(point[1]) >= pitch_station - 1e-4 else "wrist_pitch_link"
            continue
        assigned[index] = groups.get(instance.group, "base_link")
    note = (
        "No Origin_* coordinate systems. Shoulder is 固定轴1, elbow is the upper-arm "
        "bevel, then the two 电机轴2 joints (pitch about X, yaw about Z). "
        "Mass uses an aluminum density of 2700 kg/m^3."
    )
    return frames, axes, assigned, links, note


def _link_spec(link_name: str) -> tuple[str, str | None, str | None, str]:
    return next(spec for spec in _ACTIVE_LINKS if spec[0] == link_name)


def _link_color(link_name: str) -> tuple[float, float, float, float]:
    if link_name in LINK_COLOR:
        return LINK_COLOR[link_name]
    palette = (
        (0.45, 0.47, 0.50, 1.0),
        (0.20, 0.45, 0.85, 1.0),
        (0.20, 0.65, 0.40, 1.0),
        (0.95, 0.55, 0.20, 1.0),
        (0.55, 0.35, 0.75, 1.0),
        (0.15, 0.55, 0.55, 1.0),
        (0.85, 0.25, 0.25, 1.0),
        (0.90, 0.80, 0.20, 1.0),
    )
    names = [spec[0] for spec in _ACTIVE_LINKS]
    index = names.index(link_name) if link_name in names else 0
    return palette[index % len(palette)]


def rigid_root(link_name: str) -> str:
    """Link that owns this geometry once fixed joints are removed."""
    current = link_name
    seen: set[str] = set()
    while current not in seen:
        seen.add(current)
        _name, parent, joint_type, _frame = _link_spec(current)
        if parent is None or joint_type != "fixed":
            return current
        current = parent
    return current


def merge_rigid_meshes(
    visuals: list[Visual],
    frames: dict[str, np.ndarray],
    mesh_dir: Path,
) -> tuple[list[Visual], dict[str, list[trimesh.Trimesh]]]:
    """One mesh per moving link. Fixed-joint geometry is baked into its parent."""
    placed_by_link: dict[str, list[trimesh.Trimesh]] = {}
    solids_by_link: dict[str, list[trimesh.Trimesh]] = {}
    for visual in visuals:
        root = rigid_root(visual.link)
        origin = visual.origin
        if visual.link != root:
            root_frame = frames[_link_spec(root)[3]]
            link_frame = frames[_link_spec(visual.link)[3]]
            origin = np.linalg.inv(root_frame) @ link_frame @ origin
        placed = visual.mesh.copy()
        placed.apply_transform(origin)
        placed_by_link.setdefault(root, []).append(placed)
        solids_by_link.setdefault(root, []).append(_solid_mesh(placed))

    merged: list[Visual] = []
    mesh_dir.mkdir(parents=True, exist_ok=True)
    for link_name, _parent, _joint_type, _frame in _ACTIVE_LINKS:
        if rigid_root(link_name) != link_name:
            continue
        parts = placed_by_link.get(link_name, [])
        if not parts:
            continue
        combined = trimesh.util.concatenate(parts)
        filename = f"meshes/{link_name}.stl"
        combined.export(mesh_dir / f"{link_name}.stl")
        merged.append(Visual(link_name, filename, np.eye(4), combined))
    keep = {mesh_dir / f"{visual.link}.stl" for visual in merged}
    for old in mesh_dir.glob("*.stl"):
        if old.resolve() not in {path.resolve() for path in keep}:
            old.unlink()
    return merged, solids_by_link


def _write_urdf(
    path: Path,
    config: str,
    frames: dict[str, np.ndarray],
    axes: dict[str, np.ndarray],
    visuals: list[Visual],
    density: float,
    solids: dict[str, list[trimesh.Trimesh]] | None = None,
    robot_name: str = "s3_arm",
    mass_note: str = "",
) -> Path:
    robot = ET.Element("robot", name=robot_name)
    note = mass_note or f"Mass {ASSEMBLY_MASS_KG:.4f} kg is distributed by mesh volume."
    robot.append(
        ET.Comment(
            f" Active SolidWorks configuration: {config}. "
            "Joint limits are placeholders. "
            f"{note} "
            "Each link is one mesh. Fixed joints are baked into the parent link. "
        )
    )
    by_link: dict[str, list[Visual]] = {name: [] for name, *_ in _ACTIVE_LINKS}
    for visual in visuals:
        by_link[visual.link].append(visual)

    for link_name, parent, joint_type, frame_name in _ACTIVE_LINKS:
        if rigid_root(link_name) != link_name:
            continue
        link = ET.SubElement(robot, "link", name=link_name)
        placed_solids = list(solids.get(link_name, [])) if solids is not None else []
        for visual in by_link[link_name]:
            if solids is None:
                placed = visual.mesh.copy()
                placed.apply_transform(visual.origin)
                placed_solids.append(_solid_mesh(placed))
            node = ET.SubElement(link, "visual")
            _origin(node, visual.origin)
            geom = ET.SubElement(node, "geometry")
            ET.SubElement(geom, "mesh", filename=visual.mesh_file)
            material = ET.SubElement(node, "material", name=link_name)
            red, green, blue, _alpha = _link_color(link_name)
            ET.SubElement(material, "color", rgba=f"{red} {green} {blue} 1")
            collision = ET.SubElement(link, "collision")
            _origin(collision, visual.origin)
            collision_geom = ET.SubElement(collision, "geometry")
            ET.SubElement(collision_geom, "mesh", filename=visual.mesh_file)
        inertial = ET.SubElement(link, "inertial")
        if placed_solids:
            combined = trimesh.util.concatenate(placed_solids)
            combined.density = density
            mass = abs(float(combined.mass))
            com = combined.center_mass
            inertia = np.asarray(combined.moment_inertia, dtype=float)
        else:
            mass = 1e-4
            com = np.zeros(3)
            inertia = np.eye(3) * 1e-8
        origin = ET.SubElement(inertial, "origin")
        origin.set("xyz", _fmt_vec(com))
        origin.set("rpy", "0 0 0")
        ET.SubElement(inertial, "mass", value=f"{mass:.8g}")
        ET.SubElement(
            inertial,
            "inertia",
            ixx=f"{inertia[0, 0]:.8g}",
            ixy=f"{inertia[0, 1]:.8g}",
            ixz=f"{inertia[0, 2]:.8g}",
            iyy=f"{inertia[1, 1]:.8g}",
            iyz=f"{inertia[1, 2]:.8g}",
            izz=f"{inertia[2, 2]:.8g}",
        )
        if parent is not None and joint_type is not None and joint_type != "fixed":
            urdf_parent = rigid_root(parent)
            parent_frame = _link_spec(urdf_parent)[3]
            joint_origin = np.linalg.inv(frames[parent_frame]) @ frames[frame_name]
            joint = ET.SubElement(robot, "joint", name=link_name.removesuffix("_link"), type=joint_type)
            ET.SubElement(joint, "parent", link=urdf_parent)
            ET.SubElement(joint, "child", link=link_name)
            _origin(joint, joint_origin)
            if joint_type != "fixed":
                axis = frames[frame_name][:3, :3].T @ axes[link_name]
                ET.SubElement(joint, "axis", xyz=_fmt_vec(axis))
                lower, upper = JOINT_LIMITS[joint_type]
                ET.SubElement(
                    joint,
                    "limit",
                    lower=f"{lower:.6g}",
                    upper=f"{upper:.6g}",
                    effort="20",
                    velocity="1",
                )
    tree = ET.ElementTree(robot)
    ET.indent(tree, space="  ")
    path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return path


def _origin(node: ET.Element, transform: np.ndarray) -> None:
    roll, pitch, yaw = rpy_from_matrix(transform[:3, :3])
    origin = ET.SubElement(node, "origin")
    origin.set("xyz", _fmt_vec(transform[:3, 3]))
    origin.set("rpy", _fmt_vec((roll, pitch, yaw)))


def _fmt_vec(values) -> str:
    return " ".join(f"{float(v):.8g}" for v in values)


def view(urdf_path: Path) -> None:
    from view_urdf import view as view_urdf

    view_urdf(urdf_path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=("s3", "s5", "s7"), default="s3")
    parser.add_argument("--assembly", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--view", action="store_true", help="open the URDF in viser after building")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output_root = Path(__file__).resolve().parent / "output"
    if args.model == "s5":
        assembly = args.assembly or (root / "S5" / "S5、S3机械臂装配总成" / "WLT-S5ARM-00 S5机械臂装配总成.SLDASM")
        output = args.output or (output_root / "s5_arm")
        build(
            assembly,
            output,
            extra_dirs=[root / "s3机械臂_1"],
            kinematics=_s5_kinematics,
            robot_name="s5_arm",
            mass_kg=None,
        )
        urdf = output / "s5_arm.urdf"
    elif args.model == "s7":
        assembly = args.assembly or (root / "S7" / "WLT-ARM-00 机械臂总成" / "WLT-ARM-00 机械臂总成.SLDASM")
        output = args.output or (output_root / "s7_arm")
        build(
            assembly,
            output,
            extra_dirs=[root / "s3机械臂_1"],
            kinematics=_s7_kinematics,
            robot_name="s7_arm",
            mass_kg=None,
        )
        urdf = output / "s7_arm.urdf"
    else:
        assembly = args.assembly or _default_assembly()
        output = args.output or (output_root / "s3_arm")
        build(assembly, output)
        urdf = output / "s3_arm.urdf"
    if args.view:
        view(urdf)


if __name__ == "__main__":
    main()
