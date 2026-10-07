"""Mesh one SolidWorks part in metres, in the part frame.

Order:

1. The tessellation SolidWorks saved in the .SLDPRT (cadmpeg decode). This is
   what SolidWorks draws. Vertices are millimetres.
2. A STEP mesh, only when that tessellation is missing. cascadio returns
   metres. The GLB is a scene: each face lives in its node frame, and
   concatenating geometry.values() leaves the gripper fins unstacked.
   Bake the scene graph with Scene.to_geometry().

cadmpeg's own STEP export of a .SLDPRT often emits untrimmed surfaces. Those
tessellate as single triangles the size of the part. Do not prefer that
export over the saved tessellation.
"""

from __future__ import annotations

import io
import json
import os
import shutil
import subprocess
from pathlib import Path

import cascadio
import numpy as np
import trimesh

from conversion.sw_math import MM_TO_M

MESH_TOL_MM = 0.3


def cadmpeg_binary() -> Path:
    """Locate the cadmpeg CLI.

    Search order: ``CADMPEG``, ``PATH``, then ``.tools/cadmpeg`` in this
    repository or any parent of the current directory.
    """
    candidates: list[Path] = []
    env = os.environ.get("CADMPEG")
    if env:
        candidates.append(Path(env))
    found = shutil.which("cadmpeg")
    if found:
        candidates.append(Path(found))
    # src/conversion/mesh_part.py -> repository root
    candidates.append(Path(__file__).resolve().parents[2] / ".tools" / "cadmpeg")
    for start in (Path.cwd(), Path(__file__).resolve()):
        for parent in (start, *start.parents):
            candidates.append(parent / ".tools" / "cadmpeg")
    seen: set[str] = set()
    for path in candidates:
        key = str(path)
        if key in seen:
            continue
        seen.add(key)
        if path.is_file() and os.access(path, os.X_OK):
            return path
    raise FileNotFoundError(
        "cadmpeg was not found. Install it and put the binary on PATH, "
        "set CADMPEG to its path, or copy it to .tools/cadmpeg in this repository. "
        "See README.md."
    )


def mesh_from_display_list(source: Path, scratch: Path) -> trimesh.Trimesh | None:
    scratch.mkdir(parents=True, exist_ok=True)
    slug = "".join(char if char.isalnum() or char in "._-" else "_" for char in source.stem).strip("_")
    cadir = scratch / f"{slug or 'part'}.cadir.json"
    result = subprocess.run(
        [str(cadmpeg_binary()), "decode", "--force", "-o", str(cadir), str(source)],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0 or not cadir.is_file():
        return None
    model = json.loads(cadir.read_text())["model"]
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    for tessellation in model.get("tessellations", []):
        base = len(vertices)
        for vertex in tessellation["vertices"]:
            vertices.append((float(vertex["x"]), float(vertex["y"]), float(vertex["z"])))
        for triangle in tessellation["triangles"]:
            faces.append((base + int(triangle[0]), base + int(triangle[1]), base + int(triangle[2])))
    if not faces:
        return None
    mesh = trimesh.Trimesh(vertices=np.asarray(vertices), faces=np.asarray(faces), process=False)
    mesh.apply_scale(MM_TO_M)
    mesh.remove_unreferenced_vertices()
    return mesh


def mesh_from_step(step: Path, tol_linear: float = MESH_TOL_MM) -> trimesh.Trimesh | None:
    glb = cascadio.load(
        step.read_bytes(),
        file_type="step",
        tol_linear=tol_linear,
        tol_angular=0.5,
    )
    if not glb:
        return None
    loaded = trimesh.load(io.BytesIO(glb), file_type="glb", process=False)
    if isinstance(loaded, trimesh.Scene):
        mesh = loaded.to_geometry()
        if isinstance(mesh, list):
            mesh = trimesh.util.concatenate(mesh) if mesh else None
    elif isinstance(loaded, trimesh.Trimesh):
        mesh = loaded
    else:
        return None
    if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
        return None
    mesh.remove_unreferenced_vertices()
    return mesh


def mesh_part(source: Path, scratch: Path, step: Path | None = None) -> trimesh.Trimesh | None:
    """Saved tessellation first. STEP is the fallback, including real vendor STEP files."""
    if source.suffix.lower() not in {".step", ".stp"}:
        mesh = mesh_from_display_list(source, scratch)
        if mesh is not None:
            return mesh
    if step is not None and step.is_file():
        return mesh_from_step(step)
    if source.suffix.lower() in {".step", ".stp"}:
        return mesh_from_step(source)
    return None
