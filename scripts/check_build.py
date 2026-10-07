"""Fail a URDF that still has one of the known SolidWorks conversion failures.

A link whose bounding box exceeds half a metre was placed with the rotation
left untransposed, or a part origin was used in place of the mesh centroid.
An edge longer than 25 cm is an untrimmed surface from a cadmpeg STEP export
or an unbaked GLB scene graph. The S3 arm's longest genuine edge is about
18 cm, on the roll tube. A part that is itself longer, such as the S7 1 m
hose, exceeds these limits without being misplaced.
"""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

MAX_EXTENT_M = 0.5
MAX_EDGE_M = 0.25


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", type=Path)
    args = parser.parse_args()
    root = ET.parse(args.urdf).getroot()
    failed = False
    for link in root.findall("link"):
        name = link.attrib["name"]
        for mesh_node in link.findall("visual/geometry/mesh"):
            path = args.urdf.parent / mesh_node.attrib["filename"]
            mesh = trimesh.load(path, force="mesh", process=False)
            extent = float(np.max(mesh.extents))
            longest = float(mesh.edges_unique_length.max()) if len(mesh.faces) else 0.0
            flag = ""
            if extent > MAX_EXTENT_M:
                flag = "  EXTENT"
                failed = True
            if longest > MAX_EDGE_M:
                flag += "  SHARD"
                failed = True
            print(f"{name:16} extent {extent:7.3f} m   longest edge {longest:7.3f} m{flag}")
    print("joints")
    for joint in root.findall("joint"):
        axis = joint.find("axis")
        if axis is None:
            continue
        vector = np.array([float(v) for v in axis.attrib["xyz"].split()])
        length = float(np.linalg.norm(vector))
        print(f"  {joint.attrib['name']:12} {joint.attrib['type']:10} axis {np.round(vector, 3)} |a|={length:.4f}")
        if abs(length - 1.0) > 1e-3:
            failed = True
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
