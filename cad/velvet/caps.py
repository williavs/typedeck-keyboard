"""Keycaps: DSA (Willy 2026-10-12 00:00 'dsa caps - pbt ... full 131 key set'), as KeyV2's `dsa_row(3) u(N) key()` (the published
parametric of Signature Plastics' DSA; Willy's set is bought, these are its envelope for the clash checks and the wells).
OpenSCAD (CGAL) writes an OFF once per size into STL/cap_cache/; the solid is a ruled loft of the mesh's horizontal sections."""
import subprocess
from pathlib import Path

import numpy as np
import trimesh
from build123d import Polyline, Pos, loft, make_face

from velvet.params import DSA_H, HERE, SL

KEYV2 = SL.REFS / "keyv2"
OPENSCAD = "/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD"
CACHE = HERE.parent / "STL" / "cap_cache"
U = 19.05
_solids = {}


def cap_size(u):
    """The DSA cap's footprint (w along the key's x, d) for a u-wide key: KeyV2 measures 18.24 for 1u, 27.76 for 1.5u."""
    return u * U - 0.81, U - 0.81


def dsa_mesh(u):
    CACHE.mkdir(parents=True, exist_ok=True)
    off = CACHE / f"dsa_{u}u.off"
    if not off.exists():
        src = CACHE / f"dsa_{u}u.scad"
        src.write_text(f"include <{KEYV2}/includes.scad>\ndsa_row(3) u({u}) key();\n")
        if subprocess.run([OPENSCAD, "--backend=CGAL", "-o", str(off), str(src)], capture_output=True).returncode:
            subprocess.run([OPENSCAD, "--backend=Manifold", "-o", str(off), str(src)], check=True, capture_output=True)
    tm = trimesh.load(off, process=False)
    b = tm.bounds
    tm.apply_translation([-(b[0][0] + b[1][0]) / 2, -(b[0][1] + b[1][1]) / 2, -b[0][2]])   # centred, skirt on z 0
    return tm


def dsa_cap(u=1.0):
    """The DSA cap as an OCC solid, skirt on z 0, centred: a ruled loft of 6 sections of the KeyV2 mesh (convex, 0.08 simplified)."""
    if u not in _solids:
        tm = dsa_mesh(u)
        zmax = tm.bounds[1][2]
        assert abs(zmax - DSA_H) < 0.02, f"the DSA {u}u mesh is {zmax:.3f} tall, params.DSA_H says {DSA_H}"   # the key wall's ceiling rests on it (build 148)
        faces = []
        for z in (0.05, 2.0, 4.0, 5.5, 6.5, zmax - 0.3):
            s = tm.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
            p2, _ = s.to_planar(normal=[0, 0, 1], check=False)
            poly = max(p2.polygons_full, key=lambda q: q.area).convex_hull.simplify(0.08)
            faces.append(make_face(Polyline(*[(x, y, z) for x, y in list(poly.exterior.coords)[:-1]], close=True)))
        _solids[u] = loft(faces, ruled=True)
    return _solids[u]
