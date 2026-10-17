"""Print plates for a Bambu A1 (256 x 256): each plate a 3MF of the project's STL/ parts, each part turned to its print face and packed with gaps.
Run after the STL export:  python plates.py   -> plates/*.3mf + plates/plates.png (a top view of every plate).
Orientation per part: 'as' (the export's own print face), 'flip' (turned over about x), 'flat' (laid on its largest down-facing flat: the export
leaves it as it sits in the case)."""
import sys, numpy as np, trimesh
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[2]))
from print_supports import walls
from shapely.geometry import MultiPoint
from pathlib import Path
from trimesh.transformations import rotation_matrix
HERE = Path(__file__).parent
STL, OUT = HERE / "STL", HERE / "plates"
BED, MARGIN, GAP = 256.0, 6.0, 5.0
SUPPORTED = {"velvet_hull_R", "velvet_hull_L", "velvet_hatch_R", "velvet_hatch_L"}   # designed walls under their lifted flat faces, merged in (print_supports.py); the slicer supports the rest

# (plate, material, [(stl, copies, orientation)])  -- materials are a suggestion; Willy picks the filament per plate in the slicer
PLATES = [
    ("00_test_coupons", "body PLA/PETG", [("velvet_pin_coupon_0.2", 1, "as"), ("velvet_pin_coupon_0.3", 1, "as"), ("velvet_pin_coupon_0.4", 1, "as")]),
    ("01_hull_R", "body", [("velvet_hull_R", 1, "as")]),
    ("02_hull_L", "body", [("velvet_hull_L", 1, "as")]),
    ("03_base_R", "body", [("velvet_base_R", 1, "as")]),
    ("04_base_L", "body", [("velvet_base_L", 1, "as")]),
    ("05_hatch_R_pins", "body", [("velvet_hatch_R", 1, "flip"), ("velvet_tent_pin_R", 1, "as"), ("velvet_tent_pin_L", 1, "as"), ("velvet_rest_pin_R", 1, "as"),
                                 ("velvet_rest_pin_L", 1, "as"), ("velvet_flap_pin", 4, "as"), ("velvet_pi_stack_spacer", 2, "as"), ("velvet_pi_otg_cover", 1, "as")]),
    ("06_hatch_L_flap_knob", "body", [("velvet_hatch_L", 1, "flip"), ("velvet_flap", 2, "as"), ("velvet_knob", 1, "as")]),
    ("07_crowns_caps", "crown colour", [("velvet_crown_R", 1, "as"), ("velvet_crown_L", 1, "as"), ("velvet_pitft_cap", 4, "as")]),
    ("08_clear", "clear", [("velvet_light_guide_cyan_R", 1, "as"), ("velvet_light_guide_magenta_R", 1, "as"), ("velvet_light_guide_cyan_L", 1, "as"),
                           ("velvet_light_guide_magenta_L", 1, "as"), ("velvet_led_diffuser_R", 1, "as"), ("velvet_pico_pane_R", 1, "flat"), ("velvet_pico_pane_L", 1, "flat")]),
    ("09_overlay", "trackpad overlay colour", [("velvet_tp_overlay_R", 1, "as")]),
    ("10_tpu_pads", "TPU", [("velvet_wrist_pad_tpu_R", 1, "flat"), ("velvet_wrist_pad_tpu_L", 1, "flat")]),
]


def lay_flat(m):
    """Rest the part on the convex-hull facet whose pose leaves the least overhang (down-facing past 45 deg, off the bed); ties: the lower pose."""
    h = m.convex_hull
    best = None
    for i in range(len(h.facets)):
        if h.facets_area[i] < 20.0:
            continue
        v = h.facets_normal[i]; t = np.array([0.0, 0.0, -1.0]); ax = np.cross(v, t); s = np.linalg.norm(ax)
        T = np.eye(4) if s < 1e-9 and np.dot(v, t) > 0 else rotation_matrix(np.pi, [1, 0, 0]) if s < 1e-9 else rotation_matrix(np.arctan2(s, np.dot(v, t)), ax / s)
        q = m.copy(); q.apply_transform(T); z0 = q.bounds[0][2]
        n, c, a = q.face_normals, q.triangles_center[:, 2] - z0, q.area_faces
        key = (round(a[(n[:, 2] < -0.7071) & (c > 0.3)].sum() / 20.0), q.extents[2])
        if best is None or key < best[0]:
            best = (key, T)
    m.apply_transform(best[1])
    return m


def load(name, how):
    m = trimesh.load(STL / f"{name}.stl"); m.merge_vertices()
    if how == "flip":
        m.apply_transform(rotation_matrix(np.pi, [1, 0, 0]))
    elif how == "flat":
        m = lay_flat(m)
    # the footprint's long side along y (the bed's depth), then on the bed
    if m.extents[0] > m.extents[1]:
        m.apply_transform(rotation_matrix(np.pi / 2, [0, 0, 1]))
    m.apply_translation(-m.bounds[0])
    if name in SUPPORTED:
        w, _ = walls(m)
        m = trimesh.util.concatenate([m, w])
    return m


def pack(parts):
    """Shelf packing, tallest footprint first, columns along x: returns [(mesh, (x, y))] or raises when a part does not fit the bed."""
    parts = sorted(parts, key=lambda p: -p[1].extents[1])
    out, x, y, col_w = [], MARGIN, MARGIN, 0.0
    for name, m in parts:
        w, d = m.extents[0], m.extents[1]
        if y + d > BED - MARGIN:                                      # next column
            x, y, col_w = x + col_w + GAP, MARGIN, 0.0
        assert x + w <= BED - MARGIN and y + d <= BED - MARGIN, f"{name} ({w:.0f} x {d:.0f}) does not fit the plate at ({x:.0f}, {y:.0f})"
        out.append((name, m, (x, y)))
        y += d + GAP; col_w = max(col_w, w)
    # centre the whole layout on the bed
    xs = [p[2][0] + p[1].extents[0] for p in out]; ys = [p[2][1] + p[1].extents[1] for p in out]
    dx, dy = (BED - max(xs) - MARGIN) / 2.0, (BED - max(ys) - MARGIN) / 2.0
    return [(n, m, (x + dx, y + dy)) for n, m, (x, y) in out]


def main():
    OUT.mkdir(exist_ok=True)
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, axs = plt.subplots(3, 4, figsize=(20, 15)); axs = axs.ravel()
    lines = []
    for i, (plate, mat, items) in enumerate(PLATES):
        parts = [(f"{n}#{c + 1}" if k > 1 else n, load(n, how)) for n, k, how in items for c in range(k)]
        placed = pack(parts)
        sc = trimesh.Scene()
        for n, m, (x, y) in placed:
            q = m.copy(); q.apply_translation([x, y, 0.0]); sc.add_geometry(q, node_name=n, geom_name=n)
        sc.export(str(OUT / f"{plate}.3mf"))
        h = max(m.extents[2] for _, m, _ in placed); vol = sum(m.volume for _, m, _ in placed) / 1000.0
        lines.append(f"{plate:24} {mat:24} {len(placed):2} parts, tallest {h:5.1f} mm, {vol:6.1f} cm3: " + ", ".join(n for n, _, _ in placed))
        ax = axs[i]; ax.add_patch(plt.Rectangle((0, 0), BED, BED, fill=False, lw=1.5))
        for n, m, (x, y) in placed:
            g = MultiPoint(m.vertices[:, :2]).convex_hull                  # ponytail: the footprint's hull for the preview, not its true outline
            xx, yy = g.exterior.xy; ax.fill(np.array(xx) + x, np.array(yy) + y, alpha=0.6); ax.text(x + 1, y + 1, n.replace("velvet_", ""), fontsize=6)
        ax.set_xlim(-5, BED + 5); ax.set_ylim(-5, BED + 5); ax.set_aspect("equal"); ax.set_title(f"{plate} ({mat})", fontsize=9)
    for ax in axs[len(PLATES):]: ax.axis("off")
    fig.tight_layout(); fig.savefig(OUT / "plates.png", dpi=80)
    (OUT / "plates.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
