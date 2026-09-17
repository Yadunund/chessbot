#!/usr/bin/env python3
# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Generate OBJ meshes (metres, z up, base at z = 0) for each piece type from pieces.json.

    make_meshes.py pieces.json out_dir
"""

import json
import math
import os
import sys

SEGMENTS = 40


class Mesh:
    def __init__(self):
        self.vertices: list[tuple[float, float, float]] = []
        self.faces: list[tuple[int, int, int]] = []

    def vertex(self, x, y, z) -> int:
        self.vertices.append((x, y, z))
        return len(self.vertices)  # OBJ indices are 1-based

    def write(self, path):
        # Smooth vertex normals: the area-weighted sum of adjacent face normals.
        normals = [[0.0, 0.0, 0.0] for _ in self.vertices]
        for a, b, c in self.faces:
            pa, pb, pc = self.vertices[a - 1], self.vertices[b - 1], self.vertices[c - 1]
            u = [pb[i] - pa[i] for i in range(3)]
            v = [pc[i] - pa[i] for i in range(3)]
            n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
            for index in (a, b, c):
                for i in range(3):
                    normals[index - 1][i] += n[i]
        with open(path, "w") as f:
            f.writelines(f"v {x:.6f} {y:.6f} {z:.6f}\n" for x, y, z in self.vertices)
            for n in normals:
                length = math.sqrt(sum(c * c for c in n)) or 1.0
                f.write(f"vn {n[0] / length:.4f} {n[1] / length:.4f} {n[2] / length:.4f}\n")
            f.writelines(f"f {a}//{a} {b}//{b} {c}//{c}\n" for a, b, c in self.faces)


def lathe(mesh: Mesh, profile, radius: float, height: float):
    """Revolve [radius fraction, height fraction] points about z."""
    rings = []
    for r, h in profile:
        ring = [mesh.vertex(r * radius * math.cos(2 * math.pi * i / SEGMENTS),
                            r * radius * math.sin(2 * math.pi * i / SEGMENTS), h * height) for i in range(SEGMENTS)]
        rings.append(ring)
    for lower, upper in zip(rings, rings[1:]):
        for i in range(SEGMENTS):
            j = (i + 1) % SEGMENTS
            mesh.faces.append((lower[i], lower[j], upper[j]))
            mesh.faces.append((lower[i], upper[j], upper[i]))


def box(mesh: Mesh, centre, size):
    cx, cy, cz = centre
    sx, sy, sz = (s / 2 for s in size)
    v = [mesh.vertex(cx + dx * sx, cy + dy * sy, cz + dz * sz) for dx in (-1, 1) for dy in (-1, 1) for dz in (-1, 1)]
    # Vertex order: index = 4*(dx>0) + 2*(dy>0) + (dz>0)
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    for a, b, c, d in quads:
        mesh.faces.append((v[a], v[b], v[c]))
        mesh.faces.append((v[a], v[c], v[d]))


def triangulate(polygon):
    """Ear clipping for a simple polygon; returns index triples."""
    points = list(range(len(polygon)))
    area = sum(polygon[i][0] * polygon[(i + 1) % len(polygon)][1] - polygon[(i + 1) % len(polygon)][0] * polygon[i][1]
               for i in range(len(polygon)))
    if area < 0:
        points.reverse()

    def inside(p, a, b, c):
        def side(p1, p2, p3):
            return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])
        d1, d2, d3 = side(p, a, b), side(p, b, c), side(p, c, a)
        return not ((d1 < 0 or d2 < 0 or d3 < 0) and (d1 > 0 or d2 > 0 or d3 > 0))

    triangles = []
    while len(points) > 3:
        for k in range(len(points)):
            i, j, m = points[k - 1], points[k], points[(k + 1) % len(points)]
            a, b, c = polygon[i], polygon[j], polygon[m]
            if (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) <= 0:
                continue  # reflex corner
            if any(inside(polygon[o], a, b, c) for o in points if o not in (i, j, m)):
                continue
            triangles.append((i, j, m))
            points.pop(k)
            break
        else:
            break  # degenerate; give up on the rest
    if len(points) == 3:
        triangles.append(tuple(points))
    return triangles


def extrude(mesh: Mesh, outline, thickness):
    """Extrude an (x, z) outline along y, centred."""
    front = [mesh.vertex(x, -thickness / 2, z) for x, z in outline]
    back = [mesh.vertex(x, thickness / 2, z) for x, z in outline]
    for a, b, c in triangulate(outline):
        mesh.faces.append((front[a], front[c], front[b]))
        mesh.faces.append((back[a], back[b], back[c]))
    n = len(outline)
    for i in range(n):
        j = (i + 1) % n
        mesh.faces.append((front[i], front[j], back[j]))
        mesh.faces.append((front[i], back[j], back[i]))


def main():
    source, out_dir = sys.argv[1], sys.argv[2]
    with open(source) as f:
        spec = json.load(f)
    os.makedirs(out_dir, exist_ok=True)
    radius = spec["base_diameter_m"] / 2
    for kind, profile in spec["profile"].items():
        height = spec["height_m"][kind]
        mesh = Mesh()
        lathe(mesh, profile, radius, height)
        if kind == "n":
            head = spec["knight_head"]
            extrude(mesh, [(x * radius, z * height) for x, z in head["outline"]], head["thickness"] * radius)
        if kind == "k":
            cross = spec["king_cross"]
            w_up, h_up = cross["upright"]
            w_bar, h_bar = cross["bar"]
            box(mesh, (0, 0, cross["centre"] * height), (w_up * radius, w_up * radius, h_up * height))
            box(mesh, (0, 0, cross["bar_centre"] * height), (w_bar * radius, w_up * radius, h_bar * height))
        mesh.write(os.path.join(out_dir, f"{kind}.obj"))


if __name__ == "__main__":
    main()
