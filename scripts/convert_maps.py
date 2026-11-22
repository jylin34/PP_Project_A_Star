#!/usr/bin/env python3
"""Convert downloaded OSM maps into a simple text graph for the C++ code.

  hsinchu: data/hsinchu.graphml        -> data/hsinchu.gr
  taiwan:  data/taiwan-latest.osm.pbf  -> data/taiwan.gr

Usage:
  python scripts/convert_maps.py            # both
  python scripts/convert_maps.py hsinchu    # one of them

Output format (.gr, plain text, directed graph, vertex ids 0..n-1):
  # comment lines start with '#'
  n m
  x y          <- n lines: vertex coordinates in metres (UTM zone 51N)
  u v w        <- m lines: directed edge u -> v with weight w (metres)

Guarantees:
  * Only the largest strongly connected component is kept, so every
    start/goal pair has a path.
  * w >= Euclidean distance between the written coordinates of u and v,
    so h(v) = Euclidean distance to the goal is admissible and consistent.
  * Parallel edges are merged (minimum weight kept); self-loops removed.
"""
import argparse
import math
import os
import sys

import numpy as np
from pyproj import Transformer
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")
TO_UTM = Transformer.from_crs("EPSG:4326", "EPSG:32651", always_xy=True)  # lon/lat -> metres

# Road types treated as drivable (similar to OSMnx network_type="drive").
DRIVE_HIGHWAYS = {
    "motorway", "motorway_link", "trunk", "trunk_link",
    "primary", "primary_link", "secondary", "secondary_link",
    "tertiary", "tertiary_link", "unclassified", "residential",
    "living_street", "road",
}


# --------------------------------------------------------------------------- #
# Loaders: each returns (lon[], lat[], edges) where edges = [(u, v, w_or_None)]
# w is a polyline length in metres if known, else None (straight segment).
# --------------------------------------------------------------------------- #
def load_graphml(path):
    import osmnx as ox

    G = ox.load_graphml(path)
    nodes = list(G.nodes)
    idx = {n: i for i, n in enumerate(nodes)}
    lon = np.array([G.nodes[n]["x"] for n in nodes])
    lat = np.array([G.nodes[n]["y"] for n in nodes])

    edges = []
    for u, v, data in G.edges(data=True):
        w = None
        geom = data.get("geometry")
        if geom is not None:  # curved road: measure its projected polyline
            xs, ys = TO_UTM.transform(*geom.xy)
            w = float(np.hypot(np.diff(xs), np.diff(ys)).sum())
        edges.append((idx[u], idx[v], w))
    return lon, lat, edges


def load_pbf(path):
    import osmium

    idx, lon, lat, edges = {}, [], [], []

    def vid(node):
        i = idx.get(node.ref)
        if i is None:
            i = idx[node.ref] = len(lon)
            lon.append(node.location.lon)
            lat.append(node.location.lat)
        return i

    fp = (osmium.FileProcessor(path, osmium.osm.NODE | osmium.osm.WAY)
          .with_locations()  # nodes are read only to cache their coordinates
          .with_filter(osmium.filter.EntityFilter(osmium.osm.WAY))
          .with_filter(osmium.filter.KeyFilter("highway")))
    count = 0
    for way in fp:
        t = way.tags
        if t.get("highway") not in DRIVE_HIGHWAYS:
            continue
        if t.get("area") == "yes" or t.get("access") in ("private", "no") \
                or t.get("motor_vehicle") == "no":
            continue
        oneway = t.get("oneway", "no")
        forward = oneway in ("yes", "true", "1") or t.get("junction") == "roundabout"
        backward = oneway == "-1"

        try:
            ids = [vid(n) for n in way.nodes]
        except osmium.InvalidLocationError:
            continue  # way references a node outside the extract
        for a, b in zip(ids, ids[1:]):
            if not backward:
                edges.append((a, b, None))
            if not forward:
                edges.append((b, a, None))

        count += 1
        if count % 200000 == 0:
            print(f"  ... {count} ways, {len(lon)} nodes", flush=True)
    return np.array(lon), np.array(lat), edges


# --------------------------------------------------------------------------- #
def build_and_write(lon, lat, edges, out, source):
    # Project and round coordinates to millimetres (what the C++ side will read).
    x, y = TO_UTM.transform(lon, lat)
    x, y = np.round(x, 3), np.round(y, 3)

    u = np.fromiter((e[0] for e in edges), dtype=np.int64, count=len(edges))
    v = np.fromiter((e[1] for e in edges), dtype=np.int64, count=len(edges))
    poly = np.fromiter((math.nan if e[2] is None else e[2] for e in edges),
                       dtype=np.float64, count=len(edges))

    # Weight >= straight-line distance of the *written* coordinates -> admissible h.
    straight = np.hypot(x[u] - x[v], y[u] - y[v])
    w = np.where(np.isnan(poly), straight, np.maximum(poly, straight))
    w = np.ceil(w * 1000 + 1e-6) / 1000  # round up to mm

    # Drop self-loops, merge parallel edges (keep min weight).
    keep = u != v
    u, v, w = u[keep], v[keep], w[keep]
    order = np.lexsort((w, v, u))
    u, v, w = u[order], v[order], w[order]
    first = np.ones(len(u), dtype=bool)
    first[1:] = (u[1:] != u[:-1]) | (v[1:] != v[:-1])
    u, v, w = u[first], v[first], w[first]

    # Keep only the largest strongly connected component.
    n = len(x)
    A = coo_matrix((np.ones(len(u)), (u, v)), shape=(n, n)).tocsr()
    _, label = connected_components(A, directed=True, connection="strong")
    big = np.bincount(label).argmax()
    in_big = label == big
    new_id = np.full(n, -1, dtype=np.int64)
    new_id[in_big] = np.arange(in_big.sum())
    keep = in_big[u] & in_big[v]
    u, v, w = new_id[u[keep]], new_id[v[keep]], w[keep]
    x, y = x[in_big], y[in_big]

    with open(out, "w") as f:
        f.write(f"# source: {source}\n")
        f.write("# coords: UTM zone 51N (EPSG:32651), metres; weights: metres\n")
        f.write(f"{len(x)} {len(u)}\n")
        np.savetxt(f, np.column_stack((x, y)), fmt="%.3f")
        np.savetxt(f, np.column_stack((u, v, w)), fmt=("%d", "%d", "%.3f"))
    print(f"  {n} -> {len(x)} nodes (largest SCC), {len(u)} edges -> {out}")


def convert(name, force):
    src, out = {
        "hsinchu": ("hsinchu.graphml", "hsinchu.gr"),
        "taiwan": ("taiwan-latest.osm.pbf", "taiwan.gr"),
    }[name]
    src, out = os.path.join(DATA_DIR, src), os.path.join(DATA_DIR, out)
    if not os.path.exists(src):
        sys.exit(f"[{name}] missing {src}; run `make maps` first")
    if os.path.exists(out) and not force:
        print(f"[{name}] exists, skipping: {out}")
        return
    print(f"[{name}] converting {src} ...", flush=True)
    lon, lat, edges = load_graphml(src) if name == "hsinchu" else load_pbf(src)
    build_and_write(lon, lat, edges, out, os.path.basename(src))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("maps", nargs="*", choices=["hsinchu", "taiwan"], default=["hsinchu", "taiwan"])
    p.add_argument("--force", action="store_true", help="overwrite existing .gr files")
    args = p.parse_args()
    for name in args.maps:
        convert(name, args.force)


if __name__ == "__main__":
    main()
