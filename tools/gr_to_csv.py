#!/usr/bin/env python3
"""Convert a .gr road graph (from scripts/convert_maps.py) into the CSV dataset
read by the C++ code: nodes.csv (id,lat,lon), edges.csv (from,to,weight) and
queries.csv (start,goal).

.gr input format:
  # comment lines
  n m
  x y        <- n lines, UTM zone 51N metres (EPSG:32651)
  u v w      <- m lines, directed edge with weight in metres

The .gr graph is already restricted to its largest strongly connected
component, so every sampled (start, goal) pair is reachable.

Queries: sample many random vertex pairs, sort them by straight-line distance,
and pick evenly spaced ones so the set covers short, medium and long trips.
(Straight-line distance is used instead of exact shortest-path distance,
which would be too slow to compute on millions of vertices in Python.)

Usage (needs pyproj + numpy, e.g. from .venv):
  .venv/bin/python tools/gr_to_csv.py data/taiwan.gr --out data/taiwan --queries 20
"""
import argparse
import csv
import json
import math
import random
from pathlib import Path

import numpy as np
from pyproj import Transformer

TO_LONLAT = Transformer.from_crs("EPSG:32651", "EPSG:4326", always_xy=True)


def read_gr(path):
    with open(path, encoding="utf-8") as f:
        line = f.readline()
        while line.startswith("#"):
            line = f.readline()
        n, m = map(int, line.split())

        xy = np.empty((n, 2))
        for i in range(n):
            x, y = f.readline().split()
            xy[i] = (float(x), float(y))

        edges = []
        for _ in range(m):
            u, v, w = f.readline().split()
            edges.append((int(u), int(v), w))  # keep weight text as-is
    return xy, edges


def pick_queries(xy, count, seed):
    rng = random.Random(seed)
    n = len(xy)
    candidates = set()
    while len(candidates) < max(200, 30 * count):
        s, t = rng.sample(range(n), 2)
        candidates.add((s, t))

    def distance(pair):
        s, t = pair
        return math.dist(xy[s], xy[t])

    ordered = sorted(candidates, key=distance)
    selected = []
    for i in range(count):
        index = round((i + 0.5) * len(ordered) / count - 0.5)
        selected.append(ordered[max(0, min(len(ordered) - 1, index))])
    return selected


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("graph", type=Path, help="input .gr file")
    p.add_argument("--out", type=Path, required=True, help="output directory")
    p.add_argument("--queries", type=int, default=20)
    p.add_argument("--seed", type=int, default=42)
    a = p.parse_args()

    xy, edges = read_gr(a.graph)
    lon, lat = TO_LONLAT.transform(xy[:, 0], xy[:, 1])

    a.out.mkdir(parents=True, exist_ok=True)
    with (a.out / "nodes.csv").open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["id", "lat", "lon"])
        for i in range(len(xy)):
            w.writerow([i, f"{lat[i]:.7f}", f"{lon[i]:.7f}"])

    with (a.out / "edges.csv").open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["from", "to", "weight"])
        w.writerows(edges)

    queries = pick_queries(xy, a.queries, a.seed)
    with (a.out / "queries.csv").open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["start", "goal"])
        w.writerows(queries)

    meta = {
        "source": str(a.graph),
        "nodes": len(xy),
        "directed_edges": len(edges),
        "queries": len(queries),
        "seed": a.seed,
        "query_selection": "random pairs stratified by straight-line distance",
        "attribution": "© OpenStreetMap contributors (ODbL)",
    }
    (a.out / "dataset.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(meta, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
