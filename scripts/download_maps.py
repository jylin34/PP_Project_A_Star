#!/usr/bin/env python3
"""Download OpenStreetMap data for the project.

  hsinchu: drivable road graph of Hsinchu City via OSMnx -> data/hsinchu.graphml
  taiwan:  raw OSM extract from Geofabrik            -> data/taiwan-latest.osm.pbf

Usage:
  python scripts/download_maps.py            # both
  python scripts/download_maps.py hsinchu    # only Hsinchu
  python scripts/download_maps.py taiwan     # only Taiwan
  add --force to re-download existing files
"""
import argparse
import os
import sys
import urllib.request

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")
TAIWAN_URL = "https://download.geofabrik.de/asia/taiwan-latest.osm.pbf"


def download_hsinchu(force: bool) -> None:
    out = os.path.join(DATA_DIR, "hsinchu.graphml")
    if os.path.exists(out) and not force:
        print(f"[hsinchu] exists, skipping: {out}")
        return
    import osmnx as ox

    print("[hsinchu] downloading road graph via OSMnx ...")
    G = ox.graph_from_place("Hsinchu, Taiwan", network_type="drive")
    ox.save_graphml(G, out)
    print(f"[hsinchu] {len(G.nodes)} nodes, {len(G.edges)} edges -> {out}")


def download_taiwan(force: bool) -> None:
    out = os.path.join(DATA_DIR, "taiwan-latest.osm.pbf")
    if os.path.exists(out) and not force:
        print(f"[taiwan] exists, skipping: {out}")
        return
    tmp = out + ".part"
    print(f"[taiwan] downloading {TAIWAN_URL} ...")

    def progress(blocks, block_size, total):
        done = blocks * block_size
        if total > 0:
            pct = min(100.0, done * 100 / total)
            sys.stdout.write(f"\r[taiwan] {done / 1e6:.0f}/{total / 1e6:.0f} MB ({pct:.0f}%)")
            sys.stdout.flush()

    urllib.request.urlretrieve(TAIWAN_URL, tmp, reporthook=progress)
    os.replace(tmp, out)  # only keep complete downloads
    print(f"\n[taiwan] saved -> {out}")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("maps", nargs="*", choices=["hsinchu", "taiwan"], default=["hsinchu", "taiwan"])
    p.add_argument("--force", action="store_true", help="re-download existing files")
    args = p.parse_args()

    os.makedirs(DATA_DIR, exist_ok=True)
    if "hsinchu" in args.maps:
        download_hsinchu(args.force)
    if "taiwan" in args.maps:
        download_taiwan(args.force)


if __name__ == "__main__":
    main()
