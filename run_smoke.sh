#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
./run_all.sh --dataset both --sizes 12 --seeds 42 --max-grid-queries 2 --threads 1 2 \
    --hash-policies zobrist azobrist --repeats 1 --max-queries 2 \
    --nodes data/osm/sample_nodes.csv --edges data/osm/sample_edges.csv \
    --queries data/osm/sample_queries.csv --out results/smoke "$@"
