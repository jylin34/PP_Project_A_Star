
import csv
import random
from pathlib import Path

import networkx as nx
import osmnx as ox

OUT = Path("data/osm")
G = ox.io.load_graphml(OUT / "hsinchu.graphml")

# Map OSM node IDs to continuous integer IDs.
node_ids = {node: i for i, node in enumerate(G.nodes)}

with (OUT / "nodes.csv").open("w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["id", "lat", "lon"])

    for node, data in G.nodes(data=True):
        writer.writerow([
            node_ids[node],
            data["y"],
            data["x"],
        ])

with (OUT / "edges.csv").open("w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["from", "to", "weight"])

    for u, v, data in G.edges(data=True):
        writer.writerow([
            node_ids[u],
            node_ids[v],
            float(data["length"]),
        ])

# Choose start-goal pairs from the largest strongly
# connected component to guarantee reachable routes.
largest_scc = max(nx.strongly_connected_components(G), key=len)
candidates = sorted(node_ids[node] for node in largest_scc)

rng = random.Random(42)
queries = set()

while len(queries) < min(20, len(candidates) * (len(candidates) - 1)):
    start, goal = rng.sample(candidates, 2)
    queries.add((start, goal))

with (OUT / "queries.csv").open("w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["start", "goal"])

    for start, goal in sorted(queries):
        writer.writerow([start, goal])

print(f"Nodes: {G.number_of_nodes()}")
print(f"Edges: {G.number_of_edges()}")
print(f"Queries: {len(queries)}")
print("Conversion finished.")
