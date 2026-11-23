
import os
import osmnx as ox

os.makedirs("data/osm", exist_ok=True)

# Hsinchu City center (latitude, longitude)
CENTER = (24.804, 120.971)

# Download roads within approximately 8 km of the center.
G = ox.graph.graph_from_point(
    center_point=CENTER,
    dist=8000,
    dist_type="bbox",
    network_type="drive",
    simplify=True,
)

# Save the road network.
output = "data/osm/hsinchu.graphml"
ox.io.save_graphml(G, output)

print("Nodes:", len(G.nodes))
print("Edges:", len(G.edges))
print("Saved to:", output)
