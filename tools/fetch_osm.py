#!/usr/bin/env python3
"""Fetch or reuse an OSMnx road graph and export a reproducible C++ CSV dataset.
Optional tool: requires `uv pip install osmnx` (and its transitive deps).
This is NOT invoked by run_all.sh: benchmark runs never contact a map server.
"""
import argparse
import csv
import datetime as dt
import json
from pathlib import Path
import random


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--graphml',type=Path,help='Use an existing GraphML instead of downloading')
    p.add_argument('--center-lat',type=float,default=24.804)
    p.add_argument('--center-lon',type=float,default=120.971)
    p.add_argument('--radius-m',type=int,default=8000)
    p.add_argument('--queries',type=int,default=20)
    p.add_argument('--seed',type=int,default=42)
    p.add_argument('--out',type=Path,default=Path(__file__).resolve().parents[1]/'data'/'osm')
    a=p.parse_args()
    if a.queries<1: p.error('--queries must be positive')
    try:
        import osmnx as ox
        import networkx as nx
    except ImportError as exc:
        p.error('OSMnx is required: `uv venv && uv pip install osmnx`')
    a.out.mkdir(exist_ok=True,parents=True)
    if a.graphml:
        graph=ox.io.load_graphml(a.graphml)
        source=str(a.graphml)
    else:
        graph=ox.graph.graph_from_point((a.center_lat,a.center_lon),dist=a.radius_m,
                                        network_type='drive',simplify=True)
        ox.io.save_graphml(graph,a.out/'hsinchu.graphml')
        source=f'OSMnx point ({a.center_lat}, {a.center_lon}), bbox radius {a.radius_m} m'
    ids={node:idx for idx,node in enumerate(graph.nodes)}
    with (a.out/'nodes.csv').open('w',encoding='utf-8',newline='') as f:
        w=csv.writer(f);w.writerow(['id','lat','lon'])
        for old, data in graph.nodes(data=True): w.writerow([ids[old],data['y'],data['x']])
    with (a.out/'edges.csv').open('w',encoding='utf-8',newline='') as f:
        w=csv.writer(f);w.writerow(['from','to','weight'])
        for u,v,data in graph.edges(data=True):w.writerow([ids[u],ids[v],float(data['length'])])
    components=list(nx.strongly_connected_components(graph))
    largest=max(components,key=len)
    rng=random.Random(a.seed);vertices=sorted(largest)
    if len(vertices)<2:raise RuntimeError('largest strongly connected component contains <2 vertices')
    targets=set();candidates=[]
    for _ in range(max(200,30*a.queries)):
        s,t=rng.sample(vertices,2)
        if (s,t) in targets:continue
        targets.add((s,t))
        distance=nx.shortest_path_length(graph,s,t,weight='length')
        candidates.append((distance,s,t))
    candidates.sort()
    selected=[];seen=set()
    for i in range(a.queries):
        index=round((i+0.5)*len(candidates)/a.queries-0.5)
        index=max(0,min(len(candidates)-1,index))
        _,s,t=candidates[index]
        if (s,t) in seen:continue
        seen.add((s,t));selected.append((ids[s],ids[t]))
    with (a.out/'queries.csv').open('w',encoding='utf-8',newline='') as f:
        w=csv.writer(f);w.writerow(['start','goal']);w.writerows(selected)
    metadata={'created_at_utc':dt.datetime.now(dt.timezone.utc).isoformat(),
              'source':source,'nodes':graph.number_of_nodes(),'directed_edges':graph.number_of_edges(),
              'queries':len(selected),'seed':a.seed,'network_type':'drive',
              'attribution':'© OpenStreetMap contributors (ODbL)'}
    (a.out/'dataset.json').write_text(json.dumps(metadata,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps(metadata,indent=2,ensure_ascii=False))

if __name__=='__main__':main()
