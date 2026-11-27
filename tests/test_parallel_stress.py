#!/usr/bin/env python3
"""Directed graph + HDA batching/termination smoke regression. No pip dependencies."""
import csv
import json
import random
import subprocess
import sys
import tempfile
from pathlib import Path

EXE = Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parents[1]/'build'/'parallel_astar'

def write_csv(path,head,rows):
    with path.open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(head);writer.writerows(rows)

def main():
    count=0
    with tempfile.TemporaryDirectory() as temp:
        folder=Path(temp)
        nodes=folder/'nodes.csv';edges=folder/'edges.csv'
        for seed in (7,42,91,2026):
            rng=random.Random(seed);n=22
            write_csv(nodes,['id','lat','lon'],[(i,24.8+i*0.00001,120.97+(i%7)*0.00001) for i in range(n)])
            directed=[(i,(i+1)%(n-1),1+(i%4)) for i in range(n-1)]
            for _ in range(95):
                u=rng.randrange(n-1);v=rng.randrange(n-1)
                if u!=v:directed.append((u,v,rng.randint(0,20)))
            # vertex n-1 has no in/out edges: test unreachable and s==t.
            write_csv(edges,['from','to','weight'],directed)
            queries=[(0,11),(11,0),(4,15),(0,n-1),(n-1,0),(0,0)]
            for s,t in queries:
                modes=[('sequential',1,'mixed',4,8),('bidirectional',1,'mixed',4,8),('bidir_parallel',2,'mixed',4,8)]
                for p in (1,2,4,8):
                    modes.append(('shared',p,'mixed',4,8))
                    for policy in ('mixed','mod','zobrist','abstract','azobrist'):
                        for batch,interval in ((1,1),(4,8),(16,32)):
                            modes.append(('hda',p,policy,batch,interval))
                for algorithm,p,policy,batch,interval in modes:
                    cmd=[str(EXE),'--algorithm',algorithm,'--nodes',str(nodes),'--edges',str(edges),
                         '--start',str(s),'--goal',str(t),'--verify','1',
                         '--threads',str(p),'--hash',policy,'--batch',str(batch),
                         '--flush-interval',str(interval),'--tile','4']
                    try:
                        result=subprocess.run(cmd,capture_output=True,text=True,timeout=5)
                    except subprocess.TimeoutExpired as exc:
                        raise RuntimeError('deadlock or timeout on: '+' '.join(cmd)) from exc
                    if result.returncode:
                        raise RuntimeError('failed: '+' '.join(cmd)+'\n'+result.stderr+'\n'+result.stdout)
                    data=json.loads(result.stdout)
                    if not data['verified']:
                        raise RuntimeError('run did not compare against Dijkstra')
                    count+=1
    print(f'PASS {count} directed graph / thread / hash / batch / termination validations')

if __name__=='__main__':main()
