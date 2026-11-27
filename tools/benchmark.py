#!/usr/bin/env python3
"""Reproducible graph-search benchmarks using only the Python standard library.
The C++ executable validates every run against Dijkstra outside measured search time.
"""
import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import random
import statistics
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
EXE = ROOT / 'build' / 'parallel_astar'


def med(x):
    return statistics.median(x) if x else None


def opt_median(rows, key):
    vals = [float(row[key]) for row in rows if row.get(key) is not None and row.get(key) != '']
    return med(vals)


def csv_queries(path, limit):
    with path.open(newline='', encoding='utf-8-sig') as stream:
        reader = csv.DictReader(stream)
        if not {'start', 'goal'}.issubset(reader.fieldnames or []):
            raise ValueError('queries CSV needs start,goal header')
        out = []
        for item in reader:
            s, t = int(item['start']), int(item['goal'])
            if s < 0 or t < 0:
                raise ValueError('vertex IDs must be >= 0')
            out.append((s, t))
        if not out:
            raise ValueError('no queries found in ' + str(path))
        if len(out) <= limit:
            return out
        # Query files are usually sorted short -> long, so taking the first N
        # would only test short trips. Pick N evenly spaced queries instead.
        return [out[round(i * (len(out) - 1) / max(1, limit - 1))] for i in range(limit)] if limit > 1 else [out[len(out) // 2]]


def grid_queries(n, limit):
    def v(y, x): return y * n + x
    p = [
        (v(0, 0), v(n-1, n-1)),
        (v(0, n-1), v(n-1, 0)),
        (v(n//2, 0), v(n//2, n-1)),
        (v(0, n//2), v(n-1, n//2)),
        (v(n//4, n//4), v((3*n)//4, (3*n)//4)),
        (v(n//4, (3*n)//4), v((3*n)//4, n//4)),
        (v(0, 0), v(n//2, n//2)),
        (v(n//2, n//2), v(n-1, n-1)),
        (v(n//4, n//4), v(n//4, (3*n)//4)),
        (v((3*n)//4, n//4), v((3*n)//4, (3*n)//4)),
    ]
    return list(dict.fromkeys(p))[:limit]


def grid_workloads(args):
    for n in args.sizes:
        for seed in args.seeds:
            for qid, (start, goal) in enumerate(grid_queries(n, args.max_grid_queries)):
                yield {
                    'dataset': f'grid_{n}_s{seed}', 'family': 'grid',
                    'query_id': qid, 'start': start, 'goal': goal,
                    'grid_size': n, 'seed': seed,
                    'graph_options': ['--grid', str(n), '--seed', str(seed), '--density', str(args.density)],
                }


def osm_workloads(args):
    missing = [p for p in (args.nodes, args.edges, args.queries) if not p.is_file()]
    if missing:
        raise ValueError('Missing OSM CSV: ' + ', '.join(map(str, missing)) + '\n'
                         'Copy nodes.csv, edges.csv, queries.csv to data/osm/ first.')
    for qid, (start, goal) in enumerate(csv_queries(args.queries, args.max_queries)):
        yield {
            'dataset': args.osm_name, 'family': 'osm', 'query_id': qid,
            'start': start, 'goal': goal, 'grid_size': 0, 'seed': 0,
            'graph_options': ['--nodes', str(args.nodes), '--edges', str(args.edges)],
        }


def variant_modes(args):
    # These modes are repeated on the same graph/query, with equal verification.
    modes = [('dijkstra', 1, '-'), ('sequential', 1, '-'), ('bidirectional', 1, '-'), ('bidir_parallel', 2, '-')]
    for p in args.threads:
        modes.append(('shared', p, '-'))
        for h in args.hash_policies:
            modes.append(('hda', p, h))
    return modes


def run_one(job, mode, repetition, args):
    alg, threads, policy = mode
    cmd = [str(EXE), '--algorithm', alg, '--threads', str(threads),
           '--start', str(job['start']), '--goal', str(job['goal']),
           '--verify', '1' if args.verify else '0',
           '--prune', '1' if args.prune else '0',
           '--profile-heuristic', '1' if args.profile_heuristic else '0',
           '--tile', str(args.tile), '--batch', str(args.batch), '--flush-interval', str(args.flush_interval)] + job['graph_options']
    if alg == 'hda':
        cmd += ['--hash', policy]
    try:
        output = subprocess.run(cmd, capture_output=True, text=True, timeout=args.timeout)
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError('TIMEOUT: ' + ' '.join(cmd)) from exc
    if output.returncode:
        raise RuntimeError('Search or correctness failed:\n' + ' '.join(cmd) + '\n' + output.stdout + '\n' + output.stderr)
    record = json.loads(output.stdout)
    if args.verify and not record['verified']:
        raise RuntimeError('executable did not verify an expected verified run')
    record.pop('path', None)  # keep CSV small; path itself was validated in C++
    record.update(dataset=job['dataset'], dataset_family=job['family'], query_id=job['query_id'],
                  start=job['start'], goal=job['goal'], grid_size=job['grid_size'], seed=job['seed'],
                  density=args.density if job['family'] == 'grid' else '', repeat_id=repetition)
    for key in ('per_worker_expanded', 'per_worker_generated', 'per_worker_assigned'):
        record[key] = json.dumps(record[key], separators=(',', ':'))
    return record


def write_csv(path, rows):
    if not rows:
        raise ValueError('no benchmark rows')
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()), extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


# Algorithms whose P=1 scaling baseline is a different (single-thread) algorithm.
SCALING_BASE = {'bidir_parallel': 'bidirectional'}


def summarize(rows, out_dir):
    # Establish baseline for EACH query before aggregating across unrelated queries.
    groups = {}
    for r in rows:
        key = (r['dataset'], r['query_id'], r['algorithm'], r['threads'], r['hash'])
        groups.setdefault(key, []).append(r)
    baseline = {}
    for (dataset, query, algorithm, p, policy), data in groups.items():
        if algorithm == 'sequential':
            baseline[(dataset, query)] = (opt_median(data, 'time_ms'), opt_median(data, 'expanded_nodes'))
    scaling = {}
    for (dataset, query, algorithm, p, policy), data in groups.items():
        if p == 1:
            scaling[(dataset, query, algorithm, policy)] = opt_median(data, 'time_ms')
    per_query = []
    for (dataset, query, algorithm, p, policy), data in sorted(groups.items()):
        b_time, b_exp = baseline[(dataset, query)]
        m_time = opt_median(data, 'time_ms')
        m_exp = opt_median(data, 'expanded_nodes')
        p1 = scaling.get((dataset, query, SCALING_BASE.get(algorithm, algorithm), policy))
        per_query.append({
            'dataset': dataset, 'family': data[0]['dataset_family'], 'query_id': query,
            'algorithm': algorithm, 'threads': p, 'hash': policy,
            'n': len(data), 'found': data[0]['found'],
            'median_ms': m_time, 'median_expanded': m_exp,
            'median_generated': opt_median(data, 'generated_nodes'),
            'median_remote': opt_median(data, 'remote_messages'),
            'median_remote_batches': opt_median(data, 'remote_batches'),
            'communication_overhead': opt_median(data, 'communication_overhead'),
            'load_balance': opt_median(data, 'load_balance'),
            'load_balance_expanded': opt_median(data, 'load_balance_expanded'),
            'median_assigned': opt_median(data, 'assigned_nodes'),
            'heuristic_calls': opt_median(data, 'heuristic_calls'),
            'heuristic_ms': opt_median(data, 'heuristic_ms'),
            'heuristic_share': opt_median(data, 'heuristic_share'),
            'search_overhead': m_exp / b_exp - 1 if b_exp and b_exp > 0 else None,
            'speedup_vs_sequential': b_time / m_time if m_time > 0 else None,
            'scaling_speedup': p1 / m_time if p1 is not None and m_time > 0 else None,
            'scaling_efficiency': p1 / (p*m_time) if p1 is not None and m_time > 0 else None,
        })
    write_csv(out_dir / 'per_query_summary.csv', per_query)
    cross = {}
    for r in per_query:
        key = (r['dataset'], r['algorithm'], r['threads'], r['hash'])
        cross.setdefault(key, []).append(r)
    summary = []
    for (dataset, algorithm, p, policy), data in sorted(cross.items()):
        item = {'dataset': dataset, 'family': data[0]['family'], 'algorithm': algorithm,
                'threads': p, 'hash': policy, 'queries': len(data)}
        for metric in ('median_ms', 'median_expanded', 'median_generated', 'median_remote',
                       'median_remote_batches', 'communication_overhead', 'load_balance', 'load_balance_expanded', 'median_assigned',
                       'heuristic_calls', 'heuristic_ms', 'heuristic_share',
                       'search_overhead', 'speedup_vs_sequential', 'scaling_speedup', 'scaling_efficiency'):
            item[metric] = opt_median(data, metric)
        summary.append(item)
    write_csv(out_dir / 'summary.csv', summary)
    return summary


def sha256(path):
    if not path.is_file(): return None
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', choices=['grid','osm','both'], default='grid')
    parser.add_argument('--sizes', nargs='+', type=int, default=[32,64])
    parser.add_argument('--seeds', nargs='+', type=int, default=[7,42])
    parser.add_argument('--density', type=float, default=.20)
    parser.add_argument('--threads', nargs='+', type=int, default=[1,2,4])
    parser.add_argument('--hash-policies', nargs='+', choices=['mod','mixed','zobrist','abstract','azobrist'], default=['zobrist','azobrist'])
    parser.add_argument('--tile', type=int, default=8, help='spatial abstraction tile (grid cells, or 100m road cells)')
    parser.add_argument('--batch', type=int, default=32, help='per-worker inbox batch and message batch cap')
    parser.add_argument('--flush-interval', type=int, default=8, help='maximum expansions before sender outboxes flush')
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--max-grid-queries', type=int, default=3)
    parser.add_argument('--max-queries', type=int, default=20)
    parser.add_argument('--nodes', type=Path, default=ROOT/'data/osm/nodes.csv')
    parser.add_argument('--edges', type=Path, default=ROOT/'data/osm/edges.csv')
    parser.add_argument('--queries', type=Path, default=ROOT/'data/osm/queries.csv')
    parser.add_argument('--osm-name', default='hsinchu_osm', help='dataset label for OSM results/plots')
    parser.add_argument('--out', type=Path, default=ROOT/'results'/'latest')
    parser.add_argument('--timeout', type=float, default=180)
    parser.add_argument('--no-plots', action='store_true')
    parser.add_argument('--no-verify', dest='verify', action='store_false', help='NOT recommended for final experiments')
    parser.add_argument('--no-prune', dest='prune', action='store_false')
    parser.add_argument('--no-shuffle', action='store_true')
    parser.add_argument('--profile-heuristic', action='store_true',
                        help='time every heuristic() call (adds timer overhead; run as a separate experiment)')
    parser.add_argument('--resume', action='store_true', help='reuse validated runs.csv after an interruption; reject changed config/input')
    args = parser.parse_args()
    if not EXE.is_file(): parser.error('missing executable: run ./run_all.sh first')
    if min(args.threads) < 1 or args.repeats < 1 or args.max_queries < 1 or args.max_grid_queries < 1 or args.tile < 1 or args.flush_interval < 1:
        parser.error('thread/repeat/query/tile values must be positive')
    if not 0 <= args.density < 1: parser.error('density must be in [0,1)')
    args.out.mkdir(parents=True, exist_ok=True)
    try:
        tasks = list(grid_workloads(args) if args.dataset=='grid' else [])
        if args.dataset=='both': tasks += list(grid_workloads(args))
        if args.dataset in ('osm','both'): tasks += list(osm_workloads(args))
    except ValueError as exc:
        parser.error(str(exc))
    modes = variant_modes(args)
    jobs = [(task, mode, repeat) for task in tasks for repeat in range(args.repeats) for mode in modes]
    if not args.no_shuffle: random.Random(20261123).shuffle(jobs)
    # Crash-safe incremental CSV: every completed run is saved immediately.
    # A resume refuses to mix different parameters, input snapshots or binaries.
    signature = {
        'parameters': {k:(str(v) if isinstance(v,Path) else v)
                       for k,v in vars(args).items()
                       if k not in ('out','resume','no_plots','timeout','no_shuffle')},
        'binary_sha256': sha256(EXE),
        'nodes_sha256': sha256(args.nodes) if args.dataset!='grid' else None,
        'edges_sha256': sha256(args.edges) if args.dataset!='grid' else None,
        'queries_sha256': sha256(args.queries) if args.dataset!='grid' else None,
    }
    configfile = args.out/'experiment_config.json'
    outfile = args.out/'runs.csv'
    if args.resume:
        if not configfile.exists() or not outfile.exists():
            parser.error('--resume requires existing runs.csv and experiment_config.json in --out')
        saved = json.loads(configfile.read_text(encoding='utf-8'))
        if saved != signature:
            parser.error('resume refused: parameters, executable binary or dataset SHA changed')
        with outfile.open(newline='',encoding='utf-8') as f:
            rows=list(csv.DictReader(f))
    else:
        configfile.write_text(json.dumps(signature,sort_keys=True,indent=2),encoding='utf-8')
        rows=[]

    def row_key(row):
        return (row['dataset'],str(row['query_id']),row['algorithm'],
                str(row['threads']),row['hash'],str(row['repeat_id']))
    seen={row_key(r) for r in rows}
    mode = 'a' if args.resume else 'w'
    with outfile.open(mode,encoding='utf-8',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0].keys()) if rows else [])
        wrote_header=bool(rows)
        for count,(task, variant, rep) in enumerate(jobs,1):
            alg,p,h=variant
            key=(task['dataset'],str(task['query_id']),alg,str(p),h,str(rep))
            if key in seen:
                continue
            print(f'[{count}/{len(jobs)}] {task["dataset"]} q{task["query_id"]} {alg} P={p} hash={h} run={rep}',flush=True)
            row=run_one(task,variant,rep,args)
            if not wrote_header:
                writer=csv.DictWriter(stream,fieldnames=list(row))
                writer.writeheader();wrote_header=True
            writer.writerow(row);stream.flush()
            rows.append(row);seen.add(key)
    if len(rows)!=len(jobs):
        raise RuntimeError(f'incomplete run matrix: {len(rows)} of {len(jobs)}')
    summary = summarize(rows, args.out)
    try:
        git = subprocess.run(['git','rev-parse','HEAD'],cwd=ROOT,text=True,capture_output=True)
        revision = git.stdout.strip() if git.returncode==0 else None
    except FileNotFoundError: revision=None
    manifest = {
        'timestamp_utc': dt.datetime.now(dt.timezone.utc).isoformat(),
        'platform': platform.platform(), 'python': sys.version, 'git_revision': revision,
        'compiler_info': subprocess.run(['c++','--version'],text=True,capture_output=True).stdout.splitlines()[0],
        'cpu_count': os.cpu_count(), 'command': sys.argv,
        'total_runs': len(rows), 'distinct_queries': len(tasks),
        'dataset': args.dataset, 'grid_sizes': args.sizes, 'seeds': args.seeds,
        'grid_density': args.density, 'threads': args.threads, 'hash_policies': args.hash_policies,
        'tile': args.tile, 'batch': args.batch, 'flush_interval': args.flush_interval, 'repeats': args.repeats,
        'verify_each_run': args.verify, 'pruning': args.prune,
        'nodes_sha256': sha256(args.nodes) if args.dataset != 'grid' else None,
        'edges_sha256': sha256(args.edges) if args.dataset != 'grid' else None,
        'queries_sha256': sha256(args.queries) if args.dataset != 'grid' else None,
        'metric_definitions': {
            'CO': 'remote candidate messages / generated successor edges',
            'SO': 'expanded_parallel / expanded_sequential - 1 (same query)',
            'LB': 'for HDA*: max owner-assigned candidate nodes / mean owner-assigned candidate nodes; for Shared-Queue: expansion imbalance',
            'LB_expanded': 'max worker expansions / mean worker expansions',
            'scaling_speedup': 'median same-algorithm P=1 time / median P-thread time, per-query then median',
            'speedup_vs_sequential': 'median sequential time / median algorithm time, per-query then median',
            'time_ms': 'wall-clock duration inside search method; includes thread creation and join, excludes graph I/O and verification',
        },
    }
    (args.out/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False), encoding='utf-8')
    subprocess.run([sys.executable,str(ROOT/'tools'/'report.py'),
                    '--csv',str(args.out/'summary.csv'),'--out',str(args.out/'REPORT.md')],check=True)
    if not args.no_plots:
        cmd=[sys.executable,str(ROOT/'tools'/'plot_results.py'),'--csv',str(args.out/'summary.csv'),'--out',str(args.out)]
        subprocess.run(cmd,check=True)
    print(f'\nSUCCESS: {len(rows)} validated runs, {len(summary)} summaries -> {args.out}',flush=True)

if __name__=='__main__':main()
