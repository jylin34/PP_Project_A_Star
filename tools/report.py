#!/usr/bin/env python3
"""Create a reproducible Markdown result digest from experiment summary.csv."""
import argparse
import csv
from pathlib import Path

def fmt(value, digits=3):
    if value in ('',None): return '—'
    try: return f'{float(value):.{digits}f}'
    except (TypeError,ValueError): return str(value)

def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument('--csv',type=Path,required=True)
    a.add_argument('--out',type=Path,required=True)
    args=a.parse_args()
    with args.csv.open(newline='',encoding='utf-8') as f: rows=list(csv.DictReader(f))
    parts=[
        '# Experiment Result Digest',
        '',
        '> This document is generated automatically from `summary.csv`. These results do not constitute a proven speedup or published replication of the cited HDA* work.',
        '',
        'The reported median of speedup ratios is computed **per query before aggregation**. Search timing includes pthread creation/join but excludes graph loading and reference verification.',
        '',
    ]
    datasets=sorted({row['dataset'] for row in rows})
    for name in datasets:
        parts.extend(['## '+name,'',
            '| Algorithm | P | Hash | Median ms | Speedup vs seq | Scaling S(P) | Expansions | CO | SO | LB |',
            '|:--|--:|:--|--:|--:|--:|--:|--:|--:|--:|'])
        for row in [r for r in rows if r['dataset']==name]:
            parts.append('| '+ ' | '.join([
                row['algorithm'],str(row['threads']),row['hash'],
                fmt(row['median_ms']),fmt(row['speedup_vs_sequential']),
                fmt(row['scaling_speedup']),fmt(row['median_expanded'],0),
                fmt(row['communication_overhead']),fmt(row['search_overhead']),fmt(row['load_balance']),
            ])+' |')
        parts.append('')
    parts.extend([
        '## Interpretation checklist', '',
        '- Compare **communication overhead (CO)** and **search overhead (SO)** for each hash policy, not runtime alone.',
        '- Check **load balance** against increased expansions and wall time; a low CO strategy may shift work unevenly.',
        '- Report reachable and unreachable queries separately when making conclusions.',
        '- For scheduling/lock-contention conclusions, collect **direct profiling**; condition-variable sleeping time is not lock wait time.',
        '- Repeat timing on the target Linux machine (not just WSL2) and record CPU affinity, load, hardware, compiler and commit.',
        '- Distinguish a graph-specific **spatial AZH-inspired policy** from the paper’s domain-specific Abstract Zobrist feature projection.',
        '',
    ])
    args.out.write_text('\n'.join(parts),encoding='utf-8')
    print('Generated Markdown summary:',args.out)
if __name__=='__main__': main()
