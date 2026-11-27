#!/usr/bin/env python3
"""Dependency-free publication-friendly SVG charts from summary.csv."""
import argparse
import csv
import html
import math
from pathlib import Path

COLORS=['#1379b5','#c46b28','#359268','#9143a3','#b0485d','#6a7a28','#33495c']
METRICS=[
  ('median_ms','Search time (ms)','runtime'),
  ('speedup_vs_sequential','Speedup vs Sequential A*','speedup_vs_sequential'),
  ('scaling_speedup','Same-algorithm scaling speedup','scaling_speedup'),
  ('scaling_efficiency','Same-algorithm scaling efficiency','parallel_efficiency'),
  ('median_expanded','Expanded nodes','expanded'),
  ('communication_overhead','Communication overhead (CO)','communication_overhead'),
  ('search_overhead','Search overhead (SO)','search_overhead'),
  ('load_balance','Owner work distribution (LB)','load_balance'),
  ('load_balance_expanded','Expansion imbalance','expansion_imbalance'),
  ('heuristic_calls','Heuristic calls','heuristic_calls'),
  ('heuristic_ms','Heuristic time, summed over threads (ms)','heuristic_time'),
  ('heuristic_share','Heuristic time / search time','heuristic_share'),
]

def label(row):
    a=row['algorithm']
    if a=='bidir_parallel':a='bidirectional'  # P=2 point of the bidirectional line
    return a+((' / '+row['hash']) if a=='hda' else '')

def svg_chart(rows, metric, ylabel, title, output):
    series={}
    for row in rows:
        try:y=float(row[metric]);x=int(row['threads'])
        except (ValueError,KeyError,TypeError):continue
        if not math.isfinite(y):continue
        series.setdefault(label(row),[]).append((x,y))
    if not series:return
    width,height=900,540;left,right,top,bottom=105,210,78,90
    plot_w=width-left-right;plot_h=height-top-bottom
    xvalues=sorted({x for values in series.values() for x,_ in values})
    yvalues=[y for values in series.values() for _,y in values]
    ymin=min(0,min(yvalues));ymax=max(yvalues)
    if abs(ymax-ymin)<1e-14:ymax=ymin+1
    margin=.08*(ymax-ymin);ymin-=margin;ymax+=margin
    def xx(x):return left+plot_w*(xvalues.index(x)/max(1,len(xvalues)-1))
    def yy(y):return top+plot_h*(ymax-y)/(ymax-ymin)
    out=[f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" role="img">',
         '<rect width="100%" height="100%" fill="white"/>',
         f'<text x="{width/2}" y="35" font-size="21" font-family="sans-serif" text-anchor="middle" fill="#222">{html.escape(title)}</text>',
         f'<text x="{left+plot_w/2}" y="{height-24}" font-size="15" text-anchor="middle" font-family="sans-serif">Number of worker threads</text>',
         f'<text x="22" y="{top+plot_h/2}" font-size="14" transform="rotate(-90 22 {top+plot_h/2})" text-anchor="middle" font-family="sans-serif">{html.escape(ylabel)}</text>']
    for i in range(6):
        y=ymin+(ymax-ymin)*i/5
        py=yy(y)
        out.append(f'<line x1="{left}" y1="{py:.1f}" x2="{left+plot_w}" y2="{py:.1f}" stroke="#e3e3e3"/>')
        out.append(f'<text x="{left-9}" y="{py+5:.1f}" font-size="12" text-anchor="end" font-family="sans-serif">{y:.3g}</text>')
    for x in xvalues:
        out.append(f'<text x="{xx(x):.1f}" y="{top+plot_h+23}" font-size="13" text-anchor="middle" font-family="sans-serif">{x}</text>')
    for i,(name,values) in enumerate(sorted(series.items())):
        color=COLORS[i%len(COLORS)];values.sort()
        coords=' '.join(f'{xx(x):.2f},{yy(y):.2f}' for x,y in values)
        out.append(f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="2.5"/>')
        for x,y in values:out.append(f'<circle cx="{xx(x):.2f}" cy="{yy(y):.2f}" r="4" fill="{color}"/>')
        legend_y=top+24+i*26
        out.append(f'<line x1="{width-right+17}" y1="{legend_y}" x2="{width-right+36}" y2="{legend_y}" stroke="{color}" stroke-width="3"/>')
        out.append(f'<text x="{width-right+44}" y="{legend_y+5}" font-size="13" font-family="sans-serif">{html.escape(name)}</text>')
    out.append('</svg>')
    output.write_text('\n'.join(out),encoding='utf-8')

ALGO_COLORS={'dijkstra':'#888888','sequential':'#33495c','bidirectional':'#6a7a28','bidir_parallel':'#359268',
             'shared':'#c46b28','hda':'#1379b5'}

def svg_runtime_bars(rows, title, output):
    """Horizontal bar chart: actual median search time (ms) of every variant."""
    bars=[]
    for row in rows:
        try:y=float(row['median_ms'])
        except (ValueError,KeyError,TypeError):continue
        if not math.isfinite(y):continue
        a=row['algorithm']
        name=a+(' / '+row['hash'] if a=='hda' else '')+f"  (P={row['threads']})"
        bars.append((y,name,a))
    if not bars:return
    bars.sort()
    row_h=24;left,right,top,bottom=230,90,70,50
    width=900;plot_w=width-left-right;height=top+row_h*len(bars)+bottom
    ymax=max(y for y,_,_ in bars) or 1
    out=[f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" role="img">',
         '<rect width="100%" height="100%" fill="white"/>',
         f'<text x="{width/2}" y="35" font-size="21" font-family="sans-serif" text-anchor="middle" fill="#222">{html.escape(title)}</text>',
         f'<text x="{left+plot_w/2}" y="{height-15}" font-size="14" text-anchor="middle" font-family="sans-serif">Median search time (ms), lower is better</text>']
    for i,(y,name,a) in enumerate(bars):
        py=top+i*row_h;w=plot_w*y/ymax
        color=ALGO_COLORS.get(a,'#888')
        out.append(f'<text x="{left-8}" y="{py+row_h*0.68:.1f}" font-size="13" text-anchor="end" font-family="sans-serif">{html.escape(name)}</text>')
        out.append(f'<rect x="{left}" y="{py+3}" width="{max(w,1):.1f}" height="{row_h-6}" fill="{color}"/>')
        out.append(f'<text x="{left+w+6:.1f}" y="{py+row_h*0.68:.1f}" font-size="12" font-family="sans-serif" fill="#222">{y:.3g} ms</text>')
    out.append('</svg>')
    output.write_text('\n'.join(out),encoding='utf-8')

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--csv',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
    args=ap.parse_args();args.out.mkdir(exist_ok=True,parents=True)
    with args.csv.open(newline='',encoding='utf-8') as f:rows=list(csv.DictReader(f))
    by_dataset={}
    for r in rows:by_dataset.setdefault(r['dataset'],[]).append(r)
    count=0
    for dataset,data in by_dataset.items():
        profiled=any(float(r.get('heuristic_ms') or 0)>0 for r in data)
        for key,ylabel,suffix in METRICS:
            if key in ('heuristic_ms','heuristic_share') and not profiled:continue  # only with --profile-heuristic
            path=args.out / f'{dataset}_{suffix}.svg'
            svg_chart(data,key,ylabel,dataset+' / '+ylabel,path)
            count+=int(path.is_file())
        path=args.out / f'{dataset}_runtime_bars.svg'
        svg_runtime_bars(data,dataset+' / Search time per variant',path)
        count+=int(path.is_file())
    print(f'Generated {count} dependency-free SVG charts')
if __name__=='__main__':main()
