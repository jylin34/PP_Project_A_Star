# Experimental protocol

## Runs and reproducibility

- `run_all.sh` configures a Release binary, runs CTest, and invokes the runner.
- Runner verifies **every** execution against a separate Dijkstra search and the reported directed path; it fails immediately on mismatch.
- Each `.csv` includes dataset label, query ID, threads, hash, tile, batch, repeat ID, expanded/generated/remote/owner-assigned counts.
- The manifest includes UTC timestamp, OS, CPU count, compiler, arguments, and SHA-256 of road CSV input files.
- `REPORT.md` automatically summarizes the key research metrics in a Markdown table.
- Each run checkpoints to `runs.csv` immediately. `--resume` checks a saved experiment signature and input/binary SHA before accepting previously completed runs.
- Algorithm order is randomized with a fixed deterministic seed so each variant doesn't always benefit from cold/hot cache differences.
- `--no-shuffle` reproduces a conventional ordering, but should not be the default for benchmarking.

## Baselines and metrics

For each **individual query**, first take the median runtime across repeats. The per-query result is then normalized against the *same query* Sequential A* time or *same algorithm* P=1 time.

- `T_algorithm,p` is search wall clock, including pthread start/join, excluding dataset loading and Dijkstra verification.
- `S_seq = T_seq,1 / T_algorithm,p`: relative algorithm performance (can include reduced search work).
- `S_scale = T_algorithm,1 / T_algorithm,p`: same-algorithm strong scaling.
- `E_scale = S_scale / p`.
- `CO = remote_messages / generated_nodes`: share of generated successor edges that resulted in a remote candidate transfer; pruned edges remain in the generation denominator.
- `SO = expanded_algorithm / expanded_sequential - 1`: search overhead (can be negative for Bidirectional A*).
- `LB = max(owner assigned candidates) / mean(owner assigned candidates)` for HDA*. An assigned candidate is an initial task, an incoming message (including duplicates), or a locally generated successor routed to its owner. `load_balance_expanded` separately measures max/mean node expansions.

The summary takes **median per-query ratios**. It is not `median(T_seq)/median(T_parallel)` across unrelated queries.

## Dataset recommendations

- Development: 12x12 and a 7-node directed sample graph (built-in).
- Preliminary: Grid sizes 128, 256, 512; densities 0.1 and 0.2; at least 3 seeds; multiple query distances per map.
- OSM: use the fixed CSV snapshot of Hsinchu. Preserve one-way edges, use metres for weights and a globally consistent scaled geographic heuristic.
- For no-path cases, report separately from reachable queries. A no-path query has `path_cost=null` and `found=false`.
- Avoid mixing unrelated query sizes into one raw runtime average without per-query normalization.

## Threading and timing limitations

- Pthreads create/join overhead included. For tiny maps, this can dominate; report the crossover problem size instead of hiding the overhead.
- The WSL2 scheduler and available CPU cores can distort scaling. Re-run release binaries on a consistent Linux host; avoid simultaneous heavy workloads.
- `wait_ms_sum` totals per-worker condition-variable sleep time, **not mutex contention time**; it is not a rigorous lock-profile metric.
- Race-freedom on every possible schedule is not proven by CTest; stress tests and ThreadSanitizer are still recommended.
- The framework is a **shared-memory** HDA* research implementation. No MPI or network overhead is measured.

## Report-ready figures

Use the generated SVGs for runtime, speedup, scaling efficiency, expanded nodes, CO, SO, LB and expansion imbalance. For statements about lock contention, collect additional direct lock-wait instrumentation or profiling; the current numbers alone don't isolate lock contention.
