# Architecture: Parallel A* Research Framework

## Structure

```text
                    run_all.sh
                      |
       cmake/ctest + benchmark.py + plot_results.py
                      |
                 parallel_astar CLI
                      |
           Graph + Heuristic + Result
                      |
      +---------------+---------------+
      |               |               |
   Sequential      Bidirectional    Pthreads
  / Dijkstra        one-thread        | 
                                 +----+------+
                                 |           |
                             Shared      HDA* owner
                              queue       +----+
                                         inbox per worker
                                         own heap and records
                                         spatial hash routing
                                         atomic work credits
```

## Graph and heuristics

- Grid uses 4-neighbor unit-cost directed adjacency (pairs of reciprocal edges), Manhattan heuristic.
- OSM uses directed edges, distances in metres, haversine lower bound. A graph-wide scale `min(1, min_edge(weight/geodesic_distance))` guarantees the heuristic remains consistent with *the actual edge weights*, even if edge metadata is imperfect. `reverse` adjacency supports backward search on directed roads.
- The geographic metric is independent of 100m projected coordinate features used *only for hashing*.

## V0: Sequential A*

`best_g`, `parent`, lazy heap inserts; stale detection. A valid goal pop terminates for consistent heuristics. Dijkstra (`h=0`) is separate correctness oracle.

## V1: Bidirectional A*

Two own queues / best-g / parent tables, one thread. Backward edge expansion traverses `reverse`. Every intersection of discovered frontiers provides an incumbent. One frontier has a valid A* lower bound (`min_f`); a complete path of cost `U` is optimal when **either** frontier lower bound reaches `U`. If a frontier exhausts with `U=INF`, the route is unreachable. No first-meeting early stop.

## V1-P: Parallel Bidirectional A* (`bidir_parallel`)

Same search and stopping rule as V1, but the forward and backward frontiers run in **two Pthreads**. Each thread exclusively owns its queue and parent table. Shared state is minimal:

- `g` tables are `std::atomic<double>` (seq_cst): the owner stores, the other side loads in `meet()`. Store-then-load on both sides guarantees that when both reach a vertex concurrently, at least one sees the other's `g`, so no meeting is missed.
- `U` and `meeting` are updated together under one mutex; `bound` is an atomic copy of `U` for lock-free reads.
- A thread whose own `min_f >= U`, or whose queue is exhausted, sets an atomic `stop` flag and both threads exit. Optimality follows from the V1 argument: either frontier's lower bound reaching `U` proves `U` optimal.

Paths are reconstructed after `pthread_join`, so parent tables need no synchronization.

## V2: Shared-Queue A*

- Shared global priority queue protected by scheduler mutex and condition variable.
- `best_g` and `parent` protected together with striped record mutexes (4096 stripes).
- Queue lock released before expansion; no data race on records.
- `pending` counts entries in heap plus active expansions. A worker publishes new entries under scheduler lock before decrementing its active credit, preventing premature termination.

## V3: Hash-Distributed A*

All workers share read-only graph and atomic incumbent/outstanding counters. Each worker **exclusively owns** its local heap and state table. Only its destination `inbox` is synchronized by a per-owner Pthreads mutex and condition variable.

```text
     Worker i                                            Worker j
  +-------------+                                    +-------------+
  | inbox i     | <--- remote successor batches --   | inbox j     |
  | (mutex/cv)  |                                    | (mutex/cv)  |
  |             |                                    |             |
  | owner state |                                    | owner state |
  | table + g   |                                    | table + g   |
  |      |      |                                    |      |      |
  | local heap  |                                    | local heap  |
  |      |      |                                    |      |      |
  | expand      | ---- owner(s)=H(s)%P ---------->   | expand      |
  +-------------+                                    +-------------+
            \                                           / 
             \___________ atomic outstanding __________/   (no global scheduler mutex)
```

### Work accounting protocol

**A unit of work = a pending heap entry, a pending inbox message, or a currently active expansion.**

1. Initial state provides one credit.
2. Local improved successor: atomically `outstanding++` before local heap insertion.
3. Remote successor: reserve one atomic credit **when buffered in the sender outbox**; later batch publication transfers this existing credit to destination inbox (no double increment).
4. Incoming message accepted as improved: its credit **transfers** to a heap entry (not incremented twice).
5. Duplicate/pruned inbox message: decrement its credit.
6. Expanded/stale/pruned heap entry: finish exactly once after publishing all generated tasks.
7. Last credit sets `done=true` and broadcasts under each inbox mutex; no worker may sleep after missed termination signal.

Local Open/State Tables are never locked across owners. `incumbent` is an atomic upper bound used to safely prune `g+h>=U` for admissible heuristics. HDA* does not assume global best-first expansion, so it can reopen an improved state and may expand more nodes than V0.

### Batching

`--batch N` bounds both inbox-processing bursts and outgoing per-destination message batches. Outgoing buffers can persist **across expansions**; each buffered candidate has already reserved one global Outstanding Credit. Buffers flush when reaching batch cap, after `--flush-interval` expansions, or before a sender would sleep. Therefore buffer persistence cannot cause uncounted in-transit work and global quiescence remains sound.

### Hash strategies

| `--hash` | Owner features | Intent |
|---|---|---|
| mod | vertex ID mod P | simplistic baseline |
| mixed | splitmix64(vertex ID) mod P | pseudo-random state assignment |
| zobrist | mixed random key(x) XOR mixed random key(y) | coordinate-feature ZH |
| abstract | hash(spatial coarse cell) | locality baseline |
| azobrist | coarse key(x/tile) XOR coarse key(y/tile) | domain-adapted AZH |

This paper-inspired spatial abstraction is a new design for grid/road vertices, not a claim of bit-for-bit reproduction of the author's 15-puzzle or planning-specific feature encodings.
