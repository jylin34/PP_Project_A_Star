# Relationship to the cited HDA* paper

Source: **Yuu Jinnai and Alex Fukunaga (2016), Abstract Zobrist Hashing: An Efficient Work Distribution Method for Parallel Best-First Search**, AAAI 2016.

- Paper PDF: https://jinnaiyuu.github.io/pdf/papers/AAAI-16%20Jinnai-Fukunaga.pdf
- Author's code: https://github.com/jinnaiyuu/Hash-Distributed-Astar
- Background: Kishimoto, Fukunaga, and Botea, *Evaluation of a Simple, Scalable, Parallel Best-First Search Strategy*, Artificial Intelligence 195 (2013).

## Taken from the HDA* design

1. Unique owner for each state from a global deterministic hash.
2. Each worker owns and updates its *own* OPEN, duplicate-detection/state table, and parent record.
3. Generated successors are transferred asynchronously to the destination inbox.
4. Incoming messages are deduplicated by the owner; improved states can reopen.
5. Analyze **CO**, **SO**, and **LB** (paper Sec. 2.1), not only wall time.
6. Compare high-entropy vs spatially abstract hash allocation; locality reduces handoffs but may increase search overhead.

## Specific project decisions / departures

- Generic vertex pathfinding, not the original paper's sliding-tile, multiple sequence alignment or PDDL domains.
- Pthreads shared-memory messages, not cross-machine MPI processes.
- Coarsened 2D grid features are a project-specific Abstract Zobrist analog; **not an exact reproduction** of all paper feature projections.
- Sender-side outboxes persist across expansions; every buffered message is counted immediately, and buffers flush by batch cap, periodically, or before a worker sleeps.
- Global quiescence tracked via atomic work credits + per-owner condition-variable wakeups. This is simpler and conservative compared to other distributed termination protocols.
- `h = Manhattan` for 4-connected unit grids; roads use consistent (scaled) geodesic lower bound.
- A complete candidate path sets an incumbent upper bound; we don't terminate immediately on the first goal or first frontier meeting.

This architecture is inspired by the author's *algorithmic design*. It does not incorporate or redistribute their original source code.
