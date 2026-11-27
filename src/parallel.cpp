// By 其龍
// V2: Shared-Queue A*
// V3: HDA*
// 2 threads bidirectional

#include "astar.hpp"

#include <pthread.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace astar {
    namespace {

        constexpr double EPS = 1e-9;
        using Clock = std::chrono::steady_clock;

        double ms(Clock::duration d) {
            return std::chrono::duration<double, std::milli>(d).count();
        }

        // ---- Pthreads wrappers ----

        struct Mutex {
            pthread_mutex_t v{};

            Mutex() { pthread_mutex_init(&v, nullptr); }
            ~Mutex() { pthread_mutex_destroy(&v); }
            Mutex(const Mutex&) = delete;
            Mutex& operator=(const Mutex&) = delete;

            void lock() { pthread_mutex_lock(&v); }
            void unlock() { pthread_mutex_unlock(&v); }
        };

        struct Lock {
            Mutex& m;
            explicit Lock(Mutex& m) : m(m) { m.lock(); }
            ~Lock() { m.unlock(); }
        };

        struct Condition {
            pthread_cond_t v{};

            Condition() { pthread_cond_init(&v, nullptr); }
            ~Condition() { pthread_cond_destroy(&v); }

            void wait(Mutex& m) { pthread_cond_wait(&v, &m.v); }
            void one() { pthread_cond_signal(&v); }
            void all() { pthread_cond_broadcast(&v); }
        };

        // ---- Search data structures ----

        struct Entry {
            double f, g;
            Vertex v;
        };

        struct Greater {
            bool operator()(const Entry& a, const Entry& b) const {
                if (a.f != b.f) {
                    return a.f > b.f;
                }
                if (a.g != b.g) {
                    return a.g < b.g;
                }
                return a.v > b.v;
            }
        };

        using Heap = std::priority_queue<Entry, std::vector<Entry>, Greater>;

        struct State {
            double g = INF;
            Vertex parent = NONE;
        };

        // ---- Hashing ----

        uint64_t mix64(uint64_t x) {
            x += 0x9e3779b97f4a7c15ULL;
            x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
            x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
            return x ^ (x >> 31);
        }

        uint64_t coord_key(int64_t x, uint64_t salt) {
            return mix64(uint64_t(x) ^ salt);
        }

        int owner(const Graph& graph, Vertex v, const Config& conf) {
            if (conf.hash == "mod") {
                return int(v % uint32_t(conf.threads));
            }
            if (conf.hash == "mixed") {
                return int(mix64(v) % uint64_t(conf.threads));
            }

            auto p = graph.point[v];
            int64_t x = p.x, y = p.y;
            if (conf.hash == "abstract" || conf.hash == "azobrist") {
                // Floor division supports negative coordinates too.
                auto floordiv = [](int64_t value, int64_t div) {
                    return value >= 0 ? value / div : -((-value + div - 1) / div);
                };
                x = floordiv(x, conf.tile);
                y = floordiv(y, conf.tile);
            }

            uint64_t h = 0;
            if (conf.hash == "abstract") {
                h = mix64(coord_key(x, 0x51ULL) ^ coord_key(y, 0x92ULL));
            } 
            else {
                // Zobrist-style XOR over independently seeded coordinate features.
                // Using projected/coarse features yields our graph-specific AZH variant.
                h = coord_key(x, 0x85d6ULL) ^ coord_key(y, 0x721aULL);
            }
            return int(h % uint64_t(conf.threads));
        }

        // ---- Helpers ----

        void update_min(std::atomic<double>& incumbent, double cost) {
            double observed = incumbent.load(std::memory_order_relaxed);
            while (cost < observed && !incumbent.compare_exchange_weak(observed, cost, std::memory_order_relaxed)) {
            }
        }

        void aggregate(SearchResult& out, const std::vector<SearchStats>& workers) {
            for (const auto& x : workers) {
                out.stats.expanded += x.expanded;
                out.stats.generated += x.generated;
                out.stats.improved += x.improved;
                out.stats.stale += x.stale;
                out.stats.pruned += x.pruned;
                out.stats.remote += x.remote;
                out.stats.remote_batches += x.remote_batches;
                out.stats.inbox_received += x.inbox_received;
                out.stats.assigned += x.assigned;
                out.stats.wait_ms += x.wait_ms;
                out.stats.heuristic_calls += x.heuristic_calls;
                out.stats.heuristic_ms += x.heuristic_ms;
                out.stats.per_worker_expanded.push_back(x.expanded);
                out.stats.per_worker_generated.push_back(x.generated);
                out.stats.per_worker_assigned.push_back(x.assigned);
            }
        }

        void check_config(const Graph& g, Vertex s, Vertex t, const Config& config) {
            if (s >= g.size() || t >= g.size()) {
                throw std::runtime_error("start or goal outside graph");
            }
            if (config.threads < 1 || config.threads > 128) {
                throw std::runtime_error("threads must be in 1..128");
            }
            if (config.batch < 1 || config.batch > 100000) {
                throw std::runtime_error("batch must be in 1..100000");
            }
            if (config.tile < 1 || config.flush_interval < 1) {
                throw std::runtime_error("tile and flush_interval must be positive");
            }
            if (config.hash != "mixed" && config.hash != "mod" && config.hash != "zobrist" &&
                config.hash != "abstract" && config.hash != "azobrist") {
                throw std::runtime_error("unknown hash policy");
            }
        }

    }  // namespace

    // ============================================================================
    // V2: Shared-Queue A*
    // ============================================================================

    SearchResult shared_queue(const Graph& graph, Vertex s, Vertex t, const Config& config) {
        check_config(graph, s, t, config);
        const auto begin = Clock::now();
        begin_heuristic_profile();
        SearchResult result;

        if (s == t) {
            result.found = true;
            result.cost = 0;
            result.path = {s};
            result.stats.per_worker_expanded.assign(config.threads, 0);
            result.time_ms = ms(Clock::now() - begin);
            return result;
        }

        struct Context {
            const Graph& graph;
            Vertex goal;
            bool prune;

            Mutex scheduler;
            Condition cv;
            Heap open;
            uint64_t pending = 1;  // queued + currently executing entries

            std::vector<State> state;
            std::vector<std::unique_ptr<Mutex>> stripe;
            std::atomic<double> incumbent{INF};
            std::vector<SearchStats> counters;

            explicit Context(const Graph& g, Vertex t, const Config& c)
                : graph(g), goal(t), prune(c.prune), state(g.size()), counters(c.threads) {
                stripe.reserve(4096);
                for (int i = 0; i < 4096; ++i) {
                    stripe.emplace_back(std::make_unique<Mutex>());
                }
            }

            void add(Entry e) {
                Lock l(scheduler);
                open.push(e);
                ++pending;
                cv.one();
            }

            void finish() {
                Lock l(scheduler);
                if (--pending == 0) {
                    cv.all();
                }
            }
        } ctx(graph, t, config);

        ctx.state[s].g = 0;
        ctx.open.push({graph.heuristic(s, t), 0, s});

        struct Worker {
            Context* c;
            int id;

            static void* run(void* ptr) {
                auto& self = *static_cast<Worker*>(ptr);
                auto& c = *self.c;
                auto& stats = c.counters[self.id];
                begin_heuristic_profile();

                while (true) {
                    // Pop one entry from the shared queue.
                    Entry e{};
                    {
                        Lock l(c.scheduler);
                        while (c.open.empty() && c.pending > 0) {
                            auto before = Clock::now();
                            c.cv.wait(c.scheduler);
                            stats.wait_ms += ms(Clock::now() - before);
                        }
                        if (c.pending == 0) {
                            break;
                        }
                        e = c.open.top();
                        c.open.pop();
                        ++stats.assigned;
                    }

                    double best;
                    {
                        Lock l(*c.stripe[e.v % 4096]);
                        best = c.state[e.v].g;
                    }
                    if (e.g > best + EPS) {
                        ++stats.stale;
                        c.finish();
                        continue;
                    }
                    if (c.prune && e.f >= c.incumbent.load(std::memory_order_relaxed) - EPS) {
                        ++stats.pruned;
                        c.finish();
                        continue;
                    }
                    if (e.v == c.goal) {
                        update_min(c.incumbent, e.g);
                        c.finish();
                        continue;
                    }

                    // Expand.
                    ++stats.expanded;
                    for (const auto& edge : c.graph.adj[e.v]) {
                        ++stats.generated;
                        double ng = e.g + edge.cost;
                        double nf = ng + c.graph.heuristic(edge.to, c.goal);
                        if (c.prune && nf >= c.incumbent.load(std::memory_order_relaxed) - EPS) {
                            ++stats.pruned;
                            continue;
                        }

                        bool improved = false;
                        {
                            Lock l(*c.stripe[edge.to % 4096]);
                            State& rec = c.state[edge.to];
                            if (ng + EPS < rec.g) {
                                rec = {ng, e.v};
                                improved = true;
                            }
                        }
                        if (improved) {
                            ++stats.improved;
                            if (edge.to == c.goal) {
                                update_min(c.incumbent, ng);
                            }
                            c.add({nf, ng, edge.to});
                        }
                    }
                    c.finish();  // all generated work registered before current work completes
                }
                end_heuristic_profile(stats);
                return nullptr;
            }
        };

        std::vector<Worker> workers;
        workers.reserve(config.threads);
        for (int i = 0; i < config.threads; ++i) {
            workers.push_back({&ctx, i});
        }

        std::vector<pthread_t> threads(config.threads);
        for (int i = 0; i < config.threads; ++i) {
            if (int err = pthread_create(&threads[i], nullptr, Worker::run, &workers[i])) {
                throw std::runtime_error("pthread_create failed: " + std::to_string(err));
            }
        }
        for (auto& thread : threads) {
            pthread_join(thread, nullptr);
        }

        // Reconstruct path.
        auto& destination = ctx.state[t];
        if (std::isfinite(destination.g)) {
            result.found = true;
            result.cost = destination.g;
            Vertex v = t;
            for (size_t j = 0; j <= graph.size(); ++j) {
                result.path.push_back(v);
                if (v == s) {
                    break;
                }
                v = ctx.state[v].parent;
                if (v == NONE) {
                    break;
                }
            }
            if (result.path.back() != s) {
                result.path.clear();
            } 
            else {
                std::reverse(result.path.begin(), result.path.end());
            }
        }

        aggregate(result, ctx.counters);
        end_heuristic_profile(result.stats);
        result.time_ms = ms(Clock::now() - begin);
        return result;
    }

    // ============================================================================
    // V3: Hash-Distributed A* (HDA*)
    // ============================================================================

    SearchResult hda(const Graph& graph, Vertex s, Vertex t, const Config& config) {
        check_config(graph, s, t, config);
        const auto begin = Clock::now();
        begin_heuristic_profile();
        SearchResult result;

        if (s == t) {
            result.found = true;
            result.cost = 0;
            result.path = {s};
            result.stats.per_worker_expanded.assign(config.threads, 0);
            result.time_ms = ms(Clock::now() - begin);
            return result;
        }

        struct Message {
            Vertex v, parent;
            double g;
        };

        struct Owner {
            Mutex inbox_lock;
            Condition inbox_cv;
            std::deque<Message> inbox;
            Heap open;                                // accessible only by this worker
            std::unordered_map<Vertex, State> table;  // accessible only by this worker
        };

        struct Context {
            const Graph& graph;
            Vertex start, goal;
            Config config;
            std::vector<std::unique_ptr<Owner>> workers;
            std::vector<SearchStats> counters;
            std::atomic<uint64_t> outstanding{1};
            std::atomic<bool> done{false};
            std::atomic<double> incumbent{INF};

            Context(const Graph& g, Vertex s, Vertex t, const Config& c)
                : graph(g), start(s), goal(t), config(c), counters(c.threads) {
                for (int i = 0; i < c.threads; ++i) {
                    workers.emplace_back(std::make_unique<Owner>());
                }
            }

            int who(Vertex v) const { return owner(graph, v, config); }

            // IMPORTANT: count credits before publishing under the destination inbox lock.
            void send_batch(int target, const std::vector<Message>& batch) {
                if (batch.empty()) {
                    return;
                }
                // Message credits were already reserved when entering sender-side outbox.
                Owner& dest = *workers[target];
                {
                    Lock guard(dest.inbox_lock);
                    for (const auto& msg : batch) {
                        dest.inbox.push_back(msg);
                    }
                    dest.inbox_cv.one();
                }
            }

            void finish(uint64_t n = 1) {
                uint64_t old = outstanding.fetch_sub(n, std::memory_order_acq_rel);
                if (old < n) {
                    std::abort();
                }
                if (old == n) {
                    done.store(true, std::memory_order_release);
                    // Locking each inbox when waking prevents a lost wakeup between
                    // checking `done` and entering pthread_cond_wait.
                    for (auto& w : workers) {
                        Lock guard(w->inbox_lock);
                        w->inbox_cv.all();
                    }
                }
            }
        } ctx(graph, s, t, config);

        int first = ctx.who(s);
        ctx.workers[first]->table.emplace(s, State{0, NONE});
        ctx.workers[first]->open.push({graph.heuristic(s, t), 0, s});
        ctx.counters[first].assigned = 1;

        struct Worker {
            Context* c;
            int id;

            static void* run(void* ptr) {
                auto& self = *static_cast<Worker*>(ptr);
                auto& c = *self.c;
                Owner& own = *c.workers[self.id];
                SearchStats& stats = c.counters[self.id];
                begin_heuristic_profile();

                std::vector<std::vector<Message>> outbox(c.config.threads);
                std::vector<int> touched;
                touched.reserve(c.config.threads);
                std::vector<uint8_t> ever_touched(c.config.threads, 0);
                int expanded_since_flush = 0;

                auto flush = [&](int dst) {
                    if (outbox[dst].empty()) {
                        return;
                    }
                    ++ stats.remote_batches;
                    c.send_batch(dst, outbox[dst]);
                    outbox[dst].clear();
                };
                auto flush_all = [&]() {
                    for (int dst : touched) {
                        flush(dst);
                    }
                    expanded_since_flush = 0;
                };

                while (true) {
                    // ---- 1. Drain a bounded batch from the inbox ----
                    std::vector<Message> received;
                    {
                        Lock lock(own.inbox_lock);
                        int n = std::min(int(own.inbox.size()), c.config.batch);
                        received.reserve(n);
                        for (int i = 0; i < n; ++i) {
                            received.push_back(own.inbox.front());
                            own.inbox.pop_front();
                        }
                    }

                    for (const auto& msg : received) {
                        ++ stats.inbox_received;
                        ++ stats.assigned;
                        auto it = own.table.find(msg.v);
                        if (it == own.table.end() || msg.g + EPS < it->second.g) {
                            own.table[msg.v] = {msg.g, msg.parent};
                            ++ stats.improved;
                            if (msg.v == c.goal) {
                                update_min(c.incumbent, msg.g);
                            }
                            double f = msg.g + c.graph.heuristic(msg.v, c.goal);
                            if (!c.config.prune || f < c.incumbent.load(std::memory_order_relaxed) - EPS) {
                                // Inbox credit transfers to local heap entry: no count change.
                                own.open.push({f, msg.g, msg.v});
                                continue;
                            }
                            ++ stats.pruned;
                        } 
                        else {
                            ++ stats.stale;
                        }
                        // Discarded inbox credit must be completed exactly once.
                        c.finish();
                    }

                    // ---- 2. Expand one node from the local heap ----
                    if (!own.open.empty()) {
                        Entry e = own.open.top();
                        own.open.pop();

                        auto it = own.table.find(e.v);
                        if (it == own.table.end() || e.g > it->second.g + EPS) {
                            ++stats.stale;
                            c.finish();
                            continue;
                        }
                        if (c.config.prune && e.f >= c.incumbent.load(std::memory_order_relaxed) - EPS) {
                            ++stats.pruned;
                            c.finish();
                            continue;
                        }
                        if (e.v == c.goal) {
                            update_min(c.incumbent, e.g);
                            c.finish();
                            continue;
                        }

                        ++stats.expanded;
                        for (const auto& edge : c.graph.adj[e.v]) {
                            ++stats.generated;
                            double ng = e.g + edge.cost;
                            double nf = ng + c.graph.heuristic(edge.to, c.goal);
                            if (c.config.prune && nf >= c.incumbent.load(std::memory_order_relaxed) - EPS) {
                                ++stats.pruned;
                                continue;
                            }

                            int dst = c.who(edge.to);
                            if (dst == self.id) {
                                // Local successor.
                                ++stats.assigned;
                                auto jt = own.table.find(edge.to);
                                if (jt == own.table.end() || ng + EPS < jt->second.g) {
                                    own.table[edge.to] = {ng, e.v};
                                    ++stats.improved;
                                    if (edge.to == c.goal) {
                                        update_min(c.incumbent, ng);
                                    }
                                    // Register before owner-exclusive local queue publication.
                                    c.outstanding.fetch_add(1, std::memory_order_acq_rel);
                                    own.open.push({nf, ng, edge.to});
                                }
                            } else {
                                // Remote successor.
                                ++stats.remote;
                                auto& buffer = outbox[dst];
                                if (!ever_touched[dst]) {
                                    ever_touched[dst] = 1;
                                    touched.push_back(dst);
                                }
                                // This token represents work buffered but not yet published.
                                // Thus an idle sender cannot cause premature global quiescence.
                                c.outstanding.fetch_add(1, std::memory_order_acq_rel);
                                buffer.push_back({edge.to, e.v, ng});
                                if (int(buffer.size()) >= c.config.batch) {
                                    flush(dst);
                                }
                            }
                        }

                        // Periodically flush persistent sender-side buffers so receivers
                        // are not starved; all buffered messages already own credits.
                        if (++expanded_since_flush >= c.config.flush_interval) {
                            flush_all();
                        }
                        c.finish();
                        continue;
                    }

                    // ---- 3. Idle: flush and wait on own inbox ----
                    // Flush any remaining sender-side buffers before becoming idle.
                    // All buffered work was already counted, so done cannot race this.
                    flush_all();

                    // A completely idle owner waits only on its OWN inbox mutex/CV.
                    // No global scheduler mutex is acquired on the hot path.
                    {
                        Lock lock(own.inbox_lock);
                        while (own.inbox.empty() && !c.done.load(std::memory_order_acquire)) {
                            auto before = Clock::now();
                            own.inbox_cv.wait(own.inbox_lock);
                            stats.wait_ms += ms(Clock::now() - before);
                        }
                        if (c.done.load(std::memory_order_acquire) && own.inbox.empty()) {
                            break;
                        }
                    }
                }
                end_heuristic_profile(stats);
                return nullptr;
            }
        };

        std::vector<Worker> worker;
        worker.reserve(config.threads);
        for (int i = 0; i < config.threads; ++i) {
            worker.push_back({&ctx, i});
        }

        std::vector<pthread_t> threads(config.threads);
        for (int i = 0; i < config.threads; ++i) {
            if (int err = pthread_create(&threads[i], nullptr, Worker::run, &worker[i])) {
                throw std::runtime_error("pthread_create failed: " + std::to_string(err));
            }
        }
        for (auto& thread : threads) {
            pthread_join(thread, nullptr);
        }

        // Reconstruct path by following parents across owner tables.
        auto it = ctx.workers[ctx.who(t)]->table.find(t);
        if (it != ctx.workers[ctx.who(t)]->table.end()) {
            result.found = true;
            result.cost = it->second.g;
            Vertex v = t;
            for (size_t i = 0; i <= graph.size(); ++i) {
                result.path.push_back(v);
                if (v == s) {
                    break;
                }
                auto& tab = ctx.workers[ctx.who(v)]->table;
                auto jt = tab.find(v);
                if (jt == tab.end() || jt->second.parent == NONE) {
                    break;
                }
                v = jt->second.parent;
            }
            if (result.path.back() != s) {
                result.path.clear();
            } 
            else {
                std::reverse(result.path.begin(), result.path.end());
            }
        }

        aggregate(result, ctx.counters);
        end_heuristic_profile(result.stats);
        result.time_ms = ms(Clock::now() - begin);
        return result;
    }

    // ============================================================================
    // V1-P: Parallel Bidirectional A* (2 threads: forward + backward)
    // ============================================================================
    //
    // Each direction runs its own A* in its own thread, and exclusively owns its
    // queue and parent table. The threads only share:
    //   - g values (atomic): written by the owner, read by the other side in meet()
    //   - U / meeting (mutex): best complete path found so far
    //   - bound (atomic copy of U) and stop flag: lock-free reads in the hot loop
    //
    // Stopping rule is the same as the sequential V1: a direction stops everything
    // once its own min f >= U (its frontier can no longer improve U), or when its
    // queue is exhausted.

    SearchResult bidirectional_parallel(const Graph& graph, Vertex s, Vertex t) {
        if (s >= graph.size() || t >= graph.size()) {
            throw std::runtime_error("start or goal outside graph");
        }
        const auto begin = Clock::now();
        begin_heuristic_profile();
        SearchResult result;

        if (s == t) {
            result.found = true;
            result.cost = 0;
            result.path = {s};
            result.stats.per_worker_expanded.assign(2, 0);
            result.time_ms = ms(Clock::now() - begin);
            return result;
        }

        struct Side {
            // g[v] uses seq_cst store/load so that when both sides reach v at the
            // same time, at least one of them sees the other's g (no missed meeting).
            std::vector<std::atomic<double>> g;
            std::vector<Vertex> parent;  // owner only; read by main thread after join
            Heap open;                   // owner only
            SearchStats stats;           // owner only

            explicit Side(size_t n) : g(n), parent(n, NONE) {
                for (auto& x : g) {
                    x.store(INF, std::memory_order_relaxed);
                }
            }
        };

        struct Shared {
            const Graph& graph;
            Side forward, backward;

            Mutex best_lock;
            double U = INF;          // guarded by best_lock
            Vertex meeting = NONE;   // guarded by best_lock
            std::atomic<double> bound{INF};  // latest U, for lock-free reads
            std::atomic<bool> stop{false};

            Shared(const Graph& g) : graph(g), forward(g.size()), backward(g.size()) {}

            // `mine` has just been stored into own.g[v]; check if the other side reached v.
            void meet(Vertex v, double mine, const Side& other) {
                double theirs = other.g[v].load();
                if (!std::isfinite(theirs)) {
                    return;
                }
                double cand = mine + theirs;
                if (cand + EPS >= bound.load(std::memory_order_relaxed)) {
                    return;
                }
                Lock l(best_lock);
                if (cand + EPS < U) {
                    U = cand;
                    meeting = v;
                    bound.store(cand, std::memory_order_relaxed);
                }
            }
        } shared(graph);

        shared.forward.g[s].store(0);
        shared.backward.g[t].store(0);
        shared.forward.open.push({graph.heuristic(s, t), 0, s});
        shared.backward.open.push({graph.heuristic(t, s), 0, t});

        struct Worker {
            Shared* sh;
            bool is_forward;
            Vertex destination;

            static void* run(void* ptr) {
                auto& self = *static_cast<Worker*>(ptr);
                auto& sh = *self.sh;
                Side& own = self.is_forward ? sh.forward : sh.backward;
                const Side& other = self.is_forward ? sh.backward : sh.forward;
                const auto& adjacency = self.is_forward ? sh.graph.adj : sh.graph.reverse;
                const Vertex destination = self.destination;  // forward -> goal, backward -> start
                auto& stats = own.stats;
                begin_heuristic_profile();

                while (!sh.stop.load(std::memory_order_relaxed)) {
                    // Drop stale entries so top() is a valid lower bound.
                    while (!own.open.empty() &&
                           own.open.top().g > own.g[own.open.top().v].load(std::memory_order_relaxed) + EPS) {
                        own.open.pop();
                        ++stats.stale;
                    }

                    // This direction is exhausted, or cannot improve U anymore: stop both.
                    if (own.open.empty() ||
                        own.open.top().f >= sh.bound.load(std::memory_order_relaxed) - EPS) {
                        sh.stop.store(true, std::memory_order_relaxed);
                        break;
                    }

                    Entry e = own.open.top();
                    own.open.pop();
                    sh.meet(e.v, e.g, other);

                    ++stats.expanded;
                    for (const auto& edge : adjacency[e.v]) {
                        ++stats.generated;
                        double ng = e.g + edge.cost;
                        // Only this thread writes own.g, so a relaxed read is enough here.
                        if (ng + EPS < own.g[edge.to].load(std::memory_order_relaxed)) {
                            own.g[edge.to].store(ng);  // seq_cst, pairs with load in meet()
                            own.parent[edge.to] = e.v;
                            ++stats.improved;
                            own.open.push({ng + sh.graph.heuristic(edge.to, destination), ng, edge.to});
                            sh.meet(edge.to, ng, other);
                        }
                    }
                }
                end_heuristic_profile(stats);
                return nullptr;
            }
        };

        Worker workers[2] = {{&shared, true, t}, {&shared, false, s}};
        pthread_t threads[2];
        for (int i = 0; i < 2; ++i) {
            if (int err = pthread_create(&threads[i], nullptr, Worker::run, &workers[i])) {
                throw std::runtime_error("pthread_create failed: " + std::to_string(err));
            }
        }
        for (auto& thread : threads) {
            pthread_join(thread, nullptr);
        }

        // Reconstruct s -> meeting (forward parents) + meeting -> t (backward parents).
        Vertex meeting = shared.meeting;
        if (meeting != NONE) {
            std::vector<Vertex> path;
            Vertex v = meeting;
            for (size_t i = 0; i <= graph.size() && v != NONE; ++i) {
                path.push_back(v);
                if (v == s) {
                    break;
                }
                v = shared.forward.parent[v];
            }
            std::reverse(path.begin(), path.end());

            v = meeting;
            for (size_t i = 0; i <= graph.size() && v != t; ++i) {
                v = shared.backward.parent[v];
                if (v == NONE) {
                    break;
                }
                path.push_back(v);
            }

            if (!path.empty() && path.front() == s && path.back() == t) {
                // Parent chains may have improved after U was recorded:
                // report the actual cost of the returned path.
                double cost = 0;
                for (size_t i = 1; i < path.size(); ++i) {
                    double w = INF;
                    for (const auto& edge : graph.adj[path[i - 1]]) {
                        if (edge.to == path[i]) {
                            w = std::min(w, edge.cost);
                        }
                    }
                    cost += w;
                }
                result.found = true;
                result.cost = cost;
                result.path = std::move(path);
            }
        }

        aggregate(result, {shared.forward.stats, shared.backward.stats});
        result.stats.forward_expanded = shared.forward.stats.expanded;
        result.stats.backward_expanded = shared.backward.stats.expanded;
        end_heuristic_profile(result.stats);
        result.time_ms = ms(Clock::now() - begin);
        return result;
    }

}  // namespace astar
