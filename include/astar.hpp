#pragma once
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace astar {
using Vertex = uint32_t;
    constexpr Vertex NONE = std::numeric_limits<Vertex>::max();
    constexpr double INF = std::numeric_limits<double>::infinity();

    struct Edge { 
        Vertex to; 
        double cost; 
    };

    struct Point { 
        double latitude = 0, longitude = 0; 
        int64_t x = 0, y = 0; 
    };

    struct Graph {
        std::vector<std::vector<Edge>> adj, reverse;
        std::vector<Point> point;
        bool grid = false;
        int grid_side = 0;
        double road_heuristic_scale = 1.0;
        size_t size() const { 
            return adj.size(); 
        }
        double heuristic(Vertex u, Vertex goal) const;
    };

    struct SearchStats {
        uint64_t expanded=0, generated=0, improved=0, stale=0, pruned=0;
        uint64_t remote=0, remote_batches=0, inbox_received=0, assigned=0;
        uint64_t forward_expanded=0, backward_expanded=0;
        uint64_t heuristic_calls=0;   // number of Graph::heuristic() calls
        double heuristic_ms=0;        // time inside heuristic(), only with --profile-heuristic 1
        double wait_ms=0;
        std::vector<uint64_t> per_worker_expanded;
        std::vector<uint64_t> per_worker_generated;
        std::vector<uint64_t> per_worker_assigned;
    };

    struct SearchResult {
        bool found=false;
        double cost=INF;
        std::vector<Vertex> path;
        double time_ms=0;
        SearchStats stats;
    };

    struct Config {
        int threads=1;
        std::string hash="zobrist";
        int tile=8;
        int batch=32;
        int flush_interval=8;
        bool prune=true;
    };

    // ---- Heuristic profiling ----
    // Every Graph::heuristic() call is counted in a thread-local counter.
    // When profile_heuristic is true, each call is also timed (this adds timer
    // overhead to the search time, so use it in a separate experiment).
    struct HeuristicProfile {
        uint64_t calls = 0;
        double ns = 0;
    };
    extern bool profile_heuristic;          // set once before searching
    HeuristicProfile& heuristic_profile();  // counters of the calling thread

    // Call at the start of a search (in each thread that calls heuristic()).
    inline void begin_heuristic_profile() { heuristic_profile() = {}; }
    // Call at the end: moves this thread's counters into stats.
    inline void end_heuristic_profile(SearchStats& stats) {
        auto& p = heuristic_profile();
        stats.heuristic_calls += p.calls;
        stats.heuristic_ms += p.ns / 1e6;
        p = {};
    }

    Graph make_grid(int side, uint64_t seed, double density);
    Graph load_osm_csv(const std::string& nodes, const std::string& edges);
    SearchResult sequential(const Graph&, Vertex, Vertex);
    SearchResult bidirectional(const Graph&, Vertex, Vertex);
    SearchResult bidirectional_parallel(const Graph&, Vertex, Vertex);
    SearchResult shared_queue(const Graph&, Vertex, Vertex, const Config&);
    SearchResult hda(const Graph&, Vertex, Vertex, const Config&);
    SearchResult dijkstra(const Graph&, Vertex, Vertex);
    bool validate_path(const Graph&,Vertex,Vertex,const SearchResult&,std::string&);
} // namespace astar
