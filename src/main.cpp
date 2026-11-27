// By 其龍

#include "astar.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace {

    std::string json_number(double x) {
        if (!std::isfinite(x)) {
            return "null";
        }
        std::ostringstream ss;
        ss << std::setprecision(12) << x;
        return ss.str();
    }

    std::string json_vec(const std::vector<uint64_t>& a) {
        std::ostringstream o;
        o << "[";
        for (size_t i = 0; i < a.size(); ++i) {
            if (i) {
                o << ",";
            }
            o << a[i];
        }
        o << "]";
        return o.str();
    }

    std::string json_path(const std::vector<astar::Vertex>& a) {
        std::ostringstream o;
        o << "[";
        for (size_t i = 0; i < a.size(); ++i) {
            if (i) {
                o << ",";
            }
            o << a[i];
        }
        o << "]";
        return o.str();
    }

}  // namespace

int main(int argc, char** argv) {
    try {
        // ---- Parse command-line options ----
        std::unordered_map<std::string, std::string> opt;
        for (int i = 1; i < argc; ++i) {
            std::string key = argv[i];
            if (key == "--help" || key == "-h") {
                std::cout << "Usage: parallel_astar --algorithm dijkstra|sequential|bidirectional|bidir_parallel|shared|hda "
                             "[--grid N --seed N --density D | --nodes nodes.csv --edges edges.csv] "
                             "--start S --goal T [--threads P --hash POLICY --tile N --batch N --verify 1 --profile-heuristic 0|1]\n";
                return 0;
            }
            if (key.rfind("--", 0) != 0 || i + 1 >= argc) {
                throw std::runtime_error("expected --key value: " + key);
            }
            opt[key.substr(2)] = argv[++i];
        }

        auto get = [&](const std::string& name, const std::string& d = "") {
            auto it = opt.find(name);
            return it == opt.end() ? d : it->second;
        };

        auto algorithm = get("algorithm", "sequential");

        astar::Config cfg;
        cfg.threads = std::stoi(get("threads", "1"));
        cfg.hash = get("hash", "zobrist");
        cfg.tile = std::stoi(get("tile", "8"));
        cfg.batch = std::stoi(get("batch", "32"));
        cfg.flush_interval = std::stoi(get("flush-interval", "8"));
        cfg.prune = get("prune", "1") != "0";

        // Time every heuristic() call. Measure the timer's own cost first so it
        // can be subtracted (an empty timed region costs this much per call).
        astar::profile_heuristic = get("profile-heuristic", "0") != "0";
        double timer_overhead_ns = 0;
        if (astar::profile_heuristic) {
            const int rounds = 1000000;
            double total = 0;
            for (int i = 0; i < rounds; ++i) {
                auto t0 = std::chrono::steady_clock::now();
                total += std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - t0).count();
            }
            timer_overhead_ns = total / rounds;
        }

        // ---- Build graph ----
        astar::Graph graph;
        if (!get("nodes").empty() && !get("edges").empty()) {
            graph = astar::load_osm_csv(get("nodes"), get("edges"));
        }
        else if (!get("grid").empty()) {
            graph = astar::make_grid(std::stoi(get("grid")),
                                     std::stoull(get("seed", "42")),
                                     std::stod(get("density", "0.2")));
        } 
        else {
            throw std::runtime_error("provide --grid N or both --nodes and --edges");
        }
        if (graph.size() > std::numeric_limits<astar::Vertex>::max()) {
            throw std::runtime_error("graph exceeds Vertex capacity");
        }

        auto s = astar::Vertex(std::stoull(get("start", "0")));
        auto t = astar::Vertex(std::stoull(get("goal", std::to_string(graph.size() - 1))));
        if (s >= graph.size() || t >= graph.size()) {
            throw std::runtime_error("start or goal out of graph bounds");
        }

        // ---- Run search ----
        astar::SearchResult r;
        if (algorithm == "sequential") {
            r = astar::sequential(graph, s, t);
        } 
        else if (algorithm == "bidirectional") {
            r = astar::bidirectional(graph, s, t);
        } 
        else if (algorithm == "dijkstra") {
            r = astar::dijkstra(graph, s, t);
        }
        else if (algorithm == "bidir_parallel") {
            r = astar::bidirectional_parallel(graph, s, t);
        }
        else if (algorithm == "shared") {
            r = astar::shared_queue(graph, s, t, cfg);
        } 
        else if (algorithm == "hda") {
            r = astar::hda(graph, s, t, cfg);
        } 
        else {
            throw std::runtime_error("unknown algorithm: " + algorithm);
        }

        // ---- Validate result ----
        std::string error;
        bool valid = astar::validate_path(graph, s, t, r, error);
        if (!valid) {
            throw std::runtime_error("INVALID returned path: " + error);
        }

        bool verified = false;
        if (get("verify", "1") != "0") {
            auto ref = astar::dijkstra(graph, s, t);
            if (ref.found != r.found ||
                (ref.found && std::fabs(ref.cost - r.cost) > 1e-7 * std::max(1.0, ref.cost))) {
                throw std::runtime_error("reference Dijkstra mismatch");
            }
            verified = true;
        }

        // ---- Compute statistics ----
        double nworker = double(r.stats.per_worker_expanded.size());

        uint64_t busiest = 0;
        for (auto exp : r.stats.per_worker_expanded) {
            busiest = std::max(busiest, exp);
        }
        double balance_expanded = (r.stats.expanded && nworker > 0) ? busiest / (r.stats.expanded / nworker) : 1;

        uint64_t most_assigned = 0;
        for (auto a : r.stats.per_worker_assigned) {
            most_assigned = std::max(most_assigned, a);
        }
        double balance_assigned = (r.stats.assigned && nworker > 0) ? most_assigned / (r.stats.assigned / nworker) : 1;

        double balance = algorithm == "hda" ? balance_assigned : balance_expanded;
        double co = r.stats.generated ? double(r.stats.remote) / r.stats.generated : 0;

        // Heuristic time with timer overhead removed (0 when not profiling).
        double heuristic_ms = 0, heuristic_share = 0;
        if (astar::profile_heuristic) {
            heuristic_ms = std::max(0.0, r.stats.heuristic_ms - r.stats.heuristic_calls * timer_overhead_ns / 1e6);
            heuristic_share = r.time_ms > 0 ? heuristic_ms / r.time_ms : 0;
        }

        // ---- Print JSON ----
        std::cout << "{\"algorithm\":\"" << algorithm << "\""
                  << ",\"threads\":" << (algorithm == "shared" || algorithm == "hda" ? cfg.threads : algorithm == "bidir_parallel" ? 2 : 1)
                  << ",\"hash\":\"" << (algorithm == "hda" ? cfg.hash : "-") << "\""
                  << ",\"tile\":" << cfg.tile
                  << ",\"batch\":" << cfg.batch
                  << ",\"flush_interval\":" << cfg.flush_interval
                  << ",\"prune\":" << (cfg.prune ? "true" : "false")
                  << ",\"nodes\":" << graph.size()
                  << ",\"found\":" << (r.found ? "true" : "false")
                  << ",\"path_cost\":" << json_number(r.cost)
                  << ",\"path_vertices\":" << r.path.size()
                  << ",\"path\":" << json_path(r.path)
                  << ",\"time_ms\":" << json_number(r.time_ms)
                  << ",\"expanded_nodes\":" << r.stats.expanded
                  << ",\"forward_expanded\":" << r.stats.forward_expanded
                  << ",\"backward_expanded\":" << r.stats.backward_expanded
                  << ",\"road_heuristic_scale\":" << json_number(graph.road_heuristic_scale)
                  << ",\"generated_nodes\":" << r.stats.generated
                  << ",\"improved_nodes\":" << r.stats.improved
                  << ",\"stale_entries\":" << r.stats.stale
                  << ",\"pruned_nodes\":" << r.stats.pruned
                  << ",\"remote_messages\":" << r.stats.remote
                  << ",\"remote_batches\":" << r.stats.remote_batches
                  << ",\"inbox_received\":" << r.stats.inbox_received
                  << ",\"assigned_nodes\":" << r.stats.assigned
                  << ",\"wait_ms_sum\":" << json_number(r.stats.wait_ms)
                  << ",\"profile_heuristic\":" << (astar::profile_heuristic ? "true" : "false")
                  << ",\"heuristic_calls\":" << r.stats.heuristic_calls
                  << ",\"heuristic_ms\":" << json_number(heuristic_ms)
                  << ",\"heuristic_share\":" << json_number(heuristic_share)
                  << ",\"timer_overhead_ns\":" << json_number(timer_overhead_ns)
                  << ",\"communication_overhead\":" << json_number(co)
                  << ",\"load_balance\":" << json_number(balance)
                  << ",\"load_balance_expanded\":" << json_number(balance_expanded)
                  << ",\"per_worker_expanded\":" << json_vec(r.stats.per_worker_expanded)
                  << ",\"per_worker_generated\":" << json_vec(r.stats.per_worker_generated)
                  << ",\"per_worker_assigned\":" << json_vec(r.stats.per_worker_assigned)
                  << ",\"verified\":" << (verified ? "true" : "false")
                  << "}\n";
        return 0;
    } 
    catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 2;
    }
}
