#include "astar.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <stdexcept>

namespace astar {
namespace {
constexpr double EPS=1e-9;
struct Entry { double f,g;Vertex v; };
struct Compare {bool operator()(const Entry& a,const Entry& b) const {
    if(a.f!=b.f)return a.f>b.f;
    if(a.g!=b.g)return a.g<b.g; // deterministic preference for larger g
    return a.v>b.v;
}};
using Queue=std::priority_queue<Entry,std::vector<Entry>,Compare>;
struct State {double g=INF;Vertex parent=NONE;};
using Clock=std::chrono::steady_clock;
double elapsed(Clock::time_point t){return std::chrono::duration<double,std::milli>(Clock::now()-t).count();}
std::vector<Vertex> path_to(const std::vector<State>& states,Vertex s,Vertex t){
    std::vector<Vertex> out;Vertex v=t;
    for(size_t i=0;i<=states.size();++i){out.push_back(v);if(v==s)break;v=states[v].parent;if(v==NONE)break;}
    if(out.back()!=s)return {};
    std::reverse(out.begin(),out.end());return out;
}
void drop_stale(Queue& q,const std::vector<State>& state,SearchStats& stats){
    while(!q.empty() && q.top().g>state[q.top().v].g+EPS){q.pop();++stats.stale;}
}
}
SearchResult sequential(const Graph& g,Vertex s,Vertex t){
    auto start=Clock::now();SearchResult r;std::vector<State> state(g.size());Queue q;
    state[s].g=0;q.push({g.heuristic(s,t),0,s});
    while(!q.empty()){
        auto e=q.top();q.pop();if(e.g>state[e.v].g+EPS){++r.stats.stale;continue;}
        if(e.v==t){r.found=true;r.cost=e.g;break;}
        ++r.stats.expanded;
        for(const auto& ed:g.adj[e.v]){
            ++r.stats.generated;double ng=e.g+ed.cost;
            if(ng+EPS<state[ed.to].g){state[ed.to]={ng,e.v};++r.stats.improved;q.push({ng+g.heuristic(ed.to,t),ng,ed.to});}
        }
    }
    if(r.found)r.path=path_to(state,s,t);
    r.time_ms=elapsed(start);r.stats.per_worker_expanded={r.stats.expanded};r.stats.per_worker_generated={r.stats.generated};return r;
}
SearchResult dijkstra(const Graph& g,Vertex s,Vertex t){
    auto start=Clock::now();SearchResult r;std::vector<State> state(g.size());Queue q;
    state[s].g=0;q.push({0,0,s});
    while(!q.empty()){
        auto e=q.top();q.pop();if(e.g>state[e.v].g+EPS){++r.stats.stale;continue;}
        if(e.v==t){r.found=true;r.cost=e.g;break;}
        ++r.stats.expanded;
        for(const auto& ed:g.adj[e.v]){
            ++r.stats.generated;double ng=e.g+ed.cost;
            if(ng+EPS<state[ed.to].g){state[ed.to]={ng,e.v};++r.stats.improved;q.push({ng,ng,ed.to});}
        }
    }
    if(r.found)r.path=path_to(state,s,t);
    r.time_ms=elapsed(start);r.stats.per_worker_expanded={r.stats.expanded};r.stats.per_worker_generated={r.stats.generated};return r;
}
SearchResult bidirectional(const Graph& g,Vertex s,Vertex t){
    auto start=Clock::now();SearchResult r;
    if(s==t){r.found=true;r.cost=0;r.path={s};r.time_ms=elapsed(start);r.stats.per_worker_expanded={0,0};return r;}
    std::vector<State> fw(g.size()),bw(g.size());Queue qf,qb;
    fw[s].g=0;bw[t].g=0;qf.push({g.heuristic(s,t),0,s});qb.push({g.heuristic(t,s),0,t});
    double U=INF;Vertex meeting=NONE;uint64_t nforward=0,nbackward=0;
    auto meet=[&](Vertex v){
        if(std::isfinite(fw[v].g)&&std::isfinite(bw[v].g)){
            double cand=fw[v].g+bw[v].g;
            if(cand+EPS<U){U=cand;meeting=v;}
        }
    };
    while(true){
        drop_stale(qf,fw,r.stats);drop_stale(qb,bw,r.stats);
        if(qf.empty()||qb.empty())break; // if no solution, a direction exhausted => unreachable
        // Each frontier is a valid A* search. Once either lower bound reaches U,
        // that direction cannot improve the complete incumbent path.
        if(std::isfinite(U)&&(qf.top().f>=U-EPS||qb.top().f>=U-EPS))break;
        // Prefer the smaller frontier f; alternate at equality.
        bool forward=(qf.top().f<=qb.top().f);
        auto& q=forward?qf:qb;auto& here=forward?fw:bw;
        const auto& adjacency=forward?g.adj:g.reverse;
        Vertex destination=forward?t:s;
        auto e=q.top();q.pop();meet(e.v);
        if(std::isfinite(U)&&e.f>=U-EPS){++r.stats.pruned;continue;}
        ++r.stats.expanded;
        if(forward)++nforward;else ++nbackward;
        for(auto& ed:adjacency[e.v]){
            ++r.stats.generated;double ng=e.g+ed.cost;
            if(ng+EPS<here[ed.to].g){
                here[ed.to]={ng,e.v};++r.stats.improved;
                q.push({ng+g.heuristic(ed.to,destination),ng,ed.to});meet(ed.to);
            }
        }
    }
    if(meeting!=NONE){
        r.found=true;r.cost=U;
        auto a=path_to(fw,s,meeting); // start -> meeting
        std::vector<Vertex> b;Vertex u=meeting;
        for(size_t i=0;i<=g.size();++i){if(u==t)break;u=bw[u].parent;if(u==NONE)break;b.push_back(u);}
        if(!a.empty()&&(a.back()==meeting)&&(meeting==t||(!b.empty()&&b.back()==t))){
            r.path=std::move(a);r.path.insert(r.path.end(),b.begin(),b.end());
        }
        // Parent chains might have improved after U was recorded.
        // Recompute actual path cost using real directed edges.
        if(!r.path.empty()){
            double cost=0;for(size_t i=1;i<r.path.size();++i){
                double w=INF;for(const auto& ed:g.adj[r.path[i-1]])if(ed.to==r.path[i])w=std::min(w,ed.cost);
                cost+=w;
            }r.cost=cost;
        }
    }
    r.time_ms=elapsed(start);r.stats.forward_expanded=nforward;r.stats.backward_expanded=nbackward;
    r.stats.per_worker_expanded={r.stats.expanded};return r;
}
} // namespace astar
