#include "astar.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace astar {
namespace {
constexpr double PI=3.14159265358979323846;
double earth_distance(const Point& a, const Point& b) {
    const double r=6371009.0, dlat=(b.latitude-a.latitude)*PI/180.0;
    const double dlon=(b.longitude-a.longitude)*PI/180.0;
    const double x=std::sin(dlat/2)*std::sin(dlat/2)+
        std::cos(a.latitude*PI/180.0)*std::cos(b.latitude*PI/180.0)*std::sin(dlon/2)*std::sin(dlon/2);
    return r*2*std::asin(std::min(1.0,std::sqrt(x)));
}
std::vector<std::string> columns(const std::string& row) {
    std::vector<std::string> result;
    std::stringstream ss(row); std::string s;
    while(std::getline(ss,s,',')) result.push_back(s);
    return result;
}
void add_edge(Graph& g,Vertex u,Vertex v,double cost) {
    g.adj[u].push_back({v,cost});g.reverse[v].push_back({u,cost});
}
}
double Graph::heuristic(Vertex u, Vertex goal) const {
    if(grid) return std::abs(point[u].x-point[goal].x)+std::abs(point[u].y-point[goal].y);
    return road_heuristic_scale*earth_distance(point[u],point[goal]);
}
Graph make_grid(int side,uint64_t seed,double density) {
    if(side<2 || side>8192 || density<0 || density>=1) throw std::runtime_error("invalid grid side or density");
    size_t n=size_t(side)*side;
    Graph g;g.grid=true;g.grid_side=side;g.adj.resize(n);g.reverse.resize(n);g.point.resize(n);
    std::mt19937_64 rng(seed);std::uniform_real_distribution<double> uniform(0,1);
    std::vector<uint8_t> blocked(n);
    for(size_t i=0;i<n;++i) {
        g.point[i].x=int64_t(i%side);g.point[i].y=int64_t(i/side);
        blocked[i]=(uniform(rng)<density);
    }
    // Fixed anchors are always traversable, so all query pairs use the same graph.
    for(int row : {0,side/4,side/2,(3*side)/4,side-1})
        for(int col : {0,side/4,side/2,(3*side)/4,side-1}) blocked[size_t(row)*side+col]=false;
    for(int y=0;y<side;++y) for(int x=0;x<side;++x) {
        Vertex u=Vertex(y*side+x);
        if(blocked[u]) continue;
        for(auto delta : {std::pair<int,int>{0,1},{0,-1},{1,0},{-1,0}}) {
            int nx=x+delta.first,ny=y+delta.second;
            if(nx>=0&&nx<side&&ny>=0&&ny<side) {
                Vertex v=Vertex(ny*side+nx);
                if(!blocked[v]) add_edge(g,u,v,1.0);
            }
        }
    }
    return g;
}
Graph load_osm_csv(const std::string& nodes,const std::string& edges) {
    std::ifstream ni(nodes);if(!ni) throw std::runtime_error("cannot open nodes CSV: "+nodes);
    std::string line;std::getline(ni,line);
    if(line.find("id")==std::string::npos || line.find("lat")==std::string::npos)throw std::runtime_error("nodes header must be id,lat,lon");
    Graph g;std::vector<bool> seen;
    while(std::getline(ni,line)) {
        if(line.empty())continue;
        auto c=columns(line);if(c.size()<3)throw std::runtime_error("bad nodes CSV row");
        size_t id=std::stoul(c[0]);if(id>5000000)throw std::runtime_error("node ID too large");
        if(id>=g.point.size()){g.point.resize(id+1);seen.resize(id+1,false);}
        if(seen[id])throw std::runtime_error("duplicate node ID");
        g.point[id].latitude=std::stod(c[1]);g.point[id].longitude=std::stod(c[2]);seen[id]=true;
    }
    if(g.point.empty()||std::find(seen.begin(),seen.end(),false)!=seen.end()) throw std::runtime_error("node IDs must be contiguous from zero");
    g.adj.resize(g.point.size());g.reverse.resize(g.point.size());
    double lat_min=90,lon_min=180,lat_avg=0;
    for(const auto& p:g.point){lat_min=std::min(lat_min,p.latitude);lon_min=std::min(lon_min,p.longitude);lat_avg+=p.latitude;}
    lat_avg/=g.point.size();
    for(auto& p:g.point) {
        // 100-meter feature cells for spatial hashing; NOT used for path costs.
        p.x=std::llround((p.longitude-lon_min)*111320*std::cos(lat_avg*PI/180.0)/100);
        p.y=std::llround((p.latitude-lat_min)*111320/100);
    }
    std::ifstream ei(edges);if(!ei)throw std::runtime_error("cannot open edges CSV: "+edges);
    std::getline(ei,line);
    if(line.find("from")==std::string::npos || line.find("weight")==std::string::npos)throw std::runtime_error("edges header must be from,to,weight");
    double min_ratio=1.0;
    while(std::getline(ei,line)) {
        if(line.empty())continue;
        auto c=columns(line);if(c.size()<3)throw std::runtime_error("bad edges CSV row");
        Vertex u=Vertex(std::stoul(c[0])),v=Vertex(std::stoul(c[1]));double w=std::stod(c[2]);
        if(u>=g.size()||v>=g.size()||!std::isfinite(w)||w<0)throw std::runtime_error("invalid edge, vertex ID or weight");
        add_edge(g,u,v,w);
        double distance=earth_distance(g.point[u],g.point[v]);
        if(distance>1e-8)min_ratio=std::min(min_ratio,w/distance);
    }
    // This global factor preserves h(u)<=w(u,v)+h(v), even for imperfect edge data.
    g.road_heuristic_scale=std::max(0.0,std::min(1.0,min_ratio*(1.0-1e-10)));
    return g;
}
bool validate_path(const Graph& g,Vertex s,Vertex t,const SearchResult& r,std::string& error) {
    if(!r.found) {
        if(!r.path.empty()){error="unreachable result has path";return false;}
        return true;
    }
    if(r.path.empty()||r.path.front()!=s||r.path.back()!=t){error="invalid path endpoints";return false;}
    if(r.path.size()>g.size()+1){error="path cycle detected";return false;}
    double total=0;
    for(size_t i=1;i<r.path.size();++i) {
        auto u=r.path[i-1],v=r.path[i];double best=INF;
        for(auto& e:g.adj[u])if(e.to==v)best=std::min(best,e.cost);
        if(!std::isfinite(best)){error="nonexistent directed path edge";return false;}
        total+=best;
    }
    if(std::fabs(total-r.cost)>1e-7*std::max({1.,total,r.cost})) {
        error="path edge weights do not sum to reported cost";return false;
    }
    return true;
}
} // namespace astar
