#include "astar.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

void check(const astar::Graph& graph,astar::Vertex s,astar::Vertex t){
    auto ref=astar::dijkstra(graph,s,t);
    auto baseline=astar::sequential(graph,s,t);
    auto bi=astar::bidirectional(graph,s,t);
    std::vector<astar::SearchResult> variants={baseline,bi};
    for(int n : {1,2,4}){
        astar::Config config;config.threads=n;variants.push_back(astar::shared_queue(graph,s,t,config));
        for(auto hash : {"mod","mixed","zobrist","abstract","azobrist"}){
            config.hash=hash;config.batch=4;config.tile=4;
            variants.push_back(astar::hda(graph,s,t,config));
        }
    }
    for(const auto& r:variants){
        std::string err;
        if(!astar::validate_path(graph,s,t,r,err))throw std::runtime_error("invalid path: "+err);
        if(r.found!=ref.found)throw std::runtime_error("reachability mismatch");
        if(r.found&&std::fabs(r.cost-ref.cost)>1e-6*std::max(1.0,ref.cost))
            throw std::runtime_error("shortest cost mismatch");
    }
}
int main(){
    try{
        size_t count=0;
        for(int n : {7,11,17})for(int seed : {7,42,91})for(double d : {0.0,0.15,0.3}){
            auto g=astar::make_grid(n,seed,d);
            for(auto pair : {std::pair<int,int>{0,n*n-1},{n/2*n+n/2,0},{0,0}}){
                check(g,pair.first,pair.second);++count;
            }
        }
        std::cout<<"PASS "<<count<<" grid test configurations x 20 variants = "<<(count*20)<<" validations\n";
    }catch(const std::exception& ex){std::cerr<<"FAIL "<<ex.what()<<"\n";return 1;}
    return 0;
}
