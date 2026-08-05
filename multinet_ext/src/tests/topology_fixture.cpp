#include <iostream>
#include <cmath>
#include <vector>
#include <cstdint>

#include "multinet/core/spatial/surface_topology.h"
#include "multinet/core/spatial/surface_projection.h"

using namespace Multinet;

void require(bool cond, const char* msg) {
    if(!cond) {
        std::cerr << "FAILURE: " << msg << "\n";
        exit(1);
    }
}

int main() {
    std::cout << "## Topology Edge Tables Fixture\n";
    
    int edges_tested = 0;
    
    for(int f=0; f<6; ++f) {
        for(int e=0; e<4; ++e) {
            SurfaceEdge edge = static_cast<SurfaceEdge>(e);
            const EdgeTransition& t = get_edge_transition(f, edge);
            
            require(t.source_face == f, "Source face mismatch");
            require(t.source_edge == edge, "Source edge mismatch");
            
            // Validate reverse mapping exists and restores everything
            const EdgeTransition& rev = get_edge_transition(t.destination_face, t.destination_edge);
            require(rev.destination_face == f, "Reverse dest face mismatch");
            require(rev.destination_edge == edge, "Reverse dest edge mismatch");
            require(rev.dest_fixed_coordinate == ((e==0 || e==2) ? -1 : 1), "Reverse fixed coord mismatch");
            require(rev.parameter_sign == t.parameter_sign, "Reverse parameter sign mismatch");
            
            // Check 3D geometric agreement along the entire edge
            for(int i=0; i<=1024; ++i) {
                double param = -1.0 + 2.0 * i / 1024.0;
                
                double src_u = (e==0) ? -1.0 : (e==1) ? 1.0 : param;
                double src_v = (e==2) ? -1.0 : (e==3) ? 1.0 : param;
                
                double dest_param = param * t.parameter_sign;
                double dest_u = (t.destination_parameter_axis == 0) ? dest_param : t.dest_fixed_coordinate;
                double dest_v = (t.destination_parameter_axis == 1) ? dest_param : t.dest_fixed_coordinate;
                
                FramePosition64 p_src = ProjectionCOBE::map_forward(f, src_u, src_v);
                FramePosition64 p_dst = ProjectionCOBE::map_forward(t.destination_face, dest_u, dest_v);
                
                double dist = std::sqrt((p_src.x-p_dst.x)*(p_src.x-p_dst.x) + 
                                        (p_src.y-p_dst.y)*(p_src.y-p_dst.y) + 
                                        (p_src.z-p_dst.z)*(p_src.z-p_dst.z));
                                        
                require(dist < 1e-14, "Edge trajectory mismatch in 3D");
            }
            edges_tested++;
        }
    }
    
    std::cout << "- 24 Directed Edges Evaluated: " << edges_tested << " passed\n";
    std::cout << "- Reverse transitions verified\n";
    std::cout << "- 1025 deterministic positions verified\n";
    std::cout << "\nSTATUS: PASSED WITH EVIDENCE\n";
    return 0;
}
