#include <iostream>
#include <cmath>
#include <vector>
#include <cstdint>
#include <random>

#include "multinet/core/spatial/surface_topology.h"
#include "multinet/core/spatial/surface_projection.h"
#include "multinet/core/spatial/surface_address.h"
#include "multinet/core/spatial/world_manifests.h"

using namespace Multinet;

struct Vec3 {
    double x, y, z;
    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
};

struct FaceFrame { Vec3 n, u, v; };
const FaceFrame FACES[6] = {
    { { 1, 0, 0}, { 0, 0,-1}, { 0,-1, 0} }, // 0: +X
    { {-1, 0, 0}, { 0, 0, 1}, { 0,-1, 0} }, // 1: -X
    { { 0, 1, 0}, { 1, 0, 0}, { 0, 0, 1} }, // 2: +Y
    { { 0,-1, 0}, { 1, 0, 0}, { 0, 0,-1} }, // 3: -Y
    { { 0, 0, 1}, { 1, 0, 0}, { 0,-1, 0} }, // 4: +Z
    { { 0, 0,-1}, {-1, 0, 0}, { 0,-1, 0} }  // 5: -Z
};

void require(bool cond, const char* msg) {
    if(!cond) {
        std::cerr << "FAILURE: " << msg << "\n";
        exit(1);
    }
}

int main() {
    std::cout << "## canonicalize_surface_address Fixture\n";
    
    // 1. Correct the world-scale fixture
    WorldScaleInput w_input;
    w_input.area_equivalent_side_m = 5000000;
    WorldScaleManifest scale = build_world_scale_manifest(w_input);
    
    int64_t extent = static_cast<int64_t>(std::rint(scale.area_equivalent_face_extent_m * 500.0));
    std::cout << "- Calculated chart half extent mm: " << extent << "\n";
    require(extent == 1020620726LL, "Chart half extent doesn't match expected value 1,020,620,726 mm");
    
    // 4. Preserve complete identity
    SurfaceAddress zero_addr{};
    zero_addr.u_mm = 0; zero_addr.v_mm = 0; zero_addr.altitude_mm = 12345;
    zero_addr.topology_version = scale.topology_version;
    zero_addr.projection_version = scale.projection_version;
    SurfaceAddress canon_zero = canonicalize_surface_address(zero_addr, scale);
    require(canon_zero.altitude_mm == 12345, "Altitude not preserved");
    require(canon_zero.topology_version == scale.topology_version, "Topology version not preserved");
    require(canon_zero.projection_version == scale.projection_version, "Projection version not preserved");
    
    SurfaceAddress bad_version = zero_addr;
    bad_version.topology_version = 999;
    SurfaceAddress canon_bad = canonicalize_surface_address(bad_version, scale);
    require(static_cast<uint8_t>(canon_bad.face) == 255, "Invalid version not rejected");
    
    // 2. Exact edge-alias identity
    int edges_tested = 0;
    for(int f=0; f<6; ++f) {
        for(int e=0; e<4; ++e) {
            SurfaceEdge edge = static_cast<SurfaceEdge>(e);
            const EdgeTransition& t = get_edge_transition(f, edge);
            
            int64_t test_params[] = { -extent, -extent/3, 0, extent/2, extent };
            for(int64_t param : test_params) {
                SurfaceAddress src_point{};
                src_point.face = static_cast<SurfaceFace>(f);
                src_point.u_mm = (e==0) ? -extent : (e==1) ? extent : param;
                src_point.v_mm = (e==2) ? -extent : (e==3) ? extent : param;
                src_point.topology_version = scale.topology_version;
                src_point.projection_version = scale.projection_version;
                
                SurfaceAddress dst_point{};
                dst_point.face = static_cast<SurfaceFace>(t.destination_face);
                int64_t dest_param = param * t.parameter_sign;
                int64_t fixed_coord = t.dest_fixed_coordinate * extent;
                if (t.destination_parameter_axis == 0) {
                    dst_point.u_mm = dest_param; dst_point.v_mm = fixed_coord;
                } else {
                    dst_point.u_mm = fixed_coord; dst_point.v_mm = dest_param;
                }
                dst_point.topology_version = scale.topology_version;
                dst_point.projection_version = scale.projection_version;
                
                SurfaceAddress canon_src = canonicalize_surface_address(src_point, scale);
                SurfaceAddress canon_dst = canonicalize_surface_address(dst_point, scale);
                
                require(canon_src == canon_dst, "Exact edge-alias identity mismatch");
            }
            
            // 3. Bound overshoot work: +-1mm overshoot test
            SurfaceAddress overshoot1{};
            overshoot1.face = static_cast<SurfaceFace>(f);
            overshoot1.u_mm = (e==0) ? -extent : (e==1) ? extent : 0;
            overshoot1.v_mm = (e==2) ? -extent : (e==3) ? extent : 0;
            overshoot1.topology_version = scale.topology_version;
            overshoot1.projection_version = scale.projection_version;
            
            if(e==0) overshoot1.u_mm -= 1;
            if(e==1) overshoot1.u_mm += 1;
            if(e==2) overshoot1.v_mm -= 1;
            if(e==3) overshoot1.v_mm += 1;
            SurfaceAddress canon_o1 = canonicalize_surface_address(overshoot1, scale);
            // After canonicalization it should land exactly inside the adjacent face (or corner resolution if endpoint)
            require(static_cast<uint8_t>(canon_o1.face) != 255, "1mm overshoot failed");
            
            edges_tested++;
        }
    }
    std::cout << "- 24 Directed Edges Evaluated for Edge-Alias Identity\n";
    
    // 3. Bound overshoot work: Multi-face overshoot and maximum accepted overshoot
    SurfaceAddress large;
    large.face = SurfaceFace::PositiveX;
    large.u_mm = 2 * extent + 1234;
    large.v_mm = -extent + 5678; // One representative multi-face overshoot
    large.topology_version = scale.topology_version;
    large.projection_version = scale.projection_version;
    
    SurfaceAddress large_canon = canonicalize_surface_address(large, scale);
    require(static_cast<uint8_t>(large_canon.face) != 255, "Multi-face overshoot failed");
    require(large_canon.u_mm >= -extent && large_canon.u_mm <= extent, "Multi-face overshoot U bounds");
    require(large_canon.v_mm >= -extent && large_canon.v_mm <= extent, "Multi-face overshoot V bounds");
    require(canonicalize_surface_address(large_canon, scale) == large_canon, "Idempotence failed on overshoot");
    
    // The old fixed 3*extent guard is gone. A representable five-extent input
    // is accepted because the safety bound is derived from the actual input.
    SurfaceAddress farther = large;
    farther.u_mm = 5 * extent;
    SurfaceAddress canon_farther = canonicalize_surface_address(farther, scale);
    require(static_cast<uint8_t>(canon_farther.face) != 255, "Derived representable overshoot was rejected");
    require(canonicalize_surface_address(canon_farther, scale) == canon_farther,
        "Idempotence failed on derived overshoot");

    SurfaceAddress invalid_face = large;
    invalid_face.face = static_cast<SurfaceFace>(255);
    require(static_cast<uint8_t>(canonicalize_surface_address(invalid_face, scale).face) == 255,
        "Invalid canonical face was admitted");
    
    std::cout << "- Bounded Overshoot Validated\n";
    std::cout << "- Identity Preservation Validated\n";
    
    std::cout << "\nSTATUS: PASSED WITH EVIDENCE\n";
    return 0;
}
