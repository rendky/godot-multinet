#ifndef MULTINET_SURFACE_TOPOLOGY_H
#define MULTINET_SURFACE_TOPOLOGY_H

#include <cstdint>

namespace Multinet {

constexpr uint32_t TOPOLOGY_VERSION_SIX_FACE = 1;

enum class SurfaceEdge : uint8_t {
    NegativeU = 0, // Left
    PositiveU = 1, // Right
    NegativeV = 2, // Top
    PositiveV = 3  // Bottom
};

struct EdgeTransition {
    uint8_t source_face;
    SurfaceEdge source_edge;
    
    uint8_t destination_face;
    SurfaceEdge destination_edge;
    
    int8_t dest_fixed_coordinate;      // -1 or 1
    
    uint8_t source_parameter_axis;     // 0 for U, 1 for V
    uint8_t destination_parameter_axis;// 0 for U, 1 for V
    
    int8_t parameter_sign;             // 1 or -1
    
    int8_t tangent_signed_permutation; // 1 or -1
    int8_t inward_axis_signed_permutation; // 1 or -1
};

const EdgeTransition& get_edge_transition(uint8_t face, SurfaceEdge edge) noexcept;

} // namespace Multinet

#endif // MULTINET_SURFACE_TOPOLOGY_H
