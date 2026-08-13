#ifndef MULTINET_SURFACE_TOPOLOGY_H
#define MULTINET_SURFACE_TOPOLOGY_H

#include "surface_frame.h"

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

// Transports the flat presentation basis across one authoritative edge. The
// transport is deliberately chart/topology based; it does not use cubemap
// face normals, so local Y remains presentation altitude.
[[nodiscard]] bool try_transport_flat_surface_frame(
    const SurfaceFrame& source,
    SurfaceEdge source_edge,
    SurfaceFrame& destination
) noexcept;

// Builds the deterministic flat presentation basis for a face by walking the
// authoritative edge graph from PositiveX. This keeps the Godot adapter from
// growing a second six-face orientation table.
[[nodiscard]] bool try_make_flat_surface_frame_for_face(
    SurfaceFace face,
    SurfaceFrame& out_frame
) noexcept;

constexpr uint32_t pack_edge_transition_for_glsl(const EdgeTransition& trans) noexcept {
    uint32_t dst_f = static_cast<uint32_t>(trans.destination_face);
    uint32_t dst_e = static_cast<uint32_t>(trans.destination_edge);
    uint32_t p_sign = (trans.parameter_sign > 0) ? 1u : 0u;
    return (dst_f & 0xFu) | ((dst_e & 0xFu) << 4u) | (p_sign << 8u);
}

} // namespace Multinet

#endif // MULTINET_SURFACE_TOPOLOGY_H
