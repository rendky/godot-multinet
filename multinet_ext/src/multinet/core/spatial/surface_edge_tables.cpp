#include "multinet/core/spatial/surface_topology.h"

namespace Multinet {

static const EdgeTransition EDGE_TABLE[24] = {
    { 0, SurfaceEdge::NegativeU, 4, SurfaceEdge::PositiveU, 1, 1, 1, 1, 1, -1 },
    { 0, SurfaceEdge::PositiveU, 5, SurfaceEdge::NegativeU, -1, 1, 1, 1, 1, -1 },
    { 0, SurfaceEdge::NegativeV, 2, SurfaceEdge::PositiveU, 1, 0, 1, -1, -1, -1 },
    { 0, SurfaceEdge::PositiveV, 3, SurfaceEdge::PositiveU, 1, 0, 1, 1, 1, -1 },
    { 1, SurfaceEdge::NegativeU, 5, SurfaceEdge::PositiveU, 1, 1, 1, 1, 1, -1 },
    { 1, SurfaceEdge::PositiveU, 4, SurfaceEdge::NegativeU, -1, 1, 1, 1, 1, -1 },
    { 1, SurfaceEdge::NegativeV, 2, SurfaceEdge::NegativeU, -1, 0, 1, 1, 1, -1 },
    { 1, SurfaceEdge::PositiveV, 3, SurfaceEdge::NegativeU, -1, 0, 1, -1, -1, -1 },
    { 2, SurfaceEdge::NegativeU, 1, SurfaceEdge::NegativeV, -1, 1, 0, 1, 1, -1 },
    { 2, SurfaceEdge::PositiveU, 0, SurfaceEdge::NegativeV, -1, 1, 0, -1, -1, -1 },
    { 2, SurfaceEdge::NegativeV, 5, SurfaceEdge::NegativeV, -1, 0, 0, -1, -1, -1 },
    { 2, SurfaceEdge::PositiveV, 4, SurfaceEdge::NegativeV, -1, 0, 0, 1, 1, -1 },
    { 3, SurfaceEdge::NegativeU, 1, SurfaceEdge::PositiveV, 1, 1, 0, -1, -1, -1 },
    { 3, SurfaceEdge::PositiveU, 0, SurfaceEdge::PositiveV, 1, 1, 0, 1, 1, -1 },
    { 3, SurfaceEdge::NegativeV, 4, SurfaceEdge::PositiveV, 1, 0, 0, 1, 1, -1 },
    { 3, SurfaceEdge::PositiveV, 5, SurfaceEdge::PositiveV, 1, 0, 0, -1, -1, -1 },
    { 4, SurfaceEdge::NegativeU, 1, SurfaceEdge::PositiveU, 1, 1, 1, 1, 1, -1 },
    { 4, SurfaceEdge::PositiveU, 0, SurfaceEdge::NegativeU, -1, 1, 1, 1, 1, -1 },
    { 4, SurfaceEdge::NegativeV, 2, SurfaceEdge::PositiveV, 1, 0, 0, 1, 1, -1 },
    { 4, SurfaceEdge::PositiveV, 3, SurfaceEdge::NegativeV, -1, 0, 0, 1, 1, -1 },
    { 5, SurfaceEdge::NegativeU, 0, SurfaceEdge::PositiveU, 1, 1, 1, 1, 1, -1 },
    { 5, SurfaceEdge::PositiveU, 1, SurfaceEdge::NegativeU, -1, 1, 1, 1, 1, -1 },
    { 5, SurfaceEdge::NegativeV, 2, SurfaceEdge::NegativeV, -1, 0, 0, -1, -1, -1 },
    { 5, SurfaceEdge::PositiveV, 3, SurfaceEdge::PositiveV, 1, 0, 0, -1, -1, -1 }
};

const EdgeTransition& get_edge_transition(uint8_t face, SurfaceEdge edge) noexcept {
    return EDGE_TABLE[face * 4 + static_cast<uint8_t>(edge)];
}

} // namespace Multinet
