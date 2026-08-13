#include "multinet/core/spatial/surface_topology.h"

#include <array>
#include <queue>

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

namespace {

Vec3d axis_for_parameter(const Basis3d& basis, uint8_t axis) noexcept {
    return axis == 0 ? basis.u_axis : basis.v_axis;
}

void set_parameter_axis(Basis3d& basis, uint8_t axis, const Vec3d& value) noexcept {
    if (axis == 0) basis.u_axis = value;
    else basis.v_axis = value;
}

Vec3d edge_inward_axis(const Basis3d& basis, SurfaceEdge edge) noexcept {
    switch (edge) {
        case SurfaceEdge::NegativeU: return basis.u_axis;
        case SurfaceEdge::PositiveU: return { -basis.u_axis.x, -basis.u_axis.y, -basis.u_axis.z };
        case SurfaceEdge::NegativeV: return basis.v_axis;
        case SurfaceEdge::PositiveV: return { -basis.v_axis.x, -basis.v_axis.y, -basis.v_axis.z };
    }
    return basis.u_axis;
}

Vec3d negate(const Vec3d& value) noexcept {
    return { -value.x, -value.y, -value.z };
}

Vec3d scale(const Vec3d& value, int sign) noexcept {
    return sign < 0 ? negate(value) : value;
}

} // namespace

bool try_transport_flat_surface_frame(
    const SurfaceFrame& source,
    SurfaceEdge source_edge,
    SurfaceFrame& destination
) noexcept {
    if (!is_valid_surface_face(source.origin.face)) return false;

    const EdgeTransition& transition = get_edge_transition(
        static_cast<uint8_t>(source.origin.face), source_edge);
    if (!is_valid_surface_face(static_cast<SurfaceFace>(transition.destination_face))) return false;

    const Vec3d source_along = axis_for_parameter(source.tangent_basis, transition.source_parameter_axis);
    const Vec3d source_inward = edge_inward_axis(source.tangent_basis, source_edge);
    const Vec3d destination_along = scale(source_along, transition.parameter_sign);
    const Vec3d destination_inward = scale(source_inward, transition.inward_axis_signed_permutation);

    destination = source;
    destination.origin.face = static_cast<SurfaceFace>(transition.destination_face);
    set_parameter_axis(destination.tangent_basis, transition.destination_parameter_axis, destination_along);

    // The fixed destination coordinate is the inward axis. The edge sign says
    // whether positive U/V points into the destination chart.
    const bool fixed_is_negative =
        transition.destination_edge == SurfaceEdge::PositiveU ||
        transition.destination_edge == SurfaceEdge::PositiveV;
    const Vec3d destination_fixed = fixed_is_negative ? negate(destination_inward) : destination_inward;
    set_parameter_axis(destination.tangent_basis,
        transition.destination_parameter_axis == 0 ? 1 : 0,
        destination_fixed);

    // The flat presentation keeps altitude vertical. The transported U/V
    // vectors above are a signed permutation in the horizontal plane.
    destination.tangent_basis.up_axis = source.tangent_basis.up_axis;
    return true;
}

bool try_make_flat_surface_frame_for_face(
    SurfaceFace face,
    SurfaceFrame& out_frame
) noexcept {
    if (!is_valid_surface_face(face)) return false;

    SurfaceFrame identity{};
    identity.origin.face = SurfaceFace::PositiveX;
    identity.tangent_basis.u_axis = { 1.0, 0.0, 0.0 };
    identity.tangent_basis.up_axis = { 0.0, 1.0, 0.0 };
    identity.tangent_basis.v_axis = { 0.0, 0.0, 1.0 };
    if (face == SurfaceFace::PositiveX) {
        out_frame = identity;
        return true;
    }

    std::array<bool, 6> visited{};
    std::array<SurfaceFrame, 6> frames{};
    std::queue<uint8_t> pending;
    visited[0] = true;
    frames[0] = identity;
    pending.push(0);

    while (!pending.empty()) {
        const uint8_t source_face = pending.front();
        pending.pop();
        for (uint8_t edge_index = 0; edge_index < 4; ++edge_index) {
            const SurfaceEdge edge = static_cast<SurfaceEdge>(edge_index);
            const EdgeTransition& transition = get_edge_transition(source_face, edge);
            const uint8_t destination_face = transition.destination_face;
            if (visited[destination_face]) continue;

            SurfaceFrame transported;
            if (!try_transport_flat_surface_frame(frames[source_face], edge, transported)) return false;
            visited[destination_face] = true;
            frames[destination_face] = transported;
            pending.push(destination_face);
        }
    }

    if (!visited[static_cast<uint8_t>(face)]) return false;
    out_frame = frames[static_cast<uint8_t>(face)];
    return true;
}

} // namespace Multinet
