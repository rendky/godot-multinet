#include "surface_address.h"
#include "surface_frame.h"
#include "surface_topology.h"
#include "world_manifests.h"
#include "multinet/core/coordinates.h"

#include <cmath>
#include <algorithm>

namespace Multinet {

[[nodiscard]] bool try_surface_to_frame(
    const SurfacePosition64& position,
    const SurfaceFrame& frame,
    const WorldScaleManifest& scale,
    FramePosition64& out_position
) noexcept {
    if (!is_valid_surface_face(position.face) || !is_valid_surface_face(frame.origin.face)) return false;
    if (position.topology_version != scale.topology_version || position.topology_version != frame.topology_version) return false;
    if (position.projection_version != scale.projection_version || position.projection_version != frame.projection_version) return false;
    
    if (!std::isfinite(position.u_m) || !std::isfinite(position.v_m) || !std::isfinite(position.altitude_m)) return false;
    if (!std::isfinite(frame.origin.u_m) || !std::isfinite(frame.origin.v_m) || !std::isfinite(frame.origin.altitude_m)) return false;
    if (!std::isfinite(frame.tangent_basis.u_axis.x) || !std::isfinite(frame.tangent_basis.u_axis.y) || !std::isfinite(frame.tangent_basis.u_axis.z)) return false;
    if (!std::isfinite(frame.tangent_basis.v_axis.x) || !std::isfinite(frame.tangent_basis.v_axis.y) || !std::isfinite(frame.tangent_basis.v_axis.z)) return false;
    if (!std::isfinite(frame.tangent_basis.up_axis.x) || !std::isfinite(frame.tangent_basis.up_axis.y) || !std::isfinite(frame.tangent_basis.up_axis.z)) return false;

    double H = scale.chart_half_extent_mm * 0.001;
    if (frame.origin.u_m < -H || frame.origin.u_m > H || frame.origin.v_m < -H || frame.origin.v_m > H) return false;

    auto dot = [](const Vec3d& a, const Vec3d& b) { return a.x*b.x + a.y*b.y + a.z*b.z; };
    const double tol = 1e-6;
    if (std::abs(dot(frame.tangent_basis.u_axis, frame.tangent_basis.u_axis) - 1.0) > tol) return false;
    if (std::abs(dot(frame.tangent_basis.v_axis, frame.tangent_basis.v_axis) - 1.0) > tol) return false;
    if (std::abs(dot(frame.tangent_basis.up_axis, frame.tangent_basis.up_axis) - 1.0) > tol) return false;
    if (std::abs(dot(frame.tangent_basis.u_axis, frame.tangent_basis.v_axis)) > tol) return false;
    if (std::abs(dot(frame.tangent_basis.u_axis, frame.tangent_basis.up_axis)) > tol) return false;
    if (std::abs(dot(frame.tangent_basis.v_axis, frame.tangent_basis.up_axis)) > tol) return false;

    double source_u = position.u_m;
    double source_v = position.v_m;

    if (position.face != frame.origin.face) {
        const EdgeTransition* found_transition = nullptr;
        SurfaceEdge found_source_edge;

        for (int e = 0; e < 4; ++e) {
            SurfaceEdge edge = static_cast<SurfaceEdge>(e);
            const EdgeTransition& trans = get_edge_transition(static_cast<uint8_t>(frame.origin.face), edge);
            if (trans.destination_face == static_cast<uint8_t>(position.face)) {
                found_transition = &trans;
                found_source_edge = edge;
                break;
            }
        }
        
        if (!found_transition) return false;

        double destination_along = (found_transition->destination_parameter_axis == 0) ? position.u_m : position.v_m;
        double destination_fixed = (found_transition->destination_parameter_axis == 0) ? position.v_m : position.u_m;

        double source_along = destination_along * found_transition->parameter_sign;
        double inward_depth = H - found_transition->dest_fixed_coordinate * destination_fixed;

        if (inward_depth < -tol) return false;

        double source_edge_sign = (found_source_edge == SurfaceEdge::NegativeU || found_source_edge == SurfaceEdge::NegativeV) ? -1.0 : 1.0;
        double source_extended_fixed = source_edge_sign * (H + inward_depth);

        if (found_transition->source_parameter_axis == 0) {
            source_u = source_along;
            source_v = source_extended_fixed;
        } else {
            source_u = source_extended_fixed;
            source_v = source_along;
        }
    }

    double du = source_u - frame.origin.u_m;
    double dh = position.altitude_m - frame.origin.altitude_m;
    double dv = source_v - frame.origin.v_m;

    out_position.x = frame.tangent_basis.u_axis.x * du + frame.tangent_basis.up_axis.x * dh + frame.tangent_basis.v_axis.x * dv;
    out_position.y = frame.tangent_basis.u_axis.y * du + frame.tangent_basis.up_axis.y * dh + frame.tangent_basis.v_axis.y * dv;
    out_position.z = frame.tangent_basis.u_axis.z * du + frame.tangent_basis.up_axis.z * dh + frame.tangent_basis.v_axis.z * dv;

    return true;
}

[[nodiscard]] bool try_frame_to_surface(
    const FramePosition64& position,
    const SurfaceFrame& frame,
    const WorldScaleManifest& scale,
    SurfacePosition64& out_position
) noexcept {
    if (!is_valid_surface_face(frame.origin.face)) return false;
    if (frame.topology_version != scale.topology_version) return false;
    if (frame.projection_version != scale.projection_version) return false;

    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) return false;
    if (!std::isfinite(frame.origin.u_m) || !std::isfinite(frame.origin.v_m) || !std::isfinite(frame.origin.altitude_m)) return false;
    if (!std::isfinite(frame.tangent_basis.u_axis.x) || !std::isfinite(frame.tangent_basis.u_axis.y) || !std::isfinite(frame.tangent_basis.u_axis.z)) return false;
    if (!std::isfinite(frame.tangent_basis.v_axis.x) || !std::isfinite(frame.tangent_basis.v_axis.y) || !std::isfinite(frame.tangent_basis.v_axis.z)) return false;
    if (!std::isfinite(frame.tangent_basis.up_axis.x) || !std::isfinite(frame.tangent_basis.up_axis.y) || !std::isfinite(frame.tangent_basis.up_axis.z)) return false;

    double H = scale.chart_half_extent_mm * 0.001;
    if (frame.origin.u_m < -H || frame.origin.u_m > H || frame.origin.v_m < -H || frame.origin.v_m > H) return false;

    auto dot = [](const Vec3d& a, const Vec3d& b) { return a.x*b.x + a.y*b.y + a.z*b.z; };
    const double tol = 1e-6;
    if (std::abs(dot(frame.tangent_basis.u_axis, frame.tangent_basis.u_axis) - 1.0) > tol) return false;
    if (std::abs(dot(frame.tangent_basis.v_axis, frame.tangent_basis.v_axis) - 1.0) > tol) return false;
    if (std::abs(dot(frame.tangent_basis.up_axis, frame.tangent_basis.up_axis) - 1.0) > tol) return false;
    if (std::abs(dot(frame.tangent_basis.u_axis, frame.tangent_basis.v_axis)) > tol) return false;
    if (std::abs(dot(frame.tangent_basis.u_axis, frame.tangent_basis.up_axis)) > tol) return false;
    if (std::abs(dot(frame.tangent_basis.v_axis, frame.tangent_basis.up_axis)) > tol) return false;

    double du = dot(Vec3d{position.x, position.y, position.z}, frame.tangent_basis.u_axis);
    double dh = dot(Vec3d{position.x, position.y, position.z}, frame.tangent_basis.up_axis);
    double dv = dot(Vec3d{position.x, position.y, position.z}, frame.tangent_basis.v_axis);

    double u_coord = frame.origin.u_m + du;
    double v_coord = frame.origin.v_m + dv;
    double alt = frame.origin.altitude_m + dh;

    bool u_exceeds = (u_coord < -H || u_coord > H);
    bool v_exceeds = (v_coord < -H || v_coord > H);

    if (u_exceeds && v_exceeds) {
        return false;
    }

    if (!u_exceeds && !v_exceeds) {
        out_position.face = frame.origin.face;
        out_position.u_m = u_coord;
        out_position.v_m = v_coord;
        out_position.altitude_m = alt;
        out_position.topology_version = frame.topology_version;
        out_position.projection_version = frame.projection_version;
        return true;
    }

    SurfaceEdge cross_edge;
    if (u_exceeds) {
        cross_edge = (u_coord < -H) ? SurfaceEdge::NegativeU : SurfaceEdge::PositiveU;
    } else {
        cross_edge = (v_coord < -H) ? SurfaceEdge::NegativeV : SurfaceEdge::PositiveV;
    }

    SurfaceAddress addr;
    addr.face = frame.origin.face;
    addr.u_mm = static_cast<int64_t>(std::rint(u_coord * 1000.0));
    addr.v_mm = static_cast<int64_t>(std::rint(v_coord * 1000.0));
    addr.topology_version = frame.topology_version;
    addr.projection_version = frame.projection_version;

    SurfaceAddress canon = canonicalize_surface_address(addr, scale);
    if (static_cast<uint8_t>(canon.face) == 255) return false;

    out_position.face = canon.face;
    out_position.u_m = static_cast<double>(canon.u_mm) * 0.001;
    out_position.v_m = static_cast<double>(canon.v_mm) * 0.001;
    out_position.altitude_m = alt;
    out_position.topology_version = canon.topology_version;
    out_position.projection_version = canon.projection_version;

    return true;
}

SurfaceAddress canonicalize_surface_address(SurfaceAddress address, const WorldScaleManifest &scale) noexcept {
    if (address.topology_version != scale.topology_version || 
        address.projection_version != scale.projection_version) {
        address.face = static_cast<SurfaceFace>(255); // Invalid
        return address;
    }

    int64_t extent = scale.chart_half_extent_mm;
    
    uint8_t current_face = static_cast<uint8_t>(address.face);
    int64_t u = address.u_mm;
    int64_t v = address.v_mm;
    
    // Bounds check
    const int64_t MAX_OVERSHOOT = extent * 3;
    if (std::abs(u) > extent + MAX_OVERSHOOT || std::abs(v) > extent + MAX_OVERSHOOT) {
        address.face = static_cast<SurfaceFace>(255); // Invalid
        return address;
    }
    
    bool crossed = true;
    int iteration = 0;
    while(crossed) {
        if (iteration++ > 8) {
            address.face = static_cast<SurfaceFace>(255); // Invalid
            return address;
        }
        int64_t over_neg_u = (u < -extent) ? (-extent - u) : 0;
        int64_t over_pos_u = (u > extent) ? (u - extent) : 0;
        int64_t over_neg_v = (v < -extent) ? (-extent - v) : 0;
        int64_t over_pos_v = (v > extent) ? (v - extent) : 0;
        
        int64_t max_over = std::max({over_neg_u, over_pos_u, over_neg_v, over_pos_v});
        if (max_over == 0) break;
        
        SurfaceEdge cross_edge;
        if (max_over == over_neg_u) cross_edge = SurfaceEdge::NegativeU;
        else if (max_over == over_pos_u) cross_edge = SurfaceEdge::PositiveU;
        else if (max_over == over_neg_v) cross_edge = SurfaceEdge::NegativeV;
        else cross_edge = SurfaceEdge::PositiveV;
        
        const EdgeTransition& trans = get_edge_transition(current_face, cross_edge);
        int64_t source_param = (cross_edge == SurfaceEdge::NegativeU || cross_edge == SurfaceEdge::PositiveU) ? v : u;
        int64_t dest_param = source_param * trans.parameter_sign;
        int64_t new_fixed_axis_val = (trans.dest_fixed_coordinate == 1) ? (extent - max_over) : (-extent + max_over);
        
        if (trans.destination_parameter_axis == 0) {
            u = dest_param;
            v = new_fixed_axis_val;
        } else {
            u = new_fixed_axis_val;
            v = dest_param;
        }
        current_face = trans.destination_face;
    }
    
    bool resolved = false;
    while(!resolved) {
        resolved = true;
        
        bool on_neg_u = (u == -extent);
        bool on_pos_u = (u == extent);
        bool on_neg_v = (v == -extent);
        bool on_pos_v = (v == extent);
        
        uint8_t best_face = current_face;
        SurfaceEdge edge_to_cross;
        
        auto check_edge = [&](SurfaceEdge edge) {
            const EdgeTransition& t = get_edge_transition(current_face, edge);
            if (t.destination_face < best_face) {
                best_face = t.destination_face;
                edge_to_cross = edge;
            }
        };
        
        if (on_neg_u) check_edge(SurfaceEdge::NegativeU);
        if (on_pos_u) check_edge(SurfaceEdge::PositiveU);
        if (on_neg_v) check_edge(SurfaceEdge::NegativeV);
        if (on_pos_v) check_edge(SurfaceEdge::PositiveV);
        
        if (best_face < current_face) {
            const EdgeTransition& trans = get_edge_transition(current_face, edge_to_cross);
            int64_t source_param = (edge_to_cross == SurfaceEdge::NegativeU || edge_to_cross == SurfaceEdge::PositiveU) ? v : u;
            int64_t dest_param = source_param * trans.parameter_sign;
            int64_t new_fixed_axis_val = trans.dest_fixed_coordinate * extent;
            
            if (trans.destination_parameter_axis == 0) {
                u = dest_param;
                v = new_fixed_axis_val;
            } else {
                u = new_fixed_axis_val;
                v = dest_param;
            }
            current_face = best_face;
            resolved = false; 
        }
    }
    
    SurfaceAddress canon = address;
    canon.face = static_cast<SurfaceFace>(current_face);
    canon.u_mm = u;
    canon.v_mm = v;
    return canon;
}

} // namespace Multinet
