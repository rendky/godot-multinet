#include "surface_address.h"
#include "surface_frame.h"
#include "surface_topology.h"
#include "world_manifests.h"
#include "multinet/core/coordinates.h"

#include <cmath>
#include <algorithm>
#include <limits>
#include <utility>

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

    if (!u_exceeds && !v_exceeds) {
        out_position.face = frame.origin.face;
        out_position.u_m = u_coord;
        out_position.v_m = v_coord;
        out_position.altitude_m = alt;
        out_position.topology_version = frame.topology_version;
        out_position.projection_version = frame.projection_version;
        return true;
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

namespace {

[[nodiscard]] uint64_t magnitude_i64(int64_t value) noexcept {
	if (value >= 0) return static_cast<uint64_t>(value);
	return static_cast<uint64_t>(-(value + 1)) + 1ULL;
}

[[nodiscard]] bool negate_i64_checked(int64_t value, int64_t& out) noexcept {
	if (value == (std::numeric_limits<int64_t>::min)()) return false;
	out = -value;
	return true;
}

[[nodiscard]] bool fixed_coordinate_from_overshoot(
	int64_t extent,
	uint64_t overshoot,
	int8_t fixed_sign,
	int64_t& out
) noexcept {
	if (overshoot > static_cast<uint64_t>((std::numeric_limits<int64_t>::max)())) return false;
	const int64_t over = static_cast<int64_t>(overshoot);
	if (fixed_sign > 0) {
		const uint64_t positive_limit = static_cast<uint64_t>((std::numeric_limits<int64_t>::max)()) + static_cast<uint64_t>(extent);
		if (overshoot > positive_limit) return false;
		out = extent - over;
	} else {
		out = -extent + over;
	}
	return true;
}

template <typename TransitionCallback>
[[nodiscard]] bool canonicalize_surface_address_impl(
	SurfaceAddress address,
	const WorldScaleManifest& scale,
	SurfaceAddress& out,
	TransitionCallback&& on_transition
) noexcept {
	if (!address.is_valid() || address.topology_version != scale.topology_version ||
		address.projection_version != scale.projection_version || scale.chart_half_extent_mm <= 0) {
		return false;
	}

	const int64_t extent = scale.chart_half_extent_mm;
	uint8_t current_face = static_cast<uint8_t>(address.face);
	int64_t u = address.u_mm;
	int64_t v = address.v_mm;

	// The bound is derived from the supplied representable coordinate rather
	// than from a world-size heuristic. It is deliberately not a fixed number
	// of face widths; each loop consumes one actual chart transition.
	const uint64_t max_magnitude = std::max(magnitude_i64(u), magnitude_i64(v));
	const uint64_t crossings_bound = max_magnitude / static_cast<uint64_t>(extent) + 8ULL;
	uint64_t crossings = 0;

	while (true) {
		const uint64_t over_neg_u = u < -extent ? magnitude_i64(u) - static_cast<uint64_t>(extent) : 0ULL;
		const uint64_t over_pos_u = u > extent ? magnitude_i64(u) - static_cast<uint64_t>(extent) : 0ULL;
		const uint64_t over_neg_v = v < -extent ? magnitude_i64(v) - static_cast<uint64_t>(extent) : 0ULL;
		const uint64_t over_pos_v = v > extent ? magnitude_i64(v) - static_cast<uint64_t>(extent) : 0ULL;
		const uint64_t max_over = std::max({ over_neg_u, over_pos_u, over_neg_v, over_pos_v });
		if (max_over == 0) break;
		if (crossings >= crossings_bound) return false;

		SurfaceEdge cross_edge;
		if (max_over == over_neg_u) cross_edge = SurfaceEdge::NegativeU;
		else if (max_over == over_pos_u) cross_edge = SurfaceEdge::PositiveU;
		else if (max_over == over_neg_v) cross_edge = SurfaceEdge::NegativeV;
		else cross_edge = SurfaceEdge::PositiveV;

		const EdgeTransition& transition = get_edge_transition(current_face, cross_edge);
		int64_t source_param = (cross_edge == SurfaceEdge::NegativeU || cross_edge == SurfaceEdge::PositiveU) ? v : u;
		int64_t destination_param = source_param;
		if (transition.parameter_sign < 0 && !negate_i64_checked(source_param, destination_param)) return false;

		int64_t fixed_coordinate = 0;
		if (!fixed_coordinate_from_overshoot(extent, max_over, transition.dest_fixed_coordinate, fixed_coordinate)) return false;
		if (!on_transition(static_cast<SurfaceFace>(current_face), cross_edge, transition)) return false;

		if (transition.destination_parameter_axis == 0) {
			u = destination_param;
			v = fixed_coordinate;
		} else {
			u = fixed_coordinate;
			v = destination_param;
		}
		current_face = transition.destination_face;
		++crossings;
	}

	// Canonicalize exact edge aliases deterministically. At a corner several
	// aliases can exist; selecting the lowest destination face preserves the
	// existing canonical identity rule and terminates in at most five steps.
	for (uint32_t alias_step = 0; alias_step < 6; ++alias_step) {
		const bool on_neg_u = u == -extent;
		const bool on_pos_u = u == extent;
		const bool on_neg_v = v == -extent;
		const bool on_pos_v = v == extent;
		uint8_t best_face = current_face;
		SurfaceEdge edge_to_cross = SurfaceEdge::NegativeU;
		auto check_edge = [&](SurfaceEdge edge) {
			const EdgeTransition& candidate = get_edge_transition(current_face, edge);
			if (candidate.destination_face < best_face) {
				best_face = candidate.destination_face;
				edge_to_cross = edge;
			}
		};
		if (on_neg_u) check_edge(SurfaceEdge::NegativeU);
		if (on_pos_u) check_edge(SurfaceEdge::PositiveU);
		if (on_neg_v) check_edge(SurfaceEdge::NegativeV);
		if (on_pos_v) check_edge(SurfaceEdge::PositiveV);
		if (best_face >= current_face) break;

		const EdgeTransition& transition = get_edge_transition(current_face, edge_to_cross);
		int64_t source_param = (edge_to_cross == SurfaceEdge::NegativeU || edge_to_cross == SurfaceEdge::PositiveU) ? v : u;
		int64_t destination_param = source_param;
		if (transition.parameter_sign < 0 && !negate_i64_checked(source_param, destination_param)) return false;
		if (!on_transition(static_cast<SurfaceFace>(current_face), edge_to_cross, transition)) return false;
		const int64_t fixed_coordinate = transition.dest_fixed_coordinate > 0 ? extent : -extent;
		if (transition.destination_parameter_axis == 0) {
			u = destination_param;
			v = fixed_coordinate;
		} else {
			u = fixed_coordinate;
			v = destination_param;
		}
		current_face = best_face;
	}

	out = address;
	out.face = static_cast<SurfaceFace>(current_face);
	out.u_mm = u;
	out.v_mm = v;
	return true;
}

} // namespace

SurfaceAddress canonicalize_surface_address(SurfaceAddress address, const WorldScaleManifest& scale) noexcept {
	SurfaceAddress out = address;
	const bool ok = canonicalize_surface_address_impl(
		address,
		scale,
		out,
		[](SurfaceFace, SurfaceEdge, const EdgeTransition&) noexcept { return true; }
	);
	if (!ok) out.face = static_cast<SurfaceFace>(255);
	return out;
}

bool try_domain_surface_to_frame(
	const SurfacePosition64& position,
	const SurfaceFrame& frame,
	const WorldDomainManifest& domain,
	FramePosition64& out_position
) noexcept {
	if (!domain.is_valid()) return false;
	if (!domain.is_finite()) return try_surface_to_frame(position, frame, domain.closed_surface, out_position);
	if (position.face != SurfaceFace::PositiveX || frame.origin.face != SurfaceFace::PositiveX) return false;
	if (position.topology_version != domain.topology_version || frame.topology_version != domain.topology_version) return false;
	if (position.projection_version != domain.projection_version || frame.projection_version != domain.projection_version) return false;
	const double hx = static_cast<double>(domain.finite.half_extent_x_mm) * 0.001;
	const double hz = static_cast<double>(domain.finite.half_extent_z_mm) * 0.001;
	if (position.u_m < -hx || position.u_m > hx || position.v_m < -hz || position.v_m > hz) return false;
	if (!std::isfinite(position.u_m) || !std::isfinite(position.v_m) || !std::isfinite(position.altitude_m)) return false;
	const double du = position.u_m - frame.origin.u_m;
	const double dv = position.v_m - frame.origin.v_m;
	const double dh = position.altitude_m - frame.origin.altitude_m;
	out_position.x = frame.tangent_basis.u_axis.x * du + frame.tangent_basis.up_axis.x * dh + frame.tangent_basis.v_axis.x * dv;
	out_position.y = frame.tangent_basis.u_axis.y * du + frame.tangent_basis.up_axis.y * dh + frame.tangent_basis.v_axis.y * dv;
	out_position.z = frame.tangent_basis.u_axis.z * du + frame.tangent_basis.up_axis.z * dh + frame.tangent_basis.v_axis.z * dv;
	return true;
}

bool try_frame_to_domain_surface(
	const FramePosition64& position,
	const SurfaceFrame& frame,
	const WorldDomainManifest& domain,
	SurfacePosition64& out_position
) noexcept {
	if (!domain.is_valid()) return false;
	if (!domain.is_finite()) return try_frame_to_surface(position, frame, domain.closed_surface, out_position);
	if (frame.origin.face != SurfaceFace::PositiveX || frame.topology_version != domain.topology_version || frame.projection_version != domain.projection_version) return false;
	if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) return false;
	const auto dot = [](const Vec3d& a, const Vec3d& b) { return a.x * b.x + a.y * b.y + a.z * b.z; };
	const Vec3d p{ position.x, position.y, position.z };
	double u = frame.origin.u_m + dot(p, frame.tangent_basis.u_axis);
	double v = frame.origin.v_m + dot(p, frame.tangent_basis.v_axis);
	double altitude = frame.origin.altitude_m + dot(p, frame.tangent_basis.up_axis);
	const double hx = static_cast<double>(domain.finite.half_extent_x_mm) * 0.001;
	const double hz = static_cast<double>(domain.finite.half_extent_z_mm) * 0.001;
	if (u < -hx || u > hx || v < -hz || v > hz) return false;
	out_position.face = SurfaceFace::PositiveX;
	out_position.u_m = u;
	out_position.v_m = v;
	out_position.altitude_m = altitude;
	out_position.topology_version = domain.topology_version;
	out_position.projection_version = domain.projection_version;
	return true;
}

bool try_advance_domain_surface_frame(
	const FramePosition64& local_delta,
	const WorldDomainManifest& domain,
	const SurfaceFrame& initial_frame,
	SurfaceFrame& out_frame,
	SurfacePosition64& out_position,
	uint32_t& out_transition_count,
	SurfaceFace* out_last_source_face,
	SurfaceFace* out_last_destination_face,
	SurfaceEdge* out_last_edge
) noexcept {
	out_transition_count = 0;
	if (out_last_source_face) *out_last_source_face = initial_frame.origin.face;
	if (out_last_destination_face) *out_last_destination_face = initial_frame.origin.face;
	if (out_last_edge) *out_last_edge = SurfaceEdge::NegativeU;
	if (!domain.is_valid() || !initial_frame.origin.is_valid() ||
		initial_frame.topology_version != domain.topology_version ||
		initial_frame.projection_version != domain.projection_version ||
		!std::isfinite(local_delta.x) || !std::isfinite(local_delta.y) || !std::isfinite(local_delta.z)) {
		return false;
	}

	const auto add_checked = [](double a, double b, double& out) noexcept {
		out = a + b;
		return std::isfinite(out);
	};

	if (domain.is_finite()) {
		if (initial_frame.origin.face != SurfaceFace::PositiveX) return false;
		const FramePosition64 world_delta{
			initial_frame.tangent_basis.u_axis.x * local_delta.x + initial_frame.tangent_basis.up_axis.x * local_delta.y + initial_frame.tangent_basis.v_axis.x * local_delta.z,
			initial_frame.tangent_basis.u_axis.y * local_delta.x + initial_frame.tangent_basis.up_axis.y * local_delta.y + initial_frame.tangent_basis.v_axis.y * local_delta.z,
			initial_frame.tangent_basis.u_axis.z * local_delta.x + initial_frame.tangent_basis.up_axis.z * local_delta.y + initial_frame.tangent_basis.v_axis.z * local_delta.z
		};
		if (!try_frame_to_domain_surface(world_delta, initial_frame, domain, out_position)) return false;
		out_frame = initial_frame;
		out_frame.origin = out_position;
		return true;
	}

	if (initial_frame.origin.topology_version != domain.topology_version ||
		initial_frame.origin.projection_version != domain.projection_version) return false;

	double raw_altitude = 0.0;
	if (!add_checked(initial_frame.origin.altitude_m, local_delta.y, raw_altitude)) return false;

	const double half_extent_m = static_cast<double>(domain.closed_surface.chart_half_extent_mm) * 0.001;
	if (!(half_extent_m > 0.0) || !std::isfinite(half_extent_m)) return false;
	// Time comparisons are dimensionless. The final face-boundary check is in
	// metres and needs a scale-aware skin so FP64 edge landing is not rejected
	// before the clamp below. Keep the physical tolerance below a centimetre at
	// all supported scales, so genuine topology errors still fail closed.
	constexpr double time_epsilon = 1e-10;
	// The hit-time numerator uses this separate metre-space movement cutoff.
	constexpr double movement_epsilon_m = 1e-10;
	constexpr double ulp_factor = 8.0;
	const double boundary_epsilon_m = std::max(
		1e-9,
		ulp_factor * std::numeric_limits<double>::epsilon() * std::max(1.0, std::abs(half_extent_m)));
	SurfaceFrame transported = initial_frame;
	double u_m = initial_frame.origin.u_m;
	double v_m = initial_frame.origin.v_m;
	// Preserve the caller's unwrapped flat intent while each face converts it
	// into its own U/V coordinates. Endpoint canonicalization used to choose U
	// before V even when V was hit first, which is fatal near three-face joins.
	FramePosition64 remaining_flat{
		initial_frame.tangent_basis.u_axis.x * local_delta.x + initial_frame.tangent_basis.v_axis.x * local_delta.z,
		0.0,
		initial_frame.tangent_basis.u_axis.z * local_delta.x + initial_frame.tangent_basis.v_axis.z * local_delta.z
	};
	constexpr uint32_t max_crossings = 4096;
	for (;;) {
		const double delta_u = transported.tangent_basis.u_axis.x * remaining_flat.x +
			transported.tangent_basis.u_axis.z * remaining_flat.z;
		const double delta_v = transported.tangent_basis.v_axis.x * remaining_flat.x +
			transported.tangent_basis.v_axis.z * remaining_flat.z;
		const auto hit_time = [&](double coordinate, double delta) noexcept {
			if (delta > movement_epsilon_m) return (half_extent_m - coordinate) / delta;
			if (delta < -movement_epsilon_m) return (-half_extent_m - coordinate) / delta;
			return (std::numeric_limits<double>::infinity)();
		};
		double u_time = hit_time(u_m, delta_u);
		double v_time = hit_time(v_m, delta_v);
		if (u_time < 0.0 && u_time > -time_epsilon) u_time = 0.0;
		if (v_time < 0.0 && v_time > -time_epsilon) v_time = 0.0;
		const double hit = std::min(u_time, v_time);
		if (!std::isfinite(hit) || hit >= 1.0 - time_epsilon) {
			u_m += delta_u;
			v_m += delta_v;
			break;
		}
		if (hit < 0.0 || out_transition_count >= max_crossings) return false;
		u_m += delta_u * hit;
		v_m += delta_v * hit;
		const bool hit_u = std::abs(u_time - hit) <= time_epsilon;
		const bool hit_v = std::abs(v_time - hit) <= time_epsilon;
		SurfaceEdge edge = hit_u
			? (delta_u < 0.0 ? SurfaceEdge::NegativeU : SurfaceEdge::PositiveU)
			: (delta_v < 0.0 ? SurfaceEdge::NegativeV : SurfaceEdge::PositiveV);
		if (hit_u && hit_v) {
			const SurfaceEdge u_edge = delta_u < 0.0 ? SurfaceEdge::NegativeU : SurfaceEdge::PositiveU;
			const SurfaceEdge v_edge = delta_v < 0.0 ? SurfaceEdge::NegativeV : SurfaceEdge::PositiveV;
			const double remaining_factor = 1.0 - hit;
			const FramePosition64 after_hit_flat{
				remaining_flat.x * remaining_factor, 0.0, remaining_flat.z * remaining_factor
			};
			auto inward_component = [&](SurfaceEdge candidate) noexcept {
				SurfaceFrame candidate_frame{};
				if (!try_transport_flat_surface_frame(transported, candidate, candidate_frame)) {
					return -(std::numeric_limits<double>::infinity)();
				}
				const EdgeTransition& transition = get_edge_transition(
					static_cast<uint8_t>(transported.origin.face), candidate);
				const double next_u = candidate_frame.tangent_basis.u_axis.x * after_hit_flat.x +
					candidate_frame.tangent_basis.u_axis.z * after_hit_flat.z;
				const double next_v = candidate_frame.tangent_basis.v_axis.x * after_hit_flat.x +
					candidate_frame.tangent_basis.v_axis.z * after_hit_flat.z;
				const double fixed_delta = transition.destination_parameter_axis == 0 ? next_v : next_u;
				const bool fixed_positive = transition.destination_edge == SurfaceEdge::PositiveU ||
					transition.destination_edge == SurfaceEdge::PositiveV;
				return fixed_positive ? -fixed_delta : fixed_delta;
			};
			// A mathematically exact vertex hit has two aliases. Pick the face into
			// which the remaining flat motion points, rather than the old U-first
			// implementation detail. Stable tie-break preserves replay determinism.
			edge = inward_component(v_edge) > inward_component(u_edge) ? v_edge : u_edge;
		}

		const EdgeTransition& transition = get_edge_transition(
			static_cast<uint8_t>(transported.origin.face), edge);
		const double source_along = transition.source_parameter_axis == 0 ? u_m : v_m;
		SurfaceFrame next{};
		if (!try_transport_flat_surface_frame(transported, edge, next)) return false;
		const double destination_along = source_along * transition.parameter_sign;
		const double destination_fixed =
			(transition.destination_edge == SurfaceEdge::PositiveU || transition.destination_edge == SurfaceEdge::PositiveV)
				? half_extent_m : -half_extent_m;
		if (transition.destination_parameter_axis == 0) {
			u_m = destination_along;
			v_m = destination_fixed;
		} else {
			u_m = destination_fixed;
			v_m = destination_along;
		}
		next.origin.face = static_cast<SurfaceFace>(transition.destination_face);
		next.origin.u_m = u_m;
		next.origin.v_m = v_m;
		next.origin.altitude_m = raw_altitude;
		next.origin.topology_version = domain.topology_version;
		next.origin.projection_version = domain.projection_version;
		next.topology_version = domain.topology_version;
		next.projection_version = domain.projection_version;
		transported = next;
		remaining_flat.x *= 1.0 - hit;
		remaining_flat.z *= 1.0 - hit;
		++out_transition_count;
		if (out_last_source_face) *out_last_source_face = static_cast<SurfaceFace>(transition.source_face);
		if (out_last_destination_face) *out_last_destination_face = static_cast<SurfaceFace>(transition.destination_face);
		if (out_last_edge) *out_last_edge = edge;
	}
	if (u_m < -half_extent_m - boundary_epsilon_m || u_m > half_extent_m + boundary_epsilon_m ||
		v_m < -half_extent_m - boundary_epsilon_m || v_m > half_extent_m + boundary_epsilon_m) return false;
	if (!std::isfinite(u_m) || !std::isfinite(v_m)) return false;
	out_position.face = transported.origin.face;
	out_position.u_m = std::clamp(u_m, -half_extent_m, half_extent_m);
	out_position.v_m = std::clamp(v_m, -half_extent_m, half_extent_m);
	out_position.altitude_m = raw_altitude;
	out_position.topology_version = domain.topology_version;
	out_position.projection_version = domain.projection_version;
	transported.origin = out_position;
	if (out_transition_count > 0) {
		if (transported.frame_epoch > (std::numeric_limits<uint64_t>::max)() - out_transition_count) return false;
		transported.frame_epoch += out_transition_count;
	}
	out_frame = transported;
	return true;
}

} // namespace Multinet
