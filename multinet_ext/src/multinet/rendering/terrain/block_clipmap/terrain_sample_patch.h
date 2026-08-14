#ifndef MULTINET_TERRAIN_SAMPLE_PATCH_H
#define MULTINET_TERRAIN_SAMPLE_PATCH_H

#include "multinet/rendering/terrain/block_clipmap/block_clipmap_ids.h"
#include "multinet/core/spatial/surface_coordinate_conversion.h"
#include "multinet/core/spatial/surface_projection.h"
#include "multinet/core/spatial/world_manifests.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace multinet::rendering {

inline bool encode_patch_axis(double value, int8_t& out) noexcept {
	const long rounded = std::lround(value);
	if (rounded < -1 || rounded > 1 || std::abs(value - static_cast<double>(rounded)) > 1e-9) return false;
	out = static_cast<int8_t>(rounded);
	return true;
}

inline bool try_encode_sample_patch(
	const Multinet::SurfaceFrame& frame,
	uint8_t lod,
	uint8_t profile,
	TerrainSamplePatchKey& out
) noexcept {
	if (!frame.origin.is_valid()) return false;
	TerrainSamplePatchKey patch{};
	patch.anchor_face = frame.origin.face;
	patch.anchor_u_mm = static_cast<int64_t>(std::llround(frame.origin.u_m * 1000.0));
	patch.anchor_v_mm = static_cast<int64_t>(std::llround(frame.origin.v_m * 1000.0));
	patch.lod = lod;
	patch.profile = profile;
	patch.mapping_version = TERRAIN_SAMPLE_PATCH_MAPPING_V1;
	if (!encode_patch_axis(frame.tangent_basis.u_axis.x, patch.u_axis_x) ||
		!encode_patch_axis(frame.tangent_basis.u_axis.z, patch.u_axis_z) ||
		!encode_patch_axis(frame.tangent_basis.v_axis.x, patch.v_axis_x) ||
		!encode_patch_axis(frame.tangent_basis.v_axis.z, patch.v_axis_z) ||
		!patch.is_valid()) return false;
	out = patch;
	return true;
}

inline bool is_coherent_sample_patch(const TerrainSamplePatchKey& patch) noexcept {
	return patch.is_valid() &&
		(patch.mapping_version == TERRAIN_SAMPLE_PATCH_MAPPING_COHERENT_V2 ||
		 patch.mapping_version == TERRAIN_SAMPLE_PATCH_MAPPING_LOGICAL_CHART_V3 ||
		 patch.mapping_version == TERRAIN_SAMPLE_PATCH_MAPPING_BOUNDED_CHART_V4 ||
		 patch.mapping_version == TERRAIN_SAMPLE_PATCH_MAPPING_LOCAL_EXP_CHART_V5);
}

inline bool is_logical_chart_sample_patch(const TerrainSamplePatchKey& patch) noexcept {
	return patch.is_valid() &&
		(patch.mapping_version == TERRAIN_SAMPLE_PATCH_MAPPING_LOGICAL_CHART_V3 ||
		 patch.mapping_version == TERRAIN_SAMPLE_PATCH_MAPPING_BOUNDED_CHART_V4 ||
		 patch.mapping_version == TERRAIN_SAMPLE_PATCH_MAPPING_LOCAL_EXP_CHART_V5);
}

inline bool is_bounded_logical_chart_sample_patch(const TerrainSamplePatchKey& patch) noexcept {
	return patch.is_valid() && patch.mapping_version == TERRAIN_SAMPLE_PATCH_MAPPING_BOUNDED_CHART_V4;
}

struct LogicalSampleChart {
	Multinet::FramePosition64 root_direction{};
	Multinet::FramePosition64 presentation_x_angular_tangent{};
	Multinet::FramePosition64 presentation_z_angular_tangent{};
};

inline bool try_build_logical_sample_chart(
	const Multinet::SurfaceFrame& root,
	const Multinet::WorldDomainManifest& domain,
	LogicalSampleChart& out
) noexcept {
	if (!root.origin.is_valid() || !domain.is_valid() || domain.is_finite()) return false;
	const double half_extent_m = static_cast<double>(domain.closed_surface.chart_half_extent_mm) * 0.001;
	if (!(half_extent_m > 0.0) || !std::isfinite(half_extent_m)) return false;
	const double u = root.origin.u_m / half_extent_m;
	const double v = root.origin.v_m / half_extent_m;
	Multinet::FramePosition64 canonical_du{};
	Multinet::FramePosition64 canonical_dv{};
	if (!Multinet::ProjectionCOBE::map_forward_differential(
		static_cast<int>(root.origin.face), u, v, canonical_du, canonical_dv)) return false;

	LogicalSampleChart chart{};
	chart.root_direction = Multinet::ProjectionCOBE::map_forward(
		static_cast<int>(root.origin.face), u, v);
	const double inv_half_extent_m = 1.0 / half_extent_m;
	chart.presentation_x_angular_tangent = {
		(canonical_du.x * root.tangent_basis.u_axis.x + canonical_dv.x * root.tangent_basis.v_axis.x) * inv_half_extent_m,
		(canonical_du.y * root.tangent_basis.u_axis.x + canonical_dv.y * root.tangent_basis.v_axis.x) * inv_half_extent_m,
		(canonical_du.z * root.tangent_basis.u_axis.x + canonical_dv.z * root.tangent_basis.v_axis.x) * inv_half_extent_m
	};
	chart.presentation_z_angular_tangent = {
		(canonical_du.x * root.tangent_basis.u_axis.z + canonical_dv.x * root.tangent_basis.v_axis.z) * inv_half_extent_m,
		(canonical_du.y * root.tangent_basis.u_axis.z + canonical_dv.y * root.tangent_basis.v_axis.z) * inv_half_extent_m,
		(canonical_du.z * root.tangent_basis.u_axis.z + canonical_dv.z * root.tangent_basis.v_axis.z) * inv_half_extent_m
	};
	out = chart;
	return true;
}

// The V5 chart lives in one presentation plane, so its tangent axes must not
// be rebuilt from a cube-face alias after a corner crossing. Carry them by the
// shortest rotation between neighbouring canonical directions instead. Face
// ownership can change at a cube vertex; the local terrain chart must not.
inline bool try_transport_logical_sample_chart(
	const LogicalSampleChart& source,
	const Multinet::SurfaceFrame& destination_root,
	const Multinet::WorldDomainManifest& domain,
	LogicalSampleChart& out
) noexcept {
	if (!destination_root.origin.is_valid() || !domain.is_valid() || domain.is_finite()) return false;
	const double half_extent_m = static_cast<double>(domain.closed_surface.chart_half_extent_mm) * 0.001;
	if (!(half_extent_m > 0.0) || !std::isfinite(half_extent_m)) return false;
	const Multinet::FramePosition64 target_raw = Multinet::ProjectionCOBE::map_forward(
		static_cast<int>(destination_root.origin.face),
		destination_root.origin.u_m / half_extent_m,
		destination_root.origin.v_m / half_extent_m);
	const auto length = [](const Multinet::FramePosition64& value) noexcept {
		return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
	};
	const auto dot = [](const Multinet::FramePosition64& a, const Multinet::FramePosition64& b) noexcept {
		return a.x * b.x + a.y * b.y + a.z * b.z;
	};
	const auto cross = [](const Multinet::FramePosition64& a, const Multinet::FramePosition64& b) noexcept {
		return Multinet::FramePosition64{
			a.y * b.z - a.z * b.y,
			a.z * b.x - a.x * b.z,
			a.x * b.y - a.y * b.x
		};
	};
	const double source_length = length(source.root_direction);
	const double target_length = length(target_raw);
	if (!(source_length > 0.0) || !(target_length > 0.0) ||
		!std::isfinite(source_length) || !std::isfinite(target_length)) return false;
	const Multinet::FramePosition64 from{
		source.root_direction.x / source_length,
		source.root_direction.y / source_length,
		source.root_direction.z / source_length
	};
	const Multinet::FramePosition64 to{
		target_raw.x / target_length,
		target_raw.y / target_length,
		target_raw.z / target_length
	};
	const Multinet::FramePosition64 axis = cross(from, to);
	const double axis_sq = dot(axis, axis);
	const double cosine = std::clamp(dot(from, to), -1.0, 1.0);
	if (!std::isfinite(axis_sq) || !std::isfinite(cosine) || cosine < -0.999999) return false;
	const auto rotate = [&](const Multinet::FramePosition64& value) noexcept {
		if (axis_sq <= 1e-24) return value;
		const Multinet::FramePosition64 first = cross(axis, value);
		const Multinet::FramePosition64 second = cross(axis, first);
		const double scale = (1.0 - cosine) / axis_sq;
		return Multinet::FramePosition64{
			value.x + first.x + second.x * scale,
			value.y + first.y + second.y * scale,
			value.z + first.z + second.z * scale
		};
	};

	LogicalSampleChart transported{};
	transported.root_direction = to;
	transported.presentation_x_angular_tangent = rotate(source.presentation_x_angular_tangent);
	transported.presentation_z_angular_tangent = rotate(source.presentation_z_angular_tangent);
	if (!(length(transported.presentation_x_angular_tangent) > 0.0) ||
		!(length(transported.presentation_z_angular_tangent) > 0.0)) return false;
	out = transported;
	return true;
}

// Converts a flat presentation-plane input vector into the current canonical
// face's U/V delta using the same continuous V5 chart as terrain rendering.
// Navigation must not take its heading from a cube-face alias after rendering
// has deliberately stopped doing so.
inline bool try_map_logical_chart_delta_to_face_delta(
	const LogicalSampleChart& chart,
	const Multinet::SurfacePosition64& current_position,
	const Multinet::WorldDomainManifest& domain,
	double presentation_dx_m,
	double presentation_dz_m,
	Multinet::FramePosition64& out
) noexcept {
	if (!current_position.is_valid() || !domain.is_valid() || domain.is_finite() ||
		!std::isfinite(presentation_dx_m) || !std::isfinite(presentation_dz_m)) return false;
	const double half_extent_m = static_cast<double>(domain.closed_surface.chart_half_extent_mm) * 0.001;
	if (!(half_extent_m > 0.0) || !std::isfinite(half_extent_m)) return false;
	Multinet::FramePosition64 face_du{};
	Multinet::FramePosition64 face_dv{};
	if (!Multinet::ProjectionCOBE::map_forward_differential(
		static_cast<int>(current_position.face),
		current_position.u_m / half_extent_m,
		current_position.v_m / half_extent_m,
		face_du, face_dv)) return false;
	face_du.x /= half_extent_m;
	face_du.y /= half_extent_m;
	face_du.z /= half_extent_m;
	face_dv.x /= half_extent_m;
	face_dv.y /= half_extent_m;
	face_dv.z /= half_extent_m;
	const auto dot = [](const Multinet::FramePosition64& a, const Multinet::FramePosition64& b) noexcept {
		return a.x * b.x + a.y * b.y + a.z * b.z;
	};
	const Multinet::FramePosition64 desired{
		presentation_dx_m * chart.presentation_x_angular_tangent.x +
			presentation_dz_m * chart.presentation_z_angular_tangent.x,
		presentation_dx_m * chart.presentation_x_angular_tangent.y +
			presentation_dz_m * chart.presentation_z_angular_tangent.y,
		presentation_dx_m * chart.presentation_x_angular_tangent.z +
			presentation_dz_m * chart.presentation_z_angular_tangent.z
	};
	const double uu = dot(face_du, face_du);
	const double uv = dot(face_du, face_dv);
	const double vv = dot(face_dv, face_dv);
	const double determinant = uu * vv - uv * uv;
	// face_du/dv are expressed per metre. Their determinant therefore shrinks
	// with the fourth power of the world's linear scale. An absolute cutoff
	// rejects healthy charts at planet-sized sides, most visibly at three-face
	// junctions. Check conditioning instead of raw magnitude.
	const double metric_scale = uu * vv;
	if (!(uu > 0.0) || !(vv > 0.0) || !(metric_scale > 0.0) ||
		!std::isfinite(metric_scale) || !std::isfinite(determinant) ||
		!(determinant > metric_scale * 1e-12)) return false;
	const double rhs_u = dot(face_du, desired);
	const double rhs_v = dot(face_dv, desired);
	const double face_delta_u_m = (rhs_u * vv - rhs_v * uv) / determinant;
	const double face_delta_v_m = (rhs_v * uu - rhs_u * uv) / determinant;
	if (!std::isfinite(face_delta_u_m) || !std::isfinite(face_delta_v_m)) return false;
	out = { face_delta_u_m, 0.0, face_delta_v_m };
	return true;
}

inline bool try_sample_logical_chart(
	const LogicalSampleChart& chart,
	double presentation_dx_m,
	double presentation_dz_m,
	const Multinet::WorldDomainManifest& domain,
	Multinet::SurfacePosition64& out
) noexcept {
	if (!std::isfinite(presentation_dx_m) || !std::isfinite(presentation_dz_m) ||
		!domain.is_valid() || domain.is_finite()) return false;
	const Multinet::FramePosition64 tangent{
		presentation_dx_m * chart.presentation_x_angular_tangent.x + presentation_dz_m * chart.presentation_z_angular_tangent.x,
		presentation_dx_m * chart.presentation_x_angular_tangent.y + presentation_dz_m * chart.presentation_z_angular_tangent.y,
		presentation_dx_m * chart.presentation_x_angular_tangent.z + presentation_dz_m * chart.presentation_z_angular_tangent.z
	};
	const double angle = std::sqrt(tangent.x * tangent.x + tangent.y * tangent.y + tangent.z * tangent.z);
	Multinet::FramePosition64 direction = chart.root_direction;
	if (angle > 0.0) {
		const double sin_over_angle = std::sin(angle) / angle;
		const double cosine = std::cos(angle);
		direction = {
			cosine * chart.root_direction.x + sin_over_angle * tangent.x,
			cosine * chart.root_direction.y + sin_over_angle * tangent.y,
			cosine * chart.root_direction.z + sin_over_angle * tangent.z
		};
	}
	double u = 0.0;
	double v = 0.0;
	int face = -1;
	if (!Multinet::ProjectionCOBE::map_inverse(direction, -1, u, v, face)) return false;
	const double half_extent_m = static_cast<double>(domain.closed_surface.chart_half_extent_mm) * 0.001;
	Multinet::SurfacePosition64 position{};
	position.face = static_cast<Multinet::SurfaceFace>(face);
	position.u_m = u * half_extent_m;
	position.v_m = v * half_extent_m;
	position.topology_version = domain.topology_version;
	position.projection_version = domain.projection_version;
	if (!position.is_valid()) return false;
	out = position;
	return true;
}

inline bool try_sample_bounded_logical_chart(
	const LogicalSampleChart& chart,
	double presentation_dx_m,
	double presentation_dz_m,
	const Multinet::WorldDomainManifest& domain,
	Multinet::SurfacePosition64& out
) noexcept {
	if (!std::isfinite(presentation_dx_m) || !std::isfinite(presentation_dz_m) ||
		!domain.is_valid() || domain.is_finite()) return false;
	const Multinet::FramePosition64 tangent{
		presentation_dx_m * chart.presentation_x_angular_tangent.x + presentation_dz_m * chart.presentation_z_angular_tangent.x,
		presentation_dx_m * chart.presentation_x_angular_tangent.y + presentation_dz_m * chart.presentation_z_angular_tangent.y,
		presentation_dx_m * chart.presentation_x_angular_tangent.z + presentation_dz_m * chart.presentation_z_angular_tangent.z
	};
	Multinet::FramePosition64 direction{
		chart.root_direction.x + tangent.x,
		chart.root_direction.y + tangent.y,
		chart.root_direction.z + tangent.z
	};
	const double length = std::sqrt(
		direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
	if (!(length > 0.0) || !std::isfinite(length)) return false;
	direction.x /= length;
	direction.y /= length;
	direction.z /= length;

	double u = 0.0;
	double v = 0.0;
	int face = -1;
	if (!Multinet::ProjectionCOBE::map_inverse(direction, -1, u, v, face)) return false;
	const double half_extent_m = static_cast<double>(domain.closed_surface.chart_half_extent_mm) * 0.001;
	Multinet::SurfacePosition64 position{};
	position.face = static_cast<Multinet::SurfaceFace>(face);
	position.u_m = u * half_extent_m;
	position.v_m = v * half_extent_m;
	position.topology_version = domain.topology_version;
	position.projection_version = domain.projection_version;
	if (!position.is_valid()) return false;
	out = position;
	return true;
}

inline bool try_decode_sample_patch(
	const TerrainSamplePatchKey& patch,
	const Multinet::WorldDomainManifest& domain,
	Multinet::SurfaceFrame& out
) noexcept {
	if (!patch.is_valid() || !domain.is_valid()) return false;
	Multinet::SurfaceFrame frame{};
	frame.origin.face = patch.anchor_face;
	frame.origin.u_m = static_cast<double>(patch.anchor_u_mm) * 0.001;
	frame.origin.v_m = static_cast<double>(patch.anchor_v_mm) * 0.001;
	frame.origin.altitude_m = 0.0;
	frame.origin.topology_version = domain.topology_version;
	frame.origin.projection_version = domain.projection_version;
	frame.tangent_basis.u_axis = {
		static_cast<double>(patch.u_axis_x), 0.0, static_cast<double>(patch.u_axis_z)
	};
	frame.tangent_basis.up_axis = { 0.0, 1.0, 0.0 };
	frame.tangent_basis.v_axis = {
		static_cast<double>(patch.v_axis_x), 0.0, static_cast<double>(patch.v_axis_z)
	};
	frame.frame_epoch = 1;
	frame.topology_version = domain.topology_version;
	frame.projection_version = domain.projection_version;
	out = frame;
	return true;
}

// Builds the canonical mapping at a presentation-space block centre. The
// supplied X/Z delta is in the stable flat plane, not in the current face's
// U/V chart.
inline bool try_make_sample_patch(
	const Multinet::SurfaceFrame& observer_frame,
	double presentation_dx_m,
	double presentation_dz_m,
	uint8_t lod,
	uint8_t profile,
	const Multinet::WorldDomainManifest& domain,
	TerrainSamplePatchKey& out,
	uint32_t* out_transition_count = nullptr
) noexcept {
	if (!domain.is_valid() || !observer_frame.origin.is_valid() ||
		!std::isfinite(presentation_dx_m) || !std::isfinite(presentation_dz_m)) return false;

	const Multinet::FramePosition64 local_delta{
		presentation_dx_m * observer_frame.tangent_basis.u_axis.x +
			presentation_dz_m * observer_frame.tangent_basis.u_axis.z,
		0.0,
		presentation_dx_m * observer_frame.tangent_basis.v_axis.x +
			presentation_dz_m * observer_frame.tangent_basis.v_axis.z
	};
	Multinet::SurfaceFrame patch_frame{};
	Multinet::SurfacePosition64 patch_position{};
	uint32_t transitions = 0;
	if (!Multinet::try_advance_domain_surface_frame(
		local_delta, domain, observer_frame, patch_frame, patch_position, transitions)) return false;
	patch_frame.origin = patch_position;
	if (!try_encode_sample_patch(patch_frame, lod, profile, out)) return false;
	if (out_transition_count) *out_transition_count = transitions;
	return true;
}

// Builds a block identity against one immutable unfolding root. The root and
// generation remain stable as the observer moves, so page residency does too.
inline bool try_make_coherent_sample_patch(
	const Multinet::SurfaceFrame& unfolding_root,
	double unfolding_root_presentation_x_m,
	double unfolding_root_presentation_z_m,
	double presentation_center_x_m,
	double presentation_center_z_m,
	uint64_t unfolding_generation,
	uint8_t lod,
	uint8_t profile,
	const Multinet::WorldDomainManifest& domain,
	TerrainSamplePatchKey& out,
	Multinet::SurfacePosition64* out_center_position = nullptr,
	uint32_t* out_transition_count = nullptr
) noexcept {
	if (!domain.is_valid() || !unfolding_root.origin.is_valid() || unfolding_generation == 0 ||
		!std::isfinite(unfolding_root_presentation_x_m) ||
		!std::isfinite(unfolding_root_presentation_z_m) ||
		!std::isfinite(presentation_center_x_m) || !std::isfinite(presentation_center_z_m)) return false;

	const double center_dx_m = presentation_center_x_m - unfolding_root_presentation_x_m;
	const double center_dz_m = presentation_center_z_m - unfolding_root_presentation_z_m;
	const long double center_dx_mm = static_cast<long double>(center_dx_m) * 1000.0L;
	const long double center_dz_mm = static_cast<long double>(center_dz_m) * 1000.0L;
	if (center_dx_mm < static_cast<long double>((std::numeric_limits<int64_t>::min)()) ||
		center_dx_mm > static_cast<long double>((std::numeric_limits<int64_t>::max)()) ||
		center_dz_mm < static_cast<long double>((std::numeric_limits<int64_t>::min)()) ||
		center_dz_mm > static_cast<long double>((std::numeric_limits<int64_t>::max)())) return false;

	TerrainSamplePatchKey patch{};
	if (!try_encode_sample_patch(unfolding_root, lod, profile, patch)) return false;
	patch.presentation_center_dx_mm = static_cast<int64_t>(std::llround(center_dx_mm));
	patch.presentation_center_dz_mm = static_cast<int64_t>(std::llround(center_dz_mm));
	patch.unfolding_generation = unfolding_generation;
	patch.mapping_version = TERRAIN_SAMPLE_PATCH_MAPPING_COHERENT_V2;
	if (!patch.is_valid()) return false;

	const Multinet::FramePosition64 local_delta{
		center_dx_m * unfolding_root.tangent_basis.u_axis.x +
			center_dz_m * unfolding_root.tangent_basis.u_axis.z,
		0.0,
		center_dx_m * unfolding_root.tangent_basis.v_axis.x +
			center_dz_m * unfolding_root.tangent_basis.v_axis.z
	};
	Multinet::SurfaceFrame center_frame{};
	Multinet::SurfacePosition64 center_position{};
	uint32_t transitions = 0;
	if (!Multinet::try_advance_domain_surface_frame(
		local_delta, domain, unfolding_root, center_frame, center_position, transitions)) return false;

	out = patch;
	if (out_center_position) *out_center_position = center_position;
	if (out_transition_count) *out_transition_count = transitions;
	return true;
}

// Builds a flat block against an observer-local logical-sphere chart. This is
// still pre-CHP: X/Z vertices stay in the Euclidean BCCM plane. Only canonical
// Terrain lookup crosses cube edges and corners through a continuous chart.
inline bool try_make_logical_sample_patch(
	const Multinet::SurfaceFrame& chart_root,
	double chart_root_presentation_x_m,
	double chart_root_presentation_z_m,
	double presentation_center_x_m,
	double presentation_center_z_m,
	uint64_t chart_generation,
	uint8_t lod,
	uint8_t profile,
	const Multinet::WorldDomainManifest& domain,
	TerrainSamplePatchKey& out,
	Multinet::SurfacePosition64* out_center_position = nullptr
) noexcept {
	if (!domain.is_valid() || domain.is_finite() || !chart_root.origin.is_valid() || chart_generation == 0 ||
		!std::isfinite(chart_root_presentation_x_m) || !std::isfinite(chart_root_presentation_z_m) ||
		!std::isfinite(presentation_center_x_m) || !std::isfinite(presentation_center_z_m)) return false;

	const long double center_dx_mm = static_cast<long double>(
		presentation_center_x_m - chart_root_presentation_x_m) * 1000.0L;
	const long double center_dz_mm = static_cast<long double>(
		presentation_center_z_m - chart_root_presentation_z_m) * 1000.0L;
	if (center_dx_mm < static_cast<long double>((std::numeric_limits<int64_t>::min)()) ||
		center_dx_mm > static_cast<long double>((std::numeric_limits<int64_t>::max)()) ||
		center_dz_mm < static_cast<long double>((std::numeric_limits<int64_t>::min)()) ||
		center_dz_mm > static_cast<long double>((std::numeric_limits<int64_t>::max)())) return false;

	TerrainSamplePatchKey patch{};
	if (!try_encode_sample_patch(chart_root, lod, profile, patch)) return false;
	patch.presentation_center_dx_mm = static_cast<int64_t>(std::llround(center_dx_mm));
	patch.presentation_center_dz_mm = static_cast<int64_t>(std::llround(center_dz_mm));
	patch.unfolding_generation = chart_generation;
	patch.mapping_version = TERRAIN_SAMPLE_PATCH_MAPPING_LOCAL_EXP_CHART_V5;
	if (!patch.is_valid()) return false;

	LogicalSampleChart chart{};
	Multinet::SurfacePosition64 center_position{};
	if (!try_build_logical_sample_chart(chart_root, domain, chart) ||
		!try_sample_logical_chart(
			chart,
			static_cast<double>(patch.presentation_center_dx_mm) * 0.001,
			static_cast<double>(patch.presentation_center_dz_mm) * 0.001,
			domain,
			center_position)) return false;

	out = patch;
	if (out_center_position) *out_center_position = center_position;
	return true;
}

// Maps a presentation-space offset from the patch centre to canonical surface
// truth. CPU page generation and test-side shader references share this path.
inline bool try_sample_patch_position(
	const TerrainSamplePatchKey& patch,
	double presentation_dx_m,
	double presentation_dz_m,
	const Multinet::WorldDomainManifest& domain,
	Multinet::SurfacePosition64& out,
	uint32_t* out_transition_count = nullptr
) noexcept {
	Multinet::SurfaceFrame patch_frame{};
	if (!try_decode_sample_patch(patch, domain, patch_frame)) return false;
	if (is_logical_chart_sample_patch(patch)) {
		LogicalSampleChart chart{};
		if (!try_build_logical_sample_chart(patch_frame, domain, chart)) return false;
		const double sample_dx_m = static_cast<double>(patch.presentation_center_dx_mm) * 0.001 + presentation_dx_m;
		const double sample_dz_m = static_cast<double>(patch.presentation_center_dz_mm) * 0.001 + presentation_dz_m;
		const bool sampled = is_bounded_logical_chart_sample_patch(patch)
			? try_sample_bounded_logical_chart(chart, sample_dx_m, sample_dz_m, domain, out)
			: try_sample_logical_chart(chart, sample_dx_m, sample_dz_m, domain, out);
		if (!sampled) return false;
		if (out_transition_count) *out_transition_count = 0;
		return true;
	}
	const double center_dx_m = is_coherent_sample_patch(patch)
		? static_cast<double>(patch.presentation_center_dx_mm) * 0.001 : 0.0;
	const double center_dz_m = is_coherent_sample_patch(patch)
		? static_cast<double>(patch.presentation_center_dz_mm) * 0.001 : 0.0;
	const Multinet::FramePosition64 local_delta{
		(center_dx_m + presentation_dx_m) * patch_frame.tangent_basis.u_axis.x +
			(center_dz_m + presentation_dz_m) * patch_frame.tangent_basis.u_axis.z,
		0.0,
		(center_dx_m + presentation_dx_m) * patch_frame.tangent_basis.v_axis.x +
			(center_dz_m + presentation_dz_m) * patch_frame.tangent_basis.v_axis.z
	};
	Multinet::SurfaceFrame sampled_frame{};
	uint32_t transitions = 0;
	if (!Multinet::try_advance_domain_surface_frame(
		local_delta, domain, patch_frame, sampled_frame, out, transitions)) return false;
	if (out_transition_count) *out_transition_count = transitions;
	return true;
}

inline uint8_t sample_patch_orientation(const TerrainSamplePatchKey& patch) noexcept {
	if (patch.u_axis_x == 1 && patch.u_axis_z == 0) return 0;
	if (patch.u_axis_x == 0 && patch.u_axis_z == 1) return 1;
	if (patch.u_axis_x == -1 && patch.u_axis_z == 0) return 2;
	if (patch.u_axis_x == 0 && patch.u_axis_z == -1) return 3;
	return 0;
}

} // namespace multinet::rendering

#endif // MULTINET_TERRAIN_SAMPLE_PATCH_H
