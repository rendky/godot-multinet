#include "godot/terrain_adapter.h"
#include "multinet/core/spatial/surface_topology.h"
#include "multinet/core/spatial/surface_coordinate_conversion.h"
#include "multinet/rendering/terrain/block_clipmap/terrain_sample_patch.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <iostream>
#include <condition_variable>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <cmath>
#include <limits>
#include <iomanip>
#include <sstream>

#ifdef DEBUG_ENABLED
namespace {
class DebugBlockingDiagnosticDeltaField final : public Multinet::TerrainCommittedDeltaField {
	Multinet::CanonicalDiagnosticTerrainCommittedDeltaField inner_field;
	mutable std::mutex mutex;
	mutable std::condition_variable release_cv;
	mutable bool released{ false };

public:
	DebugBlockingDiagnosticDeltaField(
		Multinet::SurfacePosition64 center,
		double radius_m,
		float amplitude_m,
		const Multinet::WorldScaleManifest& manifest,
		uint32_t content_version
	) : inner_field(center, radius_m, amplitude_m, manifest, content_version) {}

	float sample_delta(Multinet::SurfacePosition64 position) const noexcept override {
		std::unique_lock<std::mutex> lock(mutex);
		release_cv.wait(lock, [this] { return released; });
		lock.unlock();
		return inner_field.sample_delta(position);
	}

	bool block_may_have_nonzero_delta(
		const multinet::rendering::TerrainRenderBlockKey& key,
		const Multinet::WorldScaleManifest& manifest,
		const multinet::rendering::BlockClipmapProfile& profile,
		double apron_m
	) const noexcept override {
		return inner_field.block_may_have_nonzero_delta(key, manifest, profile, apron_m);
	}

	uint32_t get_block_content_version(
		const multinet::rendering::TerrainRenderBlockKey& key,
		const Multinet::WorldScaleManifest& manifest,
		const multinet::rendering::BlockClipmapProfile& profile,
		double apron_m
	) const noexcept override {
		return inner_field.get_block_content_version(key, manifest, profile, apron_m);
	}

	Multinet::TerrainDeltaEnvelope get_conservative_envelope() const noexcept override {
		return inner_field.get_conservative_envelope();
	}

	void release_generation() noexcept {
		std::lock_guard<std::mutex> lock(mutex);
		released = true;
		release_cv.notify_all();
	}
};
}
#endif

namespace godot {

namespace {
constexpr double MAX_WORLD_SIDE_KM = 4294967.295; // UINT32_MAX metres, in km.
constexpr uint64_t MIN_RENDERABLE_CLOSED_SIDE_M = 79;
constexpr double EDITOR_PRESENTATION_REBASE_THRESHOLD_M = 4096.0;

bool try_km_to_world_metres(double kilometres, uint64_t& out_metres) noexcept {
	const long double metres = static_cast<long double>(kilometres) * 1000.0L;
	if (!std::isfinite(kilometres) || !std::isfinite(static_cast<double>(metres)) ||
		metres <= 0.0L || metres > static_cast<long double>(MAX_WORLD_SIDE_KM) * 1000.0L) return false;
	out_metres = static_cast<uint64_t>(std::llround(metres));
	return out_metres > 0 && out_metres <= static_cast<uint64_t>(MAX_WORLD_SIDE_KM * 1000.0);
}

Multinet::WorldScaleManifest make_compatibility_scale(const Multinet::WorldDomainManifest& domain) {
	if (!domain.is_finite()) return domain.closed_surface;
	Multinet::WorldScaleManifest scale{};
	scale.input.area_equivalent_side_m = std::max(domain.finite.extent_x_m, domain.finite.extent_z_m);
	scale.total_surface_area_m2 = domain.canonical_area_m2;
	scale.chart_half_extent_mm = std::max(domain.finite.half_extent_x_mm, domain.finite.half_extent_z_mm);
	scale.regions_per_face_axis = std::max(domain.finite.regions_x, domain.finite.regions_z);
	scale.actual_region_extent_m = std::max(domain.finite.actual_region_extent_x_m, domain.finite.actual_region_extent_z_m);
	scale.topology_version = domain.topology_version;
	scale.projection_version = domain.projection_version;
	scale.logical_area_radius_m = std::sqrt(static_cast<double>(domain.canonical_area_m2) / (4.0 * 3.14159265358979323846));
	scale.manifest_hash = domain.domain_manifest_hash;
	return scale;
}

void refresh_tracking_logical_chart(
	const Multinet::WorldDomainManifest& domain,
	const Multinet::SurfaceFrame& root,
	bool& has_chart,
	Multinet::FramePosition64& root_direction,
	Multinet::FramePosition64& presentation_x_tangent,
	Multinet::FramePosition64& presentation_z_tangent
) {
	if (!domain.is_valid() || domain.is_finite()) {
		has_chart = false;
		return;
	}
	multinet::rendering::LogicalSampleChart chart{};
	if (has_chart) {
		const multinet::rendering::LogicalSampleChart previous{
			root_direction, presentation_x_tangent, presentation_z_tangent
		};
		if (!multinet::rendering::try_transport_logical_sample_chart(previous, root, domain, chart)) {
			has_chart = false;
		}
	}
	if (!has_chart && !multinet::rendering::try_build_logical_sample_chart(root, domain, chart)) return;
	root_direction = chart.root_direction;
	presentation_x_tangent = chart.presentation_x_angular_tangent;
	presentation_z_tangent = chart.presentation_z_angular_tangent;
	has_chart = true;
}

bool try_map_closed_presentation_motion(
	const Multinet::WorldDomainManifest& domain,
	const Multinet::SurfacePosition64& position,
	bool has_chart,
	const Multinet::FramePosition64& root_direction,
	const Multinet::FramePosition64& presentation_x_tangent,
	const Multinet::FramePosition64& presentation_z_tangent,
	double presentation_dx_m,
	double presentation_dz_m,
	Multinet::FramePosition64& out_local_delta
) {
	if (domain.is_finite() || !has_chart) return false;
	const multinet::rendering::LogicalSampleChart chart{
		root_direction, presentation_x_tangent, presentation_z_tangent
	};
	return multinet::rendering::try_map_logical_chart_delta_to_face_delta(
		chart, position, domain, presentation_dx_m, presentation_dz_m, out_local_delta);
}

Multinet::SurfaceFrame make_editor_frame_for_face(
	Multinet::WorldDomainTopology topology,
	Multinet::SurfaceFace face,
	const Multinet::SurfacePosition64& origin,
	uint64_t epoch,
	uint32_t topology_version,
	uint32_t projection_version
) {
	Multinet::SurfaceFrame frame;
	frame.origin = origin;
	frame.frame_epoch = epoch;
	frame.topology_version = topology_version;
	frame.projection_version = projection_version;
	if (topology == Multinet::WorldDomainTopology::FiniteRectangle) {
		frame.tangent_basis.u_axis = { 1.0, 0.0, 0.0 };
		frame.tangent_basis.v_axis = { 0.0, 0.0, 1.0 };
		frame.tangent_basis.up_axis = { 0.0, 1.0, 0.0 };
		return frame;
	}
	Multinet::SurfaceFrame flat_basis;
	if (Multinet::try_make_flat_surface_frame_for_face(face, flat_basis)) {
		frame.tangent_basis = flat_basis.tangent_basis;
	}
	return frame;
}

bool try_flat_edge_direction(
	const Multinet::SurfaceFrame& frame,
	Multinet::SurfaceEdge edge,
	double& out_x,
	double& out_z
) noexcept {
	const double a00 = frame.tangent_basis.u_axis.x;
	const double a01 = frame.tangent_basis.u_axis.z;
	const double a10 = frame.tangent_basis.v_axis.x;
	const double a11 = frame.tangent_basis.v_axis.z;
	const double det = a00 * a11 - a01 * a10;
	if (!std::isfinite(det) || std::abs(det) < 1e-9) return false;

	double du = 0.0;
	double dv = 0.0;
	switch (edge) {
		case Multinet::SurfaceEdge::NegativeU: du = -1.0; break;
		case Multinet::SurfaceEdge::PositiveU: du = 1.0; break;
		case Multinet::SurfaceEdge::NegativeV: dv = -1.0; break;
		case Multinet::SurfaceEdge::PositiveV: dv = 1.0; break;
	}
	out_x = (a11 * du - a01 * dv) / det;
	out_z = (-a10 * du + a00 * dv) / det;
	const double length = std::sqrt(out_x * out_x + out_z * out_z);
	if (!std::isfinite(length) || length < 1e-9) return false;
	out_x /= length;
	out_z /= length;
	return true;
}
}

MultinetBCCMNode3D::MultinetBCCMNode3D() {
}

MultinetBCCMNode3D::~MultinetBCCMNode3D() {
	free_rendering();
}

void MultinetBCCMNode3D::_bind_methods() {
	// Recipe properties — use Godot names preserved from test.tscn where applicable.
	ClassDB::bind_method(D_METHOD("set_seed", "seed"), &MultinetBCCMNode3D::set_seed);
	ClassDB::bind_method(D_METHOD("get_seed"), &MultinetBCCMNode3D::get_seed);
	ClassDB::bind_method(D_METHOD("set_frequency", "frequency"), &MultinetBCCMNode3D::set_frequency);
	ClassDB::bind_method(D_METHOD("get_frequency"), &MultinetBCCMNode3D::get_frequency);
	ClassDB::bind_method(D_METHOD("set_regional_frequency", "regional_frequency"), &MultinetBCCMNode3D::set_regional_frequency);
	ClassDB::bind_method(D_METHOD("get_regional_frequency"), &MultinetBCCMNode3D::get_regional_frequency);
	ClassDB::bind_method(D_METHOD("set_detail_frequency", "detail_frequency"), &MultinetBCCMNode3D::set_detail_frequency);
	ClassDB::bind_method(D_METHOD("get_detail_frequency"), &MultinetBCCMNode3D::get_detail_frequency);
	ClassDB::bind_method(D_METHOD("set_min_elevation_m", "min_elevation_m"), &MultinetBCCMNode3D::set_min_elevation_m);
	ClassDB::bind_method(D_METHOD("get_min_elevation_m"), &MultinetBCCMNode3D::get_min_elevation_m);
	ClassDB::bind_method(D_METHOD("set_max_elevation_m", "max_elevation_m"), &MultinetBCCMNode3D::set_max_elevation_m);
	ClassDB::bind_method(D_METHOD("get_max_elevation_m"), &MultinetBCCMNode3D::get_max_elevation_m);
	ClassDB::bind_method(D_METHOD("set_octave_count", "octave_count"), &MultinetBCCMNode3D::set_octave_count);
	ClassDB::bind_method(D_METHOD("get_octave_count"), &MultinetBCCMNode3D::get_octave_count);
	ClassDB::bind_method(D_METHOD("set_persistence", "persistence"), &MultinetBCCMNode3D::set_persistence);
	ClassDB::bind_method(D_METHOD("get_persistence"), &MultinetBCCMNode3D::get_persistence);
	ClassDB::bind_method(D_METHOD("set_lacunarity", "lacunarity"), &MultinetBCCMNode3D::set_lacunarity);
	ClassDB::bind_method(D_METHOD("get_lacunarity"), &MultinetBCCMNode3D::get_lacunarity);
	ClassDB::bind_method(D_METHOD("set_coordinate_wrapping", "enabled"), &MultinetBCCMNode3D::set_coordinate_wrapping);
	ClassDB::bind_method(D_METHOD("get_coordinate_wrapping"), &MultinetBCCMNode3D::get_coordinate_wrapping);
	ClassDB::bind_method(D_METHOD("set_square_world", "enabled"), &MultinetBCCMNode3D::set_square_world);
	ClassDB::bind_method(D_METHOD("get_square_world"), &MultinetBCCMNode3D::get_square_world);
	ClassDB::bind_method(D_METHOD("set_world_side_km", "kilometres"), &MultinetBCCMNode3D::set_world_side_km);
	ClassDB::bind_method(D_METHOD("get_world_side_km"), &MultinetBCCMNode3D::get_world_side_km);
	ClassDB::bind_method(D_METHOD("set_world_extent_x_km", "kilometres"), &MultinetBCCMNode3D::set_world_extent_x_km);
	ClassDB::bind_method(D_METHOD("get_world_extent_x_km"), &MultinetBCCMNode3D::get_world_extent_x_km);
	ClassDB::bind_method(D_METHOD("set_world_extent_z_km", "kilometres"), &MultinetBCCMNode3D::set_world_extent_z_km);
	ClassDB::bind_method(D_METHOD("get_world_extent_z_km"), &MultinetBCCMNode3D::get_world_extent_z_km);
	ClassDB::bind_method(D_METHOD("set_closed_equivalent_side_km", "kilometres"), &MultinetBCCMNode3D::set_closed_equivalent_side_km);
	ClassDB::bind_method(D_METHOD("get_closed_equivalent_side_km"), &MultinetBCCMNode3D::get_closed_equivalent_side_km);
	ClassDB::bind_method(D_METHOD("set_closed_flat_coverage_radius_blocks", "radius"), &MultinetBCCMNode3D::set_closed_flat_coverage_radius_blocks);
	ClassDB::bind_method(D_METHOD("get_closed_flat_coverage_radius_blocks"), &MultinetBCCMNode3D::get_closed_flat_coverage_radius_blocks);
	ClassDB::bind_method(D_METHOD("get_closed_flat_visible_extent_km"), &MultinetBCCMNode3D::get_closed_flat_visible_extent_km);
	ClassDB::bind_method(D_METHOD("set_chp_enabled", "enabled"), &MultinetBCCMNode3D::set_chp_enabled);
	ClassDB::bind_method(D_METHOD("get_chp_enabled"), &MultinetBCCMNode3D::get_chp_enabled);
	ClassDB::bind_method(D_METHOD("set_chp_radius_policy", "policy"), &MultinetBCCMNode3D::set_chp_radius_policy);
	ClassDB::bind_method(D_METHOD("get_chp_radius_policy"), &MultinetBCCMNode3D::get_chp_radius_policy);
	ClassDB::bind_method(D_METHOD("set_chp_explicit_radius_km", "kilometres"), &MultinetBCCMNode3D::set_chp_explicit_radius_km);
	ClassDB::bind_method(D_METHOD("get_chp_explicit_radius_km"), &MultinetBCCMNode3D::get_chp_explicit_radius_km);
	ClassDB::bind_method(D_METHOD("get_canonical_area_km2"), &MultinetBCCMNode3D::get_canonical_area_km2);
	ClassDB::bind_method(D_METHOD("get_logical_radius_km"), &MultinetBCCMNode3D::get_logical_radius_km);
	ClassDB::bind_method(D_METHOD("get_closed_face_extent_km"), &MultinetBCCMNode3D::get_closed_face_extent_km);
	ClassDB::bind_method(D_METHOD("get_closed_face_half_extent_km"), &MultinetBCCMNode3D::get_closed_face_half_extent_km);
	ClassDB::bind_method(D_METHOD("get_regions_per_face_axis"), &MultinetBCCMNode3D::get_regions_per_face_axis);
	ClassDB::bind_method(D_METHOD("get_actual_region_extent_m"), &MultinetBCCMNode3D::get_actual_region_extent_m);
	ClassDB::bind_method(D_METHOD("get_active_domain_hash"), &MultinetBCCMNode3D::get_active_domain_hash);
	ClassDB::bind_method(D_METHOD("get_active_presentation_hash"), &MultinetBCCMNode3D::get_active_presentation_hash);
	ClassDB::bind_method(D_METHOD("get_active_domain_hash_text"), &MultinetBCCMNode3D::get_active_domain_hash_text);
	ClassDB::bind_method(D_METHOD("get_active_presentation_hash_text"), &MultinetBCCMNode3D::get_active_presentation_hash_text);
	ClassDB::bind_method(D_METHOD("get_domain_validation_message"), &MultinetBCCMNode3D::get_domain_validation_message);
	ClassDB::bind_method(D_METHOD("get_chp_status"), &MultinetBCCMNode3D::get_chp_status);
	ClassDB::bind_method(D_METHOD("set_chp_debug_reconstruction_mode", "mode"), &MultinetBCCMNode3D::set_chp_debug_reconstruction_mode);
	ClassDB::bind_method(D_METHOD("get_chp_debug_reconstruction_mode"), &MultinetBCCMNode3D::get_chp_debug_reconstruction_mode);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "chp_debug_reconstruction_mode"), "set_chp_debug_reconstruction_mode", "get_chp_debug_reconstruction_mode");

	ClassDB::bind_method(D_METHOD("set_chp_debug_negative_height_color", "enabled"), &MultinetBCCMNode3D::set_chp_debug_negative_height_color);
	ClassDB::bind_method(D_METHOD("get_chp_debug_negative_height_color"), &MultinetBCCMNode3D::get_chp_debug_negative_height_color);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "chp_debug_negative_height_color"), "set_chp_debug_negative_height_color", "get_chp_debug_negative_height_color");

	ClassDB::bind_method(D_METHOD("set_chp_debug_negative_height_exaggeration", "enabled"), &MultinetBCCMNode3D::set_chp_debug_negative_height_exaggeration);
	ClassDB::bind_method(D_METHOD("get_chp_debug_negative_height_exaggeration"), &MultinetBCCMNode3D::get_chp_debug_negative_height_exaggeration);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "chp_debug_negative_height_exaggeration"), "set_chp_debug_negative_height_exaggeration", "get_chp_debug_negative_height_exaggeration");

	ClassDB::bind_method(D_METHOD("set_bccm_debug_visual_mode", "mode"), &MultinetBCCMNode3D::set_bccm_debug_visual_mode);
	ClassDB::bind_method(D_METHOD("get_bccm_debug_visual_mode"), &MultinetBCCMNode3D::get_bccm_debug_visual_mode);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "bccm_debug_visual_mode"), "set_bccm_debug_visual_mode", "get_bccm_debug_visual_mode");


	ClassDB::bind_method(D_METHOD("set_freeze_update", "freeze"), &MultinetBCCMNode3D::set_freeze_update);
	ClassDB::bind_method(D_METHOD("get_freeze_update"), &MultinetBCCMNode3D::get_freeze_update);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "freeze_update"), "set_freeze_update", "get_freeze_update");

	ClassDB::bind_method(D_METHOD("get_candidate_count", "lod"), &MultinetBCCMNode3D::get_candidate_count);
	ClassDB::bind_method(D_METHOD("get_visible_count", "lod"), &MultinetBCCMNode3D::get_visible_count);
	ClassDB::bind_method(D_METHOD("get_submitted_streams"), &MultinetBCCMNode3D::get_submitted_streams);

	// GDScript-callable canonical state injection.
	ClassDB::bind_method(
		D_METHOD("set_canonical_camera_state_from_values", "face", "u_m", "v_m", "altitude_m", "frame_epoch"),
		&MultinetBCCMNode3D::set_canonical_camera_state_from_values
	);
	ClassDB::bind_method(
		D_METHOD("advance_canonical_observer", "presentation_dx", "altitude_dy", "presentation_dz", "delta_seconds"),
		&MultinetBCCMNode3D::advance_canonical_observer
	);

#ifdef DEBUG_ENABLED
	ClassDB::bind_method(D_METHOD("debug_publish_synthetic_edge_camera", "camera", "signed_distance_to_edge_m", "epoch"), &MultinetBCCMNode3D::debug_publish_synthetic_edge_camera);
	ClassDB::bind_method(D_METHOD("publish_editor_view_camera", "editor_camera"), &MultinetBCCMNode3D::publish_editor_view_camera);
	ClassDB::bind_method(D_METHOD("debug_publish_diagnostic_delta", "face", "u_m", "v_m", "radius_m", "amplitude_m", "content_version", "hold_generation"), &MultinetBCCMNode3D::debug_publish_diagnostic_delta);
	ClassDB::bind_method(D_METHOD("debug_publish_null_delta"), &MultinetBCCMNode3D::debug_publish_null_delta);
	ClassDB::bind_method(D_METHOD("debug_release_held_delta_generation"), &MultinetBCCMNode3D::debug_release_held_delta_generation);
	ClassDB::bind_method(D_METHOD("debug_get_block_state", "face", "block_u", "block_v", "lod"), &MultinetBCCMNode3D::debug_get_block_state);
#endif
	ClassDB::bind_method(D_METHOD("get_debug_summary"), &MultinetBCCMNode3D::get_debug_summary);

	// "seed" and "frequency" are the exact serialized names used in test.tscn.
	// "seed" -> world_seed in TerrainRecipeIdentity.
	// "frequency" -> continental_frequency in LegacyTerrainSignalBand.
	ADD_GROUP("Terrain Recipe", "");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "seed"), "set_seed", "get_seed");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "frequency", PROPERTY_HINT_RANGE, "0.0, 10.0, 0.0001"), "set_frequency", "get_frequency");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "reserved_regional_frequency", PROPERTY_HINT_RANGE, "0.0, 10.0, 0.0001", PROPERTY_USAGE_NO_EDITOR), "set_regional_frequency", "get_regional_frequency");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "reserved_detail_frequency", PROPERTY_HINT_RANGE, "0.0, 10.0, 0.0001", PROPERTY_USAGE_NO_EDITOR), "set_detail_frequency", "get_detail_frequency");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "min_elevation_m", PROPERTY_HINT_RANGE, "-20000.0, 20000.0, 1.0"), "set_min_elevation_m", "get_min_elevation_m");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_elevation_m", PROPERTY_HINT_RANGE, "-20000.0, 20000.0, 1.0"), "set_max_elevation_m", "get_max_elevation_m");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "octave_count", PROPERTY_HINT_RANGE, "1, 12, 1"), "set_octave_count", "get_octave_count");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "persistence", PROPERTY_HINT_RANGE, "0.0, 1.0, 0.01"), "set_persistence", "get_persistence");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "lacunarity", PROPERTY_HINT_RANGE, "1.0, 4.0, 0.01"), "set_lacunarity", "get_lacunarity");
	ADD_GROUP("World Domain", "");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "coordinate_wrapping"), "set_coordinate_wrapping", "get_coordinate_wrapping");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "square_world"), "set_square_world", "get_square_world");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "world_side_km", PROPERTY_HINT_RANGE, "0.001, 4294967.295, 0.001"), "set_world_side_km", "get_world_side_km");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "world_extent_x_km", PROPERTY_HINT_RANGE, "0.001, 4294967.295, 0.001"), "set_world_extent_x_km", "get_world_extent_x_km");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "world_extent_z_km", PROPERTY_HINT_RANGE, "0.001, 4294967.295, 0.001"), "set_world_extent_z_km", "get_world_extent_z_km");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "closed_equivalent_side_km", PROPERTY_HINT_RANGE, "0.079, 4294967.295, 0.001"), "set_closed_equivalent_side_km", "get_closed_equivalent_side_km");
	ADD_GROUP("World Presentation", "");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "closed_flat_coverage_radius_blocks", PROPERTY_HINT_RANGE, "1, 8, 1"), "set_closed_flat_coverage_radius_blocks", "get_closed_flat_coverage_radius_blocks");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "closed_flat_visible_extent_km", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR | PROPERTY_USAGE_READ_ONLY), "", "get_closed_flat_visible_extent_km");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "chp_enabled"), "set_chp_enabled", "get_chp_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "chp_radius_policy", PROPERTY_HINT_ENUM, "CanonicalClosedSurface:0,AreaEquivalent:1,Explicit:2"), "set_chp_radius_policy", "get_chp_radius_policy");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "chp_explicit_radius_km", PROPERTY_HINT_RANGE, "0.001, 4294967.295, 0.001"), "set_chp_explicit_radius_km", "get_chp_explicit_radius_km");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "chp_status", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR | PROPERTY_USAGE_READ_ONLY), "", "get_chp_status");
	ADD_GROUP("World Domain (Derived)", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "canonical_area_km2", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR | PROPERTY_USAGE_READ_ONLY), "", "get_canonical_area_km2");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "logical_radius_km", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR | PROPERTY_USAGE_READ_ONLY), "", "get_logical_radius_km");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "closed_face_extent_km", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR | PROPERTY_USAGE_READ_ONLY), "", "get_closed_face_extent_km");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "closed_face_half_extent_km", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR | PROPERTY_USAGE_READ_ONLY), "", "get_closed_face_half_extent_km");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "regions_per_face_axis", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR | PROPERTY_USAGE_READ_ONLY), "", "get_regions_per_face_axis");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "actual_region_extent_m", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR | PROPERTY_USAGE_READ_ONLY), "", "get_actual_region_extent_m");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "active_domain_hash", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR | PROPERTY_USAGE_READ_ONLY), "", "get_active_domain_hash_text");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "active_presentation_hash", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR | PROPERTY_USAGE_READ_ONLY), "", "get_active_presentation_hash_text");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "domain_validation_message", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR | PROPERTY_USAGE_READ_ONLY), "", "get_domain_validation_message");
	ClassDB::bind_method(D_METHOD("set_source_mode", "mode"), &MultinetBCCMNode3D::set_source_mode);
	ClassDB::bind_method(D_METHOD("get_source_mode"), &MultinetBCCMNode3D::get_source_mode);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "source_mode", PROPERTY_HINT_ENUM, "AnalyticBase:0,AbsoluteHeightPageDebug:1,HybridAdditiveDelta:2"), "set_source_mode", "get_source_mode");

	ClassDB::bind_method(D_METHOD("set_analytic_debug_prewarm_pages", "prewarm"), &MultinetBCCMNode3D::set_analytic_debug_prewarm_pages);
	ClassDB::bind_method(D_METHOD("get_analytic_debug_prewarm_pages"), &MultinetBCCMNode3D::get_analytic_debug_prewarm_pages);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "analytic_debug_prewarm_pages"), "set_analytic_debug_prewarm_pages", "get_analytic_debug_prewarm_pages");
	ClassDB::bind_method(D_METHOD("set_face_colors_enabled", "enabled"), &MultinetBCCMNode3D::set_face_colors_enabled);
	ClassDB::bind_method(D_METHOD("get_face_colors_enabled"), &MultinetBCCMNode3D::get_face_colors_enabled);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "face_colors_enabled"), "set_face_colors_enabled", "get_face_colors_enabled");
	ClassDB::bind_method(D_METHOD("set_diamond_triangulation_enabled", "enabled"), &MultinetBCCMNode3D::set_diamond_triangulation_enabled);
	ClassDB::bind_method(D_METHOD("get_diamond_triangulation_enabled"), &MultinetBCCMNode3D::get_diamond_triangulation_enabled);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "diamond_triangulation_enabled"), "set_diamond_triangulation_enabled", "get_diamond_triangulation_enabled");

	ClassDB::bind_method(D_METHOD("set_camera_target", "path"), &MultinetBCCMNode3D::set_camera_target);
	ClassDB::bind_method(D_METHOD("get_camera_target"), &MultinetBCCMNode3D::get_camera_target);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "camera_target"), "set_camera_target", "get_camera_target");

	ClassDB::bind_method(D_METHOD("set_high_speed_cut_diagnostics_enabled", "enabled"), &MultinetBCCMNode3D::set_high_speed_cut_diagnostics_enabled);
	ClassDB::bind_method(D_METHOD("get_high_speed_cut_diagnostics_enabled"), &MultinetBCCMNode3D::get_high_speed_cut_diagnostics_enabled);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "high_speed_cut_diagnostics_enabled"), "set_high_speed_cut_diagnostics_enabled", "get_high_speed_cut_diagnostics_enabled");
}


void MultinetBCCMNode3D::_validate_property(PropertyInfo& p_property) const {
	const StringName name = p_property.name;
	const bool wrapping = get_coordinate_wrapping();
	const bool finite_square = !wrapping && square_world;
	const bool finite_rectangle = !wrapping && !square_world;
	const bool chp_visible = world_presentation_input.chp_enabled && !wrapping;

	auto hide = [&p_property]() {
		// Keep inactive values serialized so switching modes does not lose the
		// user's previous configuration, but remove them from the Inspector.
		p_property.usage = PROPERTY_USAGE_STORAGE;
	};

	if (name == StringName("square_world") && wrapping) hide();
	if (name == StringName("world_side_km") && !finite_square) hide();
	if ((name == StringName("world_extent_x_km") || name == StringName("world_extent_z_km")) && !finite_rectangle) hide();
	if (name == StringName("closed_equivalent_side_km") && !wrapping) hide();
	if ((name == StringName("closed_flat_coverage_radius_blocks") || name == StringName("closed_flat_visible_extent_km")) && !wrapping) hide();

	if (name == StringName("logical_radius_km") && !wrapping) hide();
	if ((name == StringName("closed_face_extent_km") || name == StringName("closed_face_half_extent_km") ||
			 name == StringName("regions_per_face_axis") || name == StringName("actual_region_extent_m")) && !wrapping) hide();

	if ((name == StringName("chp_radius_policy") || name == StringName("chp_explicit_radius_km")) && !chp_visible) hide();
}

bool MultinetBCCMNode3D::configure_bccm_profile() {
	if (bccm_renderer.set_candidate_grid_radius(closed_flat_coverage_radius_blocks)) return true;
	std::cerr << "[MultinetBCCMNode3D] invalid or late closed flat coverage radius: "
		<< closed_flat_coverage_radius_blocks << std::endl;
	return false;
}

void MultinetBCCMNode3D::refresh_chp_view() {
	current_chp_view = {};
	if (!world_domain_manifest.is_valid() || !world_presentation_manifest.is_valid() ||
		current_cam_state.frame_epoch == 0 || !current_cam_state.canonical_position.is_valid()) return;
	Multinet::FramePosition64 camera_in_frame{};
	if (!Multinet::try_domain_surface_to_frame(
		current_cam_state.canonical_position,
		current_cam_state.active_frame,
		world_domain_manifest,
		camera_in_frame)) return;
	const uint64_t epoch = current_cam_state.frame_epoch;
	const uint32_t camera_epoch = epoch > static_cast<uint64_t>((std::numeric_limits<uint32_t>::max)())
		? (std::numeric_limits<uint32_t>::max)()
		: static_cast<uint32_t>(epoch);
	const uint32_t source_epoch = source_expectation.source_version == 0 ? 1 : source_expectation.source_version;
	const bool view_valid = multinet::rendering::chp::try_build_curved_horizon_view(
		world_domain_manifest,
		world_presentation_manifest,
		resolved_chp_profile,
		current_cam_state.canonical_position,
		camera_in_frame,
		epoch,
		camera_epoch,
		source_epoch,
		current_chp_view);
	if (!view_valid) current_chp_view = {};
}

void MultinetBCCMNode3D::refresh_chp_contract() {
	const double ordinary_coverage_m = bccm_renderer.get_effective_coverage_extent_m();
	chp_profile.requested_maximum_deformation_distance_m =
		std::isfinite(ordinary_coverage_m) && ordinary_coverage_m > 0.0 ? ordinary_coverage_m : 1.0;
	resolved_chp_profile = {};
	if (world_presentation_manifest.is_valid() && world_presentation_manifest.chp_enabled) {
		const bool profile_valid = multinet::rendering::chp::try_resolve_curved_horizon_profile(
			world_presentation_manifest,
			chp_profile,
			resolved_chp_profile);
		if (!profile_valid) resolved_chp_profile = {};
	}
	refresh_chp_view();
}

// ---------------------------------------------------------------------------
// Recipe rebuild path
// ---------------------------------------------------------------------------

void MultinetBCCMNode3D::request_recipe_rebuild() {
	source_dirty = true;
	recipe_rebuild_pending = true;
}

void MultinetBCCMNode3D::apply_recipe_rebuild() {
	if (!recipe_rebuild_pending) return;
	recipe_rebuild_pending = false;
	domain_rebuild_pending = false;
	presentation_rebuild_pending = false;

	// 1. Clean up renderer resources.
	bccm_renderer.cleanup();

	// 2. Shut down and reset the old source (in-flight jobs drain first).
	if (render_source) {
		render_source->shutdown();
		render_source.reset();
	}

	// 3. Executor is preserved — not destroyed.

	// 4. Finalize recipe against manifest (rebuilds hash).
	world_domain_manifest = Multinet::build_world_domain_manifest(world_domain_input);
	world_presentation_manifest = Multinet::build_world_presentation_manifest(world_domain_manifest, world_presentation_input);
	manifest = make_compatibility_scale(world_domain_manifest);
	if (!manifest.is_valid()) {
		domain_validation_message = "World domain manifest is invalid; check the configured dimensions and integer range.";
		std::cerr << "[MultinetBCCMNode3D] apply_recipe_rebuild: invalid manifest." << std::endl;
		return;
	}
	if (!Multinet::finalize_terrain_recipe(recipe, world_domain_manifest)) {
		domain_validation_message = "Terrain recipe could not be finalized for the active world domain.";
		std::cerr << "[MultinetBCCMNode3D] apply_recipe_rebuild: finalize_terrain_recipe failed." << std::endl;
		return;
	}

	// 5. Ensure executor is running.
	if (!executor) {
		executor = std::make_unique<Multinet::BoundedBackgroundJobExecutor>();
	}

	// 6. Construct new ConcreteTerrainRenderSource.
	render_source = std::make_unique<Multinet::ConcreteTerrainRenderSource>(
		recipe, world_domain_manifest, *executor
	);

	// 7. Rebuild BCCMSourceExpectation.
	build_source_expectation();

	manifest_built = true;
	source_dirty = false;
	domain_rebuild_pending = false;
	domain_validation_message = "OK";

	// 8. Reinitialize renderer.
	if (get_world_3d().is_valid()) {
		auto* rs = godot::RenderingServer::get_singleton();
		Multinet::TerrainFallbackBounds fb = render_source->get_snapshot().fallback_bounds;
		if (!configure_bccm_profile() || !bccm_renderer.initialize(rs, get_world_3d()->get_scenario(), world_domain_manifest, recipe.identity, fb)) {
			std::cerr << "[MultinetBCCMNode3D] apply_recipe_rebuild: renderer initialization failed." << std::endl;
		} else {
			bccm_renderer.bind_material_uniforms(recipe, world_domain_manifest);
		}
	}
	refresh_chp_contract();
}

// ---------------------------------------------------------------------------
// Internal pipeline
// ---------------------------------------------------------------------------

void MultinetBCCMNode3D::refresh_source_expectation_from_source() {
	if (render_source) {
		Multinet::TerrainRenderSourceSnapshot snap = render_source->get_snapshot();
		source_expectation.recipe_identity = snap.recipe_identity;
		source_expectation.world_manifest_hash = snap.world_manifest_hash;
		source_expectation.topology_version = snap.topology_version;
		source_expectation.projection_version = snap.projection_version;
		source_expectation.terrain_version = snap.terrain_version;
		source_expectation.source_version = snap.source_version;
	} else {
		source_expectation.recipe_identity = recipe.identity;
		source_expectation.world_manifest_hash = manifest.manifest_hash;
		source_expectation.topology_version = manifest.topology_version;
		source_expectation.projection_version = manifest.projection_version;
		source_expectation.terrain_version = 1;
		source_expectation.source_version = 1;
	}
}

void MultinetBCCMNode3D::build_source_expectation() {
	refresh_source_expectation_from_source();
}

void MultinetBCCMNode3D::rebuild_source() {
	if (!source_dirty) return;

	// 1. Build manifest first — abort if invalid.
	world_domain_manifest = Multinet::build_world_domain_manifest(world_domain_input);
	world_presentation_manifest = Multinet::build_world_presentation_manifest(world_domain_manifest, world_presentation_input);
	manifest = make_compatibility_scale(world_domain_manifest);
	if (!manifest.is_valid()) {
		domain_validation_message = "World domain manifest is invalid; check the configured dimensions and integer range.";
		return;
	}

	// 2. Finalize recipe against the manifest — abort if it fails.
	if (!Multinet::finalize_terrain_recipe(recipe, world_domain_manifest)) {
		domain_validation_message = "Terrain recipe could not be finalized for the active world domain.";
		return;
	}

	// 3. Shut down any existing source (waits for in-flight jobs).
	if (render_source) {
		render_source->shutdown();
		render_source.reset();
	}

	// 4. Ensure executor is running.
	if (!executor) {
		executor = std::make_unique<Multinet::BoundedBackgroundJobExecutor>();
	}

	// 5. Construct source with executor reference.
	render_source = std::make_unique<Multinet::ConcreteTerrainRenderSource>(
		recipe, world_domain_manifest, *executor
	);

	build_source_expectation();

	manifest_built = true;
	source_dirty = false;
	domain_validation_message = "OK";
	refresh_chp_contract();
}

void MultinetBCCMNode3D::set_canonical_camera_state(
	const Multinet::SurfacePosition64& p_canonical_pos,
	const Multinet::SurfaceFrame& p_active_frame,
	uint64_t p_frame_epoch
) {
	current_cam_state.canonical_position = p_canonical_pos;
	current_cam_state.active_frame = p_active_frame;
	current_cam_state.frame_epoch = p_frame_epoch;
	current_cam_state.is_visible = is_visible_in_tree();
	refresh_chp_view();
}

void MultinetBCCMNode3D::set_canonical_camera_state_from_values(
	int p_face,
	double p_u_m,
	double p_v_m,
	double p_altitude_m,
	uint64_t p_frame_epoch
) {
	if (!manifest_built) return;

	Multinet::SurfacePosition64 pos;
	pos.face = static_cast<Multinet::SurfaceFace>(p_face < 0 ? 0 : (p_face > 5 ? 5 : p_face));
	pos.u_m = p_u_m;
	pos.v_m = p_v_m;
	pos.altitude_m = p_altitude_m;
	pos.topology_version = manifest.topology_version;
	pos.projection_version = manifest.projection_version;

	Multinet::SurfaceFrame frame = make_editor_frame_for_face(
			world_domain_manifest.input.topology,
			pos.face,
			pos,
			p_frame_epoch,
			manifest.topology_version,
			manifest.projection_version
		);

	set_canonical_camera_state(pos, frame, p_frame_epoch);
	current_cam_state.presentation_x_m = 0.0;
	current_cam_state.presentation_z_m = 0.0;
	current_cam_state.unfolding_generation = current_cam_state.unfolding_generation == (std::numeric_limits<uint64_t>::max)()
		? 1
		: current_cam_state.unfolding_generation + 1;
	current_cam_state.has_presentation_position = true;
	current_cam_state.unfolding_root_frame = frame;
	current_cam_state.unfolding_root_frame.origin.altitude_m = 0.0;
	current_cam_state.unfolding_root_presentation_x_m = 0.0;
	current_cam_state.unfolding_root_presentation_z_m = 0.0;
	current_cam_state.has_unfolding_root = true;
	refresh_tracking_logical_chart(
		world_domain_manifest,
		current_cam_state.unfolding_root_frame,
		current_cam_state.has_logical_chart,
		current_cam_state.logical_chart_root_direction,
		current_cam_state.logical_chart_presentation_x_tangent,
		current_cam_state.logical_chart_presentation_z_tangent);
	refresh_chp_view();
}

godot::Dictionary MultinetBCCMNode3D::advance_canonical_observer(
	double p_presentation_dx,
	double p_altitude_dy,
	double p_presentation_dz,
	double p_delta_seconds
) {
	godot::Dictionary result;
	result["valid"] = false;
	if (!manifest_built || !world_domain_manifest.is_valid() ||
		!std::isfinite(p_presentation_dx) || !std::isfinite(p_altitude_dy) || !std::isfinite(p_presentation_dz) ||
		!std::isfinite(p_delta_seconds) || p_delta_seconds < 0.0) {
		return result;
	}
	if (!current_cam_state.has_presentation_position) {
		current_cam_state.presentation_x_m = 0.0;
		current_cam_state.presentation_z_m = 0.0;
		current_cam_state.unfolding_generation = std::max<uint64_t>(1, current_cam_state.unfolding_generation);
		current_cam_state.has_presentation_position = true;
	}

	if (current_cam_state.frame_epoch == 0 || !current_cam_state.canonical_position.is_valid()) {
		current_cam_state.canonical_position.face = Multinet::SurfaceFace::PositiveX;
		current_cam_state.canonical_position.u_m = 0.0;
		current_cam_state.canonical_position.v_m = 0.0;
		current_cam_state.canonical_position.altitude_m = 3000.0;
		current_cam_state.canonical_position.topology_version = world_domain_manifest.topology_version;
		current_cam_state.canonical_position.projection_version = world_domain_manifest.projection_version;
		current_cam_state.frame_epoch = 1;
		current_cam_state.active_frame = make_editor_frame_for_face(
			world_domain_input.topology,
			current_cam_state.canonical_position.face,
			current_cam_state.canonical_position,
			current_cam_state.frame_epoch,
			world_domain_manifest.topology_version,
			world_domain_manifest.projection_version
		);
	}
	if (!current_cam_state.has_unfolding_root) {
		current_cam_state.unfolding_root_frame = current_cam_state.active_frame;
		current_cam_state.unfolding_root_frame.origin.altitude_m = 0.0;
		current_cam_state.unfolding_root_presentation_x_m = current_cam_state.presentation_x_m;
		current_cam_state.unfolding_root_presentation_z_m = current_cam_state.presentation_z_m;
		current_cam_state.has_unfolding_root = true;
		refresh_tracking_logical_chart(
			world_domain_manifest,
			current_cam_state.unfolding_root_frame,
			current_cam_state.has_logical_chart,
			current_cam_state.logical_chart_root_direction,
			current_cam_state.logical_chart_presentation_x_tangent,
			current_cam_state.logical_chart_presentation_z_tangent);
	}

	Multinet::SurfaceFrame next_frame;
	Multinet::SurfacePosition64 next_position;
	uint32_t transition_count = 0;
	Multinet::SurfaceFace last_source = current_cam_state.canonical_position.face;
	Multinet::SurfaceFace last_destination = last_source;
	Multinet::SurfaceEdge last_edge = Multinet::SurfaceEdge::NegativeU;
	Multinet::FramePosition64 local_delta{};
	const bool closed_chart_motion = !world_domain_manifest.is_finite() &&
		current_cam_state.has_logical_chart;
	if (closed_chart_motion) {
		// The visible V5 surface and the observer must consume the same flat
		// presentation vector. Using the cube frame here lets a held key walk away
		// from the terrain direction at a three-face corner.
		if (!try_map_closed_presentation_motion(
			world_domain_manifest,
			current_cam_state.canonical_position,
			current_cam_state.has_logical_chart,
			current_cam_state.logical_chart_root_direction,
			current_cam_state.logical_chart_presentation_x_tangent,
			current_cam_state.logical_chart_presentation_z_tangent,
			p_presentation_dx,
			p_presentation_dz,
			local_delta
		)) {
			return result;
		}
	} else {
		const double canonical_local_u =
			current_cam_state.active_frame.tangent_basis.u_axis.x * p_presentation_dx +
			current_cam_state.active_frame.tangent_basis.u_axis.z * p_presentation_dz;
		const double canonical_local_v =
			current_cam_state.active_frame.tangent_basis.v_axis.x * p_presentation_dx +
			current_cam_state.active_frame.tangent_basis.v_axis.z * p_presentation_dz;
		local_delta = Multinet::FramePosition64{ canonical_local_u, 0.0, canonical_local_v };
	}
	local_delta.y = p_altitude_dy;
	const double presentation_dx = p_presentation_dx;
	const double presentation_dz = p_presentation_dz;
	if (!Multinet::try_advance_domain_surface_frame(
		local_delta,
		world_domain_manifest,
		current_cam_state.active_frame,
		next_frame,
		next_position,
		transition_count,
		&last_source,
		&last_destination,
		&last_edge
	)) {
		runtime_outside_finite_boundary = world_domain_manifest.is_finite();
		result["outside_finite_boundary"] = runtime_outside_finite_boundary;
		return result;
	}

	current_cam_state.canonical_position = next_position;
	current_cam_state.active_frame = next_frame;
	current_cam_state.frame_epoch = next_frame.frame_epoch;
	current_cam_state.is_visible = is_visible_in_tree();
	current_cam_state.presentation_x_m += presentation_dx;
	current_cam_state.presentation_z_m += presentation_dz;
	if (!world_domain_manifest.is_finite()) {
		const bool analytic_chart_can_track =
			bccm_renderer.get_source_mode() == multinet::rendering::TerrainSourceMode::AnalyticBase &&
			!bccm_renderer.get_analytic_debug_prewarm_pages();
		if (analytic_chart_can_track) {
			// AnalyticBase has no chart-addressed page payload to invalidate. Keep
			// V5 local every movement tick rather than letting its exponential map
			// fold the flat viewport after a long run.
			current_cam_state.unfolding_root_frame = next_frame;
			current_cam_state.unfolding_root_frame.origin.altitude_m = 0.0;
			current_cam_state.unfolding_root_presentation_x_m = current_cam_state.presentation_x_m;
			current_cam_state.unfolding_root_presentation_z_m = current_cam_state.presentation_z_m;
			refresh_tracking_logical_chart(
				world_domain_manifest,
				current_cam_state.unfolding_root_frame,
				current_cam_state.has_logical_chart,
				current_cam_state.logical_chart_root_direction,
				current_cam_state.logical_chart_presentation_x_tangent,
				current_cam_state.logical_chart_presentation_z_tangent);
		}
	}
	runtime_outside_finite_boundary = false;
	runtime_last_transition_count = transition_count;
	runtime_last_transition_source_face = static_cast<int>(last_source);
	runtime_last_transition_destination_face = static_cast<int>(last_destination);
	runtime_last_transition_edge = transition_count > 0 ? static_cast<int>(last_edge) : -1;
	if (p_delta_seconds > 0.0) {
		runtime_ground_speed_m_s = std::sqrt(presentation_dx * presentation_dx + presentation_dz * presentation_dz) / p_delta_seconds;
		runtime_vertical_speed_m_s = p_altitude_dy / p_delta_seconds;
		runtime_total_speed_m_s = std::sqrt(
			presentation_dx * presentation_dx + presentation_dz * presentation_dz + p_altitude_dy * p_altitude_dy) / p_delta_seconds;
	} else {
		runtime_ground_speed_m_s = 0.0;
		runtime_vertical_speed_m_s = 0.0;
		runtime_total_speed_m_s = 0.0;
	}

	refresh_chp_view();

	result["valid"] = true;
	result["face"] = static_cast<int>(next_position.face);
	result["u_m"] = next_position.u_m;
	result["v_m"] = next_position.v_m;
	result["altitude_m"] = next_position.altitude_m;
	result["frame_epoch"] = static_cast<int64_t>(next_frame.frame_epoch);
	result["transition_count"] = transition_count;
	result["presentation_x_m"] = current_cam_state.presentation_x_m;
	result["presentation_z_m"] = current_cam_state.presentation_z_m;
	result["ground_speed_m_s"] = runtime_ground_speed_m_s;
	result["vertical_speed_m_s"] = runtime_vertical_speed_m_s;
	result["total_speed_m_s"] = runtime_total_speed_m_s;
	result["last_transition_source_face"] = static_cast<int>(last_source);
	result["last_transition_destination_face"] = static_cast<int>(last_destination);
	result["last_transition_edge"] = static_cast<int>(last_edge);
	// Horizontal position is rebased around the observer; altitude is real.
	// The flat basis must not rotate when canonical authority changes faces.
	result["presentation_position"] = godot::Vector3(0.0f, static_cast<float>(next_position.altitude_m), 0.0f);
	result["presentation_u_axis"] = godot::Vector3(1.0f, 0.0f, 0.0f);
	result["presentation_up_axis"] = godot::Vector3(0.0f, 1.0f, 0.0f);
	result["presentation_v_axis"] = godot::Vector3(0.0f, 0.0f, 1.0f);
	return result;
}

#ifdef DEBUG_ENABLED
void MultinetBCCMNode3D::debug_publish_synthetic_edge_camera(godot::Camera3D* p_camera, double p_signed_distance_to_edge_m, uint64_t p_epoch) {
	if (!p_camera || !manifest_built) return;

	double extent = static_cast<double>(manifest.chart_half_extent_mm) * 0.001;
	double distance_to_edge = p_signed_distance_to_edge_m;
	
	Multinet::SurfacePosition64 pos;
	pos.face = Multinet::SurfaceFace::PositiveX;
	pos.v_m = 0.0;
	pos.altitude_m = 50.0;
	pos.topology_version = manifest.topology_version;
	pos.projection_version = manifest.projection_version;

	if (distance_to_edge >= 0.0) {
		pos.u_m = extent - distance_to_edge;
	} else {
		// Crossed the edge! We map the coordinates explicitly via the transition table.
		const Multinet::EdgeTransition& trans = Multinet::get_edge_transition(static_cast<uint8_t>(Multinet::SurfaceFace::PositiveX), Multinet::SurfaceEdge::PositiveU);
		pos.face = static_cast<Multinet::SurfaceFace>(trans.destination_face);
		
		double overshoot = -distance_to_edge;
		double dest_param = 0.0 * trans.parameter_sign; // source v_m was 0
		double new_fixed_axis_val = (trans.dest_fixed_coordinate == 1) ? (extent - overshoot) : (-extent + overshoot);
		
		if (trans.destination_parameter_axis == 0) {
			pos.u_m = dest_param;
			pos.v_m = new_fixed_axis_val;
		} else {
			pos.u_m = new_fixed_axis_val;
			pos.v_m = dest_param;
		}
	}

	Multinet::SurfaceFrame frame;
	frame.origin.face = pos.face;
	frame.origin.u_m = 0.0;
	frame.origin.v_m = 0.0;
	frame.origin.altitude_m = 0.0;
	frame.origin.topology_version = manifest.topology_version;
	frame.origin.projection_version = manifest.projection_version;

	frame.tangent_basis.u_axis = { 1.0, 0.0, 0.0 };
	frame.tangent_basis.v_axis = { 0.0, 0.0, 1.0 };
	frame.tangent_basis.up_axis = { 0.0, 1.0, 0.0 };

	set_canonical_camera_state(pos, frame, p_epoch);

	godot::Transform3D xform = p_camera->get_global_transform();
	xform.origin = godot::Vector3(pos.u_m, pos.altitude_m, pos.v_m);
	p_camera->set_global_transform(xform);
}

godot::Dictionary MultinetBCCMNode3D::debug_publish_diagnostic_delta(
	int p_face,
	double p_u_m,
	double p_v_m,
	double p_radius_m,
	float p_amplitude_m,
	uint32_t p_content_version,
	bool p_hold_generation
) {
	godot::Dictionary result;
	if (!manifest_built || !render_source || p_radius_m <= 0.0 || p_content_version == 0) return result;
	if (p_hold_generation) debug_release_held_delta_generation();

	const int face_index = std::clamp(p_face, 0, 5);
	Multinet::SurfacePosition64 center;
	center.face = static_cast<Multinet::SurfaceFace>(face_index);
	center.u_m = p_u_m;
	center.v_m = p_v_m;
	center.altitude_m = 0.0;
	center.topology_version = manifest.topology_version;
	center.projection_version = manifest.projection_version;

	std::shared_ptr<Multinet::TerrainCommittedDeltaField> field;
	if (p_hold_generation) {
		field = std::make_shared<DebugBlockingDiagnosticDeltaField>(center, p_radius_m, p_amplitude_m, manifest, p_content_version);
		debug_held_delta_field = field;
	} else {
		field = std::make_shared<Multinet::CanonicalDiagnosticTerrainCommittedDeltaField>(center, p_radius_m, p_amplitude_m, manifest, p_content_version);
		debug_held_delta_field.reset();
	}

	Multinet::TerrainCommittedDeltaSnapshot snapshot;
	snapshot.contract_version = Multinet::TERRAIN_PAGE_CONTRACT_VERSION_1;
	snapshot.publication_version = ++debug_publication_version;
	snapshot.minimum_delta_m = std::min(0.0f, p_amplitude_m);
	snapshot.maximum_delta_m = std::max(0.0f, p_amplitude_m);
	snapshot.maximum_abs_gradient = p_radius_m > 0.0
		? static_cast<float>(0.5 * std::abs(p_amplitude_m) * 3.14159265358979323846 / p_radius_m)
		: 0.0f;
	snapshot.field = field;
	render_source->set_payload_kind(Multinet::TerrainPagePayloadKind::AdditiveHeightDeltaV1);
	render_source->set_committed_delta_snapshot(snapshot);
	refresh_source_expectation_from_source();
	bccm_renderer.set_source_mode(multinet::rendering::TerrainSourceMode::HybridAdditiveDelta);

	multinet::rendering::BlockClipmapProfile profile;
	int32_t block_u = static_cast<int32_t>(std::floor(p_u_m / profile.lod0_block_size));
	int32_t block_v = static_cast<int32_t>(std::floor(p_v_m / profile.lod0_block_size));
	const auto key = multinet::rendering::make_canonical_block_key(center.face, block_u, block_v, 0, manifest);
	result["face"] = face_index;
	result["block_u"] = key.block_u;
	result["block_v"] = key.block_v;
	result["lod"] = 0;
	result["publication_version"] = snapshot.publication_version;
	result["content_version"] = p_content_version;
	result["hold_generation"] = p_hold_generation;
	result["amplitude_m"] = p_amplitude_m;
	return result;
}

void MultinetBCCMNode3D::debug_publish_null_delta() {
	if (!manifest_built || !render_source) return;
	debug_release_held_delta_generation();
	Multinet::TerrainCommittedDeltaSnapshot snapshot;
	snapshot.contract_version = Multinet::TERRAIN_PAGE_CONTRACT_VERSION_1;
	snapshot.publication_version = ++debug_publication_version;
	snapshot.minimum_delta_m = 0.0f;
	snapshot.maximum_delta_m = 0.0f;
	snapshot.maximum_abs_gradient = 0.0f;
	snapshot.field = std::make_shared<Multinet::NullTerrainCommittedDeltaField>();
	render_source->set_payload_kind(Multinet::TerrainPagePayloadKind::AdditiveHeightDeltaV1);
	render_source->set_committed_delta_snapshot(snapshot);
	refresh_source_expectation_from_source();
}

void MultinetBCCMNode3D::debug_release_held_delta_generation() {
	if (!debug_held_delta_field) return;
	if (auto held = std::dynamic_pointer_cast<DebugBlockingDiagnosticDeltaField>(debug_held_delta_field)) {
		held->release_generation();
	}
	debug_held_delta_field.reset();
}

godot::Dictionary MultinetBCCMNode3D::debug_get_block_state(int p_face, int p_block_u, int p_block_v, int p_lod) const {
	godot::Dictionary result;
	const int face_index = std::clamp(p_face, 0, 5);
	multinet::rendering::TerrainRenderBlockKey key{
		static_cast<Multinet::SurfaceFace>(face_index),
		static_cast<int32_t>(p_block_u),
		static_cast<int32_t>(p_block_v),
		static_cast<uint8_t>(std::max(0, p_lod)),
		multinet::rendering::ORDINARY_BCCM_V1_PROFILE,
		0
	};
	const auto state = bccm_renderer.get_debug_block_state(key);
	const auto resolution_name = [](multinet::rendering::ResolutionClass c) {
		switch (c) {
			case multinet::rendering::ResolutionClass::Analytic: return godot::String("Analytic");
			case multinet::rendering::ResolutionClass::ExactResident: return godot::String("ExactResident");
			case multinet::rendering::ResolutionClass::ExactReadyEmpty: return godot::String("ExactReadyEmpty");
			case multinet::rendering::ResolutionClass::StalePrevious: return godot::String("StalePrevious");
			case multinet::rendering::ResolutionClass::AbsoluteResident: return godot::String("AbsoluteResident");
			case multinet::rendering::ResolutionClass::AbsoluteAnalyticFallback: return godot::String("AbsoluteAnalyticFallback");
			default: return godot::String("NoContent");
		}
	};
	const auto slot_name = [](multinet::rendering::TerrainGpuPageState s) {
		switch (s) {
			case multinet::rendering::TerrainGpuPageState::UploadPending: return godot::String("UploadPending");
			case multinet::rendering::TerrainGpuPageState::Resident: return godot::String("Resident");
			case multinet::rendering::TerrainGpuPageState::Retiring: return godot::String("Retiring");
			default: return godot::String("Free");
		}
	};
	result["submitted"] = state.submitted;
	result["face"] = face_index;
	result["block_u"] = key.block_u;
	result["block_v"] = key.block_v;
	result["lod"] = key.lod;
	result["selected_layer"] = state.selected_gpu_layer;
	result["resolution_class"] = resolution_name(state.resolution_class);
	result["requested_content_version"] = state.requested_content_version;
	result["selected_content_version"] = state.selected_content_version;
	result["selected_slot_state"] = slot_name(state.selected_slot_state);
	result["resident_same_block_count"] = state.resident_same_block_count;
	result["upload_pending_same_block_count"] = state.upload_pending_same_block_count;
	result["retiring_same_block_count"] = state.retiring_same_block_count;
	result["retiring_content_version"] = state.retiring_content_version;
	result["retire_after_frame"] = static_cast<int64_t>(state.retire_after_frame);
	return result;
}
#endif

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void MultinetBCCMNode3D::_notification(int p_what) {
	if (p_what == NOTIFICATION_READY) {
		// Construct executor early so it is available for rebuild_source.
		if (!executor) {
			executor = std::make_unique<Multinet::BoundedBackgroundJobExecutor>();
		}
		init_rendering();
		set_process(true);

	} else if (p_what == NOTIFICATION_PROCESS) {
#ifdef DEBUG_ENABLED
		if (!get_global_transform().is_equal_approx(godot::Transform3D())) {
			static bool reported_xform_err = false;
			if (!reported_xform_err) {
				std::cerr << "[MultinetBCCMNode3D] WARNING: Non-identity node transforms are unsupported." << std::endl;
				reported_xform_err = true;
			}
		}
#endif

		if (Engine::get_singleton()->is_editor_hint()) {
			// Apply one deferred recipe rebuild if flagged.
			if (recipe_rebuild_pending) {
				apply_recipe_rebuild();
			}

			// Initial source build on first process (if not already done).
			if (source_dirty || !render_source) {
				rebuild_source();
			}

			// Initialize renderer once per world (unified, single call).
			if (!bccm_renderer.initialized() && get_world_3d().is_valid() && manifest_built && render_source) {
				auto* rs = godot::RenderingServer::get_singleton();
				Multinet::TerrainFallbackBounds fb = render_source->get_snapshot().fallback_bounds;
				if (!configure_bccm_profile() || !bccm_renderer.initialize(rs, get_world_3d()->get_scenario(), world_domain_manifest, recipe.identity, fb)) {
					std::cerr << "[MultinetBCCMNode3D] Renderer initialization failed (process)." << std::endl;
				} else {
		bccm_renderer.bind_material_uniforms(recipe, world_domain_manifest);
				}
			}

#ifdef DEBUG_ENABLED
			// Editor path: the viewport camera is a local displacement source. The
			// persistent observer state owns canonical face/u/v and frame identity.
			if (manifest_built && editor_view_snapshot.valid && editor_observer_state.valid) {
				set_canonical_camera_state(
					editor_observer_state.canonical_position,
					editor_observer_state.active_frame,
					editor_observer_state.frame_epoch
				);
				// Editor world X/Z is the stable flat presentation plane. Ground
				// Y must stay canonical; otherwise the terrain follows the camera
				// vertically and altitude becomes visually meaningless.
				current_cam_state.presentation_x_m = editor_view_snapshot.world_position.x;
				current_cam_state.presentation_z_m = editor_view_snapshot.world_position.z;
				current_cam_state.unfolding_generation = editor_observer_state.presentation_generation;
				current_cam_state.has_presentation_position = true;
				current_cam_state.unfolding_root_frame = editor_observer_state.unfolding_root_frame;
				current_cam_state.unfolding_root_presentation_x_m = editor_observer_state.unfolding_root_presentation_x_m;
				current_cam_state.unfolding_root_presentation_z_m = editor_observer_state.unfolding_root_presentation_z_m;
				current_cam_state.has_unfolding_root = true;
				current_cam_state.logical_chart_root_direction = editor_observer_state.logical_chart_root_direction;
				current_cam_state.logical_chart_presentation_x_tangent = editor_observer_state.logical_chart_presentation_x_tangent;
				current_cam_state.logical_chart_presentation_z_tangent = editor_observer_state.logical_chart_presentation_z_tangent;
				current_cam_state.has_logical_chart = editor_observer_state.has_logical_chart;
				current_cam_state.has_presentation_binding = true;
				current_cam_state.presentation_basis = godot::Basis();
				current_cam_state.presentation_origin = editor_observer_state.presentation_origin_world;

				const double cur_cont_x = static_cast<double>(editor_view_snapshot.world_position.x) + editor_presentation_rebase_offset_x_m;
				const double cur_cont_y = static_cast<double>(editor_view_snapshot.world_position.y);
				const double cur_cont_z = static_cast<double>(editor_view_snapshot.world_position.z) + editor_presentation_rebase_offset_z_m;
				continuous_camera_m_ = godot::Vector3(static_cast<float>(cur_cont_x), static_cast<float>(cur_cont_y), static_cast<float>(cur_cont_z));

				if (bccm_renderer.initialized() && render_source && current_cam_state.frame_epoch > 0) {
					const double delta_seconds = get_process_delta_time();
					if (freeze_update) {
						const godot::Vector3 camera_delta = continuous_camera_m_ - previous_continuous_camera_m_;
						last_continuous_camera_delta_m_ = camera_delta;
						previous_continuous_camera_m_ = continuous_camera_m_;
						const godot::Vector3 freeze_camera_offset = continuous_camera_m_ - frozen_continuous_camera_anchor_m_;
						bccm_renderer.set_parent_morph_view_offset(godot::Vector2(freeze_camera_offset.x, freeze_camera_offset.z));
						bccm_renderer.update_frozen_view_presentation_delta(camera_delta, &current_chp_view, delta_seconds);
					} else {
						previous_continuous_camera_m_ = continuous_camera_m_;
						last_continuous_camera_delta_m_ = godot::Vector3(0.0f, 0.0f, 0.0f);
						bccm_renderer.update_with_view(
							editor_view_snapshot.world_position,
							editor_view_snapshot.frustum,
							manifest,
							current_cam_state,
							source_expectation,
							render_source.get(),
							&current_chp_view,
							delta_seconds
						);
					}
				}
			}
#endif
		} else {
			// Runtime path — strict camera_target resolution
			Camera3D* cam = nullptr;
			if (!camera_target.is_empty()) {
				Node* target = get_node_or_null(camera_target);
				cam = Object::cast_to<Camera3D>(target);
				if (!cam) {
					static bool reported_cam_err = false;
					if (!reported_cam_err) {
						std::cerr << "[MultinetBCCMNode3D] ERROR: camera_target is set but does not resolve to a valid Camera3D node. Skipping update." << std::endl;
						reported_cam_err = true;
					}
					return;
				}
			} else if (get_viewport()) {
				cam = get_viewport()->get_camera_3d();
			}

			if (cam) {
				const godot::Vector3 camera_forward = -cam->get_global_transform().basis.get_column(2);
				const double heading_length = std::sqrt(
					static_cast<double>(camera_forward.x) * camera_forward.x +
					static_cast<double>(camera_forward.z) * camera_forward.z);
				if (std::isfinite(heading_length) && heading_length > 1e-6) {
					camera_forward_world = camera_forward;
					has_camera_forward_world = true;
				}
				if (recipe_rebuild_pending) apply_recipe_rebuild();
				if (source_dirty || !render_source) rebuild_source();
				if (!bccm_renderer.initialized() && get_world_3d().is_valid() && manifest_built && render_source) {
					auto* rs = godot::RenderingServer::get_singleton();
					Multinet::TerrainFallbackBounds fb = render_source->get_snapshot().fallback_bounds;
					if (!configure_bccm_profile() || !bccm_renderer.initialize(rs, get_world_3d()->get_scenario(), world_domain_manifest, recipe.identity, fb)) {
						std::cerr << "[MultinetBCCMNode3D] Renderer initialization failed." << std::endl;
					} else {
						bccm_renderer.bind_material_uniforms(recipe, world_domain_manifest);
					}
				}

				if (bccm_renderer.initialized() && render_source && current_cam_state.frame_epoch > 0) {
					const double delta_seconds = get_process_delta_time();
					current_cam_state.has_presentation_binding = current_cam_state.has_presentation_position;
					current_cam_state.presentation_origin = godot::Vector3(0.0f, 0.0f, 0.0f);
					current_cam_state.presentation_basis = godot::Basis();
					const godot::Vector3 cur_pos = cam->get_global_position();
					continuous_camera_m_ = cur_pos;
					if (freeze_update) {
						const godot::Vector3 camera_delta = continuous_camera_m_ - previous_continuous_camera_m_;
						last_continuous_camera_delta_m_ = camera_delta;
						previous_continuous_camera_m_ = continuous_camera_m_;
						const godot::Vector3 freeze_camera_offset = continuous_camera_m_ - frozen_continuous_camera_anchor_m_;
						bccm_renderer.set_parent_morph_view_offset(godot::Vector2(freeze_camera_offset.x, freeze_camera_offset.z));
						bccm_renderer.update_frozen_view_presentation_delta(camera_delta, &current_chp_view, delta_seconds);
					} else {
						previous_continuous_camera_m_ = continuous_camera_m_;
						last_continuous_camera_delta_m_ = godot::Vector3(0.0f, 0.0f, 0.0f);
						bccm_renderer.update(cam, manifest, current_cam_state, source_expectation, render_source.get(), &current_chp_view, delta_seconds);
					}
				}
			}
		}


	} else if (p_what == NOTIFICATION_EXIT_TREE) {
		free_rendering();
	}
}

void MultinetBCCMNode3D::init_rendering() {
	if (bccm_renderer.initialized()) return;
	if (!manifest_built) rebuild_source();
	if (!manifest_built || !render_source) return;

	if (get_world_3d().is_valid()) {
		auto* rs = godot::RenderingServer::get_singleton();
		Multinet::TerrainFallbackBounds fb = render_source->get_snapshot().fallback_bounds;
		if (!configure_bccm_profile() || !bccm_renderer.initialize(rs, get_world_3d()->get_scenario(), world_domain_manifest, recipe.identity, fb)) {
			std::cerr << "[MultinetBCCMNode3D] Renderer initialization failed (init_rendering)." << std::endl;
		} else {
			bccm_renderer.bind_material_uniforms(recipe, world_domain_manifest);
		}
	}
	refresh_chp_contract();
}

void MultinetBCCMNode3D::free_rendering() {
	bccm_renderer.cleanup();

	// Shut down the source before the executor, so in-flight jobs can finish.
#ifdef DEBUG_ENABLED
	debug_release_held_delta_generation();
#endif
	if (render_source) {
		render_source->shutdown();
		render_source.reset();
	}

	// Shut down executor (joins worker thread).
	if (executor) {
		executor->shutdown();
		executor.reset();
	}
}

// ---------------------------------------------------------------------------
// Recipe property setters / getters
// ---------------------------------------------------------------------------

void MultinetBCCMNode3D::set_source_mode(int p_mode) {
	uint8_t m = static_cast<uint8_t>(p_mode < 0 ? 0 : (p_mode > 2 ? 2 : p_mode));
	auto mode = static_cast<multinet::rendering::TerrainSourceMode>(m);
	if (render_source) {
		if (mode == multinet::rendering::TerrainSourceMode::HybridAdditiveDelta) {
			render_source->set_payload_kind(Multinet::TerrainPagePayloadKind::AdditiveHeightDeltaV1);
		} else if (mode == multinet::rendering::TerrainSourceMode::AbsoluteHeightPageDebug) {
			render_source->set_payload_kind(Multinet::TerrainPagePayloadKind::AbsoluteHeightDebugV1);
		} else if (mode == multinet::rendering::TerrainSourceMode::AnalyticBase) {
			render_source->cancel_all_page_work_and_advance_epoch();
		}
	}
	refresh_source_expectation_from_source();
	bccm_renderer.set_source_mode(mode);
}

int MultinetBCCMNode3D::get_source_mode() const {
	return static_cast<int>(bccm_renderer.get_source_mode());
}

void MultinetBCCMNode3D::set_analytic_debug_prewarm_pages(bool p_prewarm) {
	bccm_renderer.set_analytic_debug_prewarm_pages(p_prewarm);
}

bool MultinetBCCMNode3D::get_analytic_debug_prewarm_pages() const {
	return bccm_renderer.get_analytic_debug_prewarm_pages();
}

void MultinetBCCMNode3D::set_face_colors_enabled(bool p_enabled) {
	if (face_colors_enabled == p_enabled) return;
	face_colors_enabled = p_enabled;
	bccm_renderer.set_face_colors_enabled(face_colors_enabled);
}

bool MultinetBCCMNode3D::get_face_colors_enabled() const {
	return face_colors_enabled;
}

void MultinetBCCMNode3D::set_diamond_triangulation_enabled(bool p_enabled) {
	if (diamond_triangulation_enabled == p_enabled) return;
	diamond_triangulation_enabled = p_enabled;
	bccm_renderer.set_diamond_triangulation_enabled(diamond_triangulation_enabled);
}

bool MultinetBCCMNode3D::get_diamond_triangulation_enabled() const {
	return diamond_triangulation_enabled;
}

void MultinetBCCMNode3D::set_seed(uint32_t p_seed) {
	if (recipe.identity.world_seed == p_seed) return;
	recipe.identity.world_seed = p_seed;
	request_recipe_rebuild();
}
uint32_t MultinetBCCMNode3D::get_seed() const {
	return recipe.identity.world_seed;
}

// "frequency" is the serialized name in test.tscn — maps to continental_frequency.
void MultinetBCCMNode3D::set_frequency(float p_freq) {
	if (recipe.legacy_signals.continental_frequency == p_freq) return;
	recipe.legacy_signals.continental_frequency = p_freq;
	request_recipe_rebuild();
}
float MultinetBCCMNode3D::get_frequency() const {
	return recipe.legacy_signals.continental_frequency;
}

void MultinetBCCMNode3D::set_regional_frequency(float p_freq) {
	if (recipe.legacy_signals.regional_frequency == p_freq) return;
	recipe.legacy_signals.regional_frequency = p_freq;
	request_recipe_rebuild();
}
float MultinetBCCMNode3D::get_regional_frequency() const {
	return recipe.legacy_signals.regional_frequency;
}

void MultinetBCCMNode3D::set_detail_frequency(float p_freq) {
	if (recipe.legacy_signals.detail_frequency == p_freq) return;
	recipe.legacy_signals.detail_frequency = p_freq;
	request_recipe_rebuild();
}
float MultinetBCCMNode3D::get_detail_frequency() const {
	return recipe.legacy_signals.detail_frequency;
}

void MultinetBCCMNode3D::set_min_elevation_m(float p_elev) {
	if (recipe.legacy_signals.min_elevation_m == p_elev) return;
	recipe.legacy_signals.min_elevation_m = p_elev;
	request_recipe_rebuild();
}
float MultinetBCCMNode3D::get_min_elevation_m() const {
	return recipe.legacy_signals.min_elevation_m;
}

void MultinetBCCMNode3D::set_max_elevation_m(float p_elev) {
	if (recipe.legacy_signals.max_elevation_m == p_elev) return;
	recipe.legacy_signals.max_elevation_m = p_elev;
	request_recipe_rebuild();
}
float MultinetBCCMNode3D::get_max_elevation_m() const {
	return recipe.legacy_signals.max_elevation_m;
}

void MultinetBCCMNode3D::set_octave_count(int p_count) {
	uint8_t clamped = static_cast<uint8_t>(p_count < 1 ? 1 : (p_count > 12 ? 12 : p_count));
	if (recipe.legacy_signals.octave_count == clamped) return;
	recipe.legacy_signals.octave_count = clamped;
	request_recipe_rebuild();
}
int MultinetBCCMNode3D::get_octave_count() const {
	return static_cast<int>(recipe.legacy_signals.octave_count);
}

void MultinetBCCMNode3D::set_persistence(float p_val) {
	if (recipe.legacy_signals.persistence == p_val) return;
	recipe.legacy_signals.persistence = p_val;
	request_recipe_rebuild();
}
float MultinetBCCMNode3D::get_persistence() const {
	return recipe.legacy_signals.persistence;
}

void MultinetBCCMNode3D::set_lacunarity(float p_val) {
	if (recipe.legacy_signals.lacunarity == p_val) return;
	recipe.legacy_signals.lacunarity = p_val;
	request_recipe_rebuild();
}
float MultinetBCCMNode3D::get_lacunarity() const {
	return recipe.legacy_signals.lacunarity;
}

void MultinetBCCMNode3D::set_coordinate_wrapping(bool p_enabled) {
	const auto requested = p_enabled ? Multinet::WorldDomainTopology::ClosedSurfaceSixFace : Multinet::WorldDomainTopology::FiniteRectangle;
	if (world_domain_input.topology == requested) return;
	godot::String conversion_message = "OK";
	if (p_enabled) {
		const Multinet::WorldExtentConversionResult conversion = Multinet::square_extent_preserving_area(
			world_domain_input.finite.extent_x_m, world_domain_input.finite.extent_z_m);
		if (!conversion.valid || conversion.extent_x_m < MIN_RENDERABLE_CLOSED_SIDE_M) {
			domain_validation_message = "Closed wrapping needs an equivalent side of at least 0.079 km for the ordinary 32 m LOD0 block.";
			return;
		}
		finite_aspect_history_valid = true;
		finite_aspect_history_square_world = square_world;
		finite_aspect_history_x_m = world_domain_input.finite.extent_x_m;
		finite_aspect_history_z_m = world_domain_input.finite.extent_z_m;
		world_domain_input.closed_surface.area_equivalent_side_m = conversion.extent_x_m;
		if (conversion.area_delta_m2 != 0.0L) {
			conversion_message = "Closed conversion quantized area by " + godot::String::num(static_cast<double>(conversion.area_delta_m2), 3) + " m2.";
		}
	} else {
		const uint64_t side = world_domain_input.closed_surface.area_equivalent_side_m;
		if (side == 0) return;
		const Multinet::WorldExtentConversionResult conversion = Multinet::finite_extent_from_closed_side(
			side, finite_aspect_history_x_m, finite_aspect_history_z_m, finite_aspect_history_valid);
		if (!conversion.valid) return;
		world_domain_input.finite.extent_x_m = conversion.extent_x_m;
		world_domain_input.finite.extent_z_m = conversion.extent_z_m;
		square_world = finite_aspect_history_valid ? finite_aspect_history_square_world : true;
		if (conversion.area_delta_m2 != 0.0L) {
			conversion_message = "Finite restoration quantized area by " + godot::String::num(static_cast<double>(conversion.area_delta_m2), 3) + " m2.";
		}
	}
	world_domain_input.topology = requested;
	domain_rebuild_pending = true;
	recipe_rebuild_pending = true;
	source_dirty = true;
	domain_validation_message = conversion_message;
	notify_property_list_changed();
}

bool MultinetBCCMNode3D::get_coordinate_wrapping() const {
	return world_domain_input.topology == Multinet::WorldDomainTopology::ClosedSurfaceSixFace;
}

void MultinetBCCMNode3D::set_square_world(bool p_enabled) {
	if (square_world == p_enabled) return;
	if (p_enabled && !get_coordinate_wrapping()) {
		const Multinet::WorldExtentConversionResult conversion = Multinet::square_extent_preserving_area(
			world_domain_input.finite.extent_x_m, world_domain_input.finite.extent_z_m);
		if (!conversion.valid) {
			domain_validation_message = "Square World conversion requires valid finite dimensions.";
			return;
		}
		world_domain_input.finite.extent_x_m = conversion.extent_x_m;
		world_domain_input.finite.extent_z_m = conversion.extent_z_m;
		if (conversion.area_delta_m2 != 0.0L) {
			domain_validation_message = "Square World quantized area by " + godot::String::num(static_cast<double>(conversion.area_delta_m2), 3) + " m2.";
		}
	}
	square_world = p_enabled;
	domain_rebuild_pending = true;
	recipe_rebuild_pending = true;
	source_dirty = true;
	notify_property_list_changed();
}

bool MultinetBCCMNode3D::get_square_world() const { return square_world; }

void MultinetBCCMNode3D::set_world_side_km(double p_km) {
	uint64_t metres = 0;
	if (!try_km_to_world_metres(p_km, metres)) {
		domain_validation_message = "World scale must be within 0.001 km and 4,294,967.295 km (UINT32_MAX metres).";
		return;
	}
	if (get_coordinate_wrapping()) {
		if (metres < MIN_RENDERABLE_CLOSED_SIDE_M) {
			domain_validation_message = "Closed wrapping needs an equivalent side of at least 0.079 km for the ordinary 32 m LOD0 block.";
			return;
		}
		world_domain_input.closed_surface.area_equivalent_side_m = metres;
	} else {
		world_domain_input.finite.extent_x_m = metres;
		if (square_world) world_domain_input.finite.extent_z_m = metres;
	}
	domain_rebuild_pending = true;
	recipe_rebuild_pending = true;
	source_dirty = true;
	domain_validation_message = "OK";
}

double MultinetBCCMNode3D::get_world_side_km() const {
	return get_coordinate_wrapping()
		? static_cast<double>(world_domain_input.closed_surface.area_equivalent_side_m) / 1000.0
		: static_cast<double>(world_domain_input.finite.extent_x_m) / 1000.0;
}

void MultinetBCCMNode3D::set_world_extent_x_km(double p_km) {
	uint64_t metres = 0;
	if (!try_km_to_world_metres(p_km, metres)) {
		domain_validation_message = "World extent X must be within 0.001 km and 4,294,967.295 km.";
		return;
	}
	world_domain_input.finite.extent_x_m = metres;
	if (square_world) world_domain_input.finite.extent_z_m = metres;
	domain_rebuild_pending = true;
	recipe_rebuild_pending = true;
	source_dirty = true;
	domain_validation_message = "OK";
}

double MultinetBCCMNode3D::get_world_extent_x_km() const { return static_cast<double>(world_domain_input.finite.extent_x_m) / 1000.0; }

void MultinetBCCMNode3D::set_world_extent_z_km(double p_km) {
	uint64_t metres = 0;
	if (!try_km_to_world_metres(p_km, metres)) {
		domain_validation_message = "World extent Z must be within 0.001 km and 4,294,967.295 km.";
		return;
	}
	world_domain_input.finite.extent_z_m = metres;
	if (square_world) world_domain_input.finite.extent_x_m = metres;
	domain_rebuild_pending = true;
	recipe_rebuild_pending = true;
	source_dirty = true;
	domain_validation_message = "OK";
}

double MultinetBCCMNode3D::get_world_extent_z_km() const { return static_cast<double>(world_domain_input.finite.extent_z_m) / 1000.0; }

void MultinetBCCMNode3D::set_closed_equivalent_side_km(double p_km) {
	uint64_t metres = 0;
	if (!try_km_to_world_metres(p_km, metres)) {
		domain_validation_message = "Closed equivalent side must be within 0.079 km and 4,294,967.295 km.";
		return;
	}
	if (metres < MIN_RENDERABLE_CLOSED_SIDE_M) {
		domain_validation_message = "Closed wrapping needs an equivalent side of at least 0.079 km for the ordinary 32 m LOD0 block.";
		return;
	}
	world_domain_input.closed_surface.area_equivalent_side_m = metres;
	domain_rebuild_pending = true;
	recipe_rebuild_pending = true;
	source_dirty = true;
	domain_validation_message = "OK";
}

double MultinetBCCMNode3D::get_closed_equivalent_side_km() const { return static_cast<double>(world_domain_input.closed_surface.area_equivalent_side_m) / 1000.0; }

void MultinetBCCMNode3D::set_closed_flat_coverage_radius_blocks(int p_radius) {
	const int maximum = multinet::rendering::BlockClipmapProfile::MAX_SUPPORTED_CANDIDATE_GRID_RADIUS;
	const int clamped = std::clamp(p_radius, 1, maximum);
	if (closed_flat_coverage_radius_blocks == clamped) return;
	closed_flat_coverage_radius_blocks = clamped;
	request_recipe_rebuild();
}

int MultinetBCCMNode3D::get_closed_flat_coverage_radius_blocks() const {
	return closed_flat_coverage_radius_blocks;
}

double MultinetBCCMNode3D::get_closed_flat_visible_extent_km() const {
	return bccm_renderer.get_effective_coverage_extent_m() / 1000.0;
}

void MultinetBCCMNode3D::set_chp_enabled(bool p_enabled) {
	if (world_presentation_input.chp_enabled == p_enabled) return;
	world_presentation_input.chp_enabled = p_enabled;
	presentation_rebuild_pending = true;
	world_presentation_manifest = Multinet::build_world_presentation_manifest(world_domain_manifest, world_presentation_input);
	refresh_chp_contract();
	notify_property_list_changed();
}

bool MultinetBCCMNode3D::get_chp_enabled() const { return world_presentation_input.chp_enabled; }

void MultinetBCCMNode3D::set_chp_radius_policy(int p_policy) {
	const int clamped = std::clamp(p_policy, 0, 2);
	world_presentation_input.chp_radius_policy = static_cast<Multinet::CHPRadiusPolicy>(clamped);
	presentation_rebuild_pending = true;
	world_presentation_manifest = Multinet::build_world_presentation_manifest(world_domain_manifest, world_presentation_input);
	refresh_chp_contract();
	notify_property_list_changed();
}

int MultinetBCCMNode3D::get_chp_radius_policy() const { return static_cast<int>(world_presentation_input.chp_radius_policy); }

void MultinetBCCMNode3D::set_chp_explicit_radius_km(double p_km) {
	uint64_t metres = 0;
	if (!try_km_to_world_metres(p_km, metres)) {
		domain_validation_message = "CHP explicit radius must be within 0.001 km and 4,294,967.295 km.";
		return;
	}
	world_presentation_input.explicit_chp_radius_mm = metres * 1000ULL;
	presentation_rebuild_pending = true;
	world_presentation_manifest = Multinet::build_world_presentation_manifest(world_domain_manifest, world_presentation_input);
	refresh_chp_contract();
	notify_property_list_changed();
}

double MultinetBCCMNode3D::get_chp_explicit_radius_km() const { return static_cast<double>(world_presentation_input.explicit_chp_radius_mm) / 1000000.0; }

double MultinetBCCMNode3D::get_canonical_area_km2() const { return static_cast<double>(world_domain_manifest.canonical_area_m2) / 1000000.0; }

double MultinetBCCMNode3D::get_logical_radius_km() const {
	return world_domain_manifest.is_valid()
		? std::sqrt(static_cast<double>(world_domain_manifest.canonical_area_m2) / (4.0 * 3.14159265358979323846)) / 1000.0
		: 0.0;
}

double MultinetBCCMNode3D::get_closed_face_extent_km() const {
	return !world_domain_manifest.is_finite() && world_domain_manifest.is_valid()
		? world_domain_manifest.closed_surface.area_equivalent_face_extent_m / 1000.0
		: 0.0;
}

double MultinetBCCMNode3D::get_closed_face_half_extent_km() const {
	return !world_domain_manifest.is_finite() && world_domain_manifest.is_valid()
		? static_cast<double>(world_domain_manifest.closed_surface.chart_half_extent_mm) * 0.001 / 1000.0
		: 0.0;
}

uint32_t MultinetBCCMNode3D::get_regions_per_face_axis() const {
	return !world_domain_manifest.is_finite() && world_domain_manifest.is_valid()
		? world_domain_manifest.closed_surface.regions_per_face_axis
		: 0;
}

double MultinetBCCMNode3D::get_actual_region_extent_m() const {
	return !world_domain_manifest.is_finite() && world_domain_manifest.is_valid()
		? world_domain_manifest.closed_surface.actual_region_extent_m
		: 0.0;
}

uint64_t MultinetBCCMNode3D::get_active_domain_hash() const { return world_domain_manifest.domain_manifest_hash; }

uint64_t MultinetBCCMNode3D::get_active_presentation_hash() const { return world_presentation_manifest.presentation_manifest_hash; }

godot::String MultinetBCCMNode3D::get_active_domain_hash_text() const {
	std::ostringstream stream;
	stream << "0x" << std::uppercase << std::hex << std::setw(16) << std::setfill('0') << get_active_domain_hash();
	return godot::String(stream.str().c_str());
}

godot::String MultinetBCCMNode3D::get_active_presentation_hash_text() const {
	std::ostringstream stream;
	stream << "0x" << std::uppercase << std::hex << std::setw(16) << std::setfill('0') << get_active_presentation_hash();
	return godot::String(stream.str().c_str());
}

godot::String MultinetBCCMNode3D::get_domain_validation_message() const {
	return domain_validation_message;
}

godot::String MultinetBCCMNode3D::get_chp_status() const {
	if (!world_presentation_manifest.is_valid()) return "CPU CONTRACT INVALID / GPU INTEGRATION NOT STARTED";
	if (world_presentation_input.chp_enabled && !resolved_chp_profile.is_valid()) {
		return "CPU CONTRACT INVALID / GPU INTEGRATION NOT STARTED";
	}
	if (!current_chp_view.is_valid()) return "CPU PROFILE READY / CAMERA VIEW NOT READY / GPU INTEGRATION NOT STARTED";
	if (world_presentation_input.chp_enabled && current_chp_view.chp_effective) {
		if (bccm_renderer.get_chp_debug_reconstruction_mode() == 1) {
			return "R1 GPU IDENTITY RECONSTRUCTION (Mode 1) / CULLING CONSERVATIVE";
		}
		return "R1 GPU POSITION ACTIVE (SphericalPolynomial6) / CULLING CONSERVATIVE";
	}
	return "CPU VIEW READY / GPU POSITION OFF";
}

void MultinetBCCMNode3D::set_chp_debug_reconstruction_mode(int p_mode) {
	bccm_renderer.set_chp_debug_reconstruction_mode(p_mode);
}

int MultinetBCCMNode3D::get_chp_debug_reconstruction_mode() const {
	return bccm_renderer.get_chp_debug_reconstruction_mode();
}

void MultinetBCCMNode3D::set_chp_debug_negative_height_color(bool p_enabled) {
	bccm_renderer.set_chp_debug_negative_height_color(p_enabled);
}

bool MultinetBCCMNode3D::get_chp_debug_negative_height_color() const {
	return bccm_renderer.get_chp_debug_negative_height_color();
}

void MultinetBCCMNode3D::set_chp_debug_negative_height_exaggeration(bool p_enabled) {
	bccm_renderer.set_chp_debug_negative_height_exaggeration(p_enabled);
}

bool MultinetBCCMNode3D::get_chp_debug_negative_height_exaggeration() const {
	return bccm_renderer.get_chp_debug_negative_height_exaggeration();
}

void MultinetBCCMNode3D::set_bccm_debug_visual_mode(int p_mode) {
	bccm_renderer.set_bccm_debug_visual_mode(p_mode);
}

int MultinetBCCMNode3D::get_bccm_debug_visual_mode() const {
	return bccm_renderer.get_bccm_debug_visual_mode();
}

void MultinetBCCMNode3D::set_freeze_update(bool p_freeze) {
	if (p_freeze && !freeze_update) {
		frozen_bccm_frame = current_cam_state.frame_epoch;
		frozen_root_epoch = current_cam_state.unfolding_root_frame.frame_epoch;
		frozen_chp_profile_version = chp_profile.profile_version;

#ifdef DEBUG_ENABLED
		const double cur_cont_x = static_cast<double>(editor_view_snapshot.world_position.x) + editor_presentation_rebase_offset_x_m;
		const double cur_cont_y = static_cast<double>(editor_view_snapshot.world_position.y);
		const double cur_cont_z = static_cast<double>(editor_view_snapshot.world_position.z) + editor_presentation_rebase_offset_z_m;
		continuous_camera_m_ = godot::Vector3(static_cast<float>(cur_cont_x), static_cast<float>(cur_cont_y), static_cast<float>(cur_cont_z));
#endif
		previous_continuous_camera_m_ = continuous_camera_m_;
		frozen_continuous_camera_anchor_m_ = continuous_camera_m_;
		last_continuous_camera_delta_m_ = godot::Vector3(0.0f, 0.0f, 0.0f);
		bccm_renderer.set_parent_morph_view_offset(godot::Vector2(0.0f, 0.0f));
		frozen_frustum_snapshot_id_++;
		frozen_frustum_active_ = true;
	} else if (!p_freeze && freeze_update) {
		frozen_frustum_active_ = false;
		bccm_renderer.set_parent_morph_view_offset(godot::Vector2(0.0f, 0.0f));
		bccm_renderer.clear_frozen_frustum_visualization();
	}
	freeze_update = p_freeze;
}

void MultinetBCCMNode3D::set_high_speed_cut_diagnostics_enabled(bool p_enabled) {
	high_speed_cut_diagnostics_enabled = p_enabled;
	bccm_renderer.set_high_speed_cut_diagnostics_enabled(p_enabled);
}

bool MultinetBCCMNode3D::get_high_speed_cut_diagnostics_enabled() const {
	return high_speed_cut_diagnostics_enabled;
}

void MultinetBCCMNode3D::set_camera_target(const godot::NodePath& p_path) { camera_target = p_path; }

godot::NodePath MultinetBCCMNode3D::get_camera_target() const { return camera_target; }

// ---------------------------------------------------------------------------
// Diagnostic accessors
// ---------------------------------------------------------------------------

uint32_t MultinetBCCMNode3D::get_candidate_count(int p_lod) const {
	return bccm_renderer.get_candidate_count(static_cast<uint8_t>(p_lod));
}

uint32_t MultinetBCCMNode3D::get_visible_count(int p_lod) const {
	return bccm_renderer.get_visible_count(static_cast<uint8_t>(p_lod));
}

uint32_t MultinetBCCMNode3D::get_submitted_streams() const {
	return bccm_renderer.get_submitted_streams();
}

godot::Dictionary MultinetBCCMNode3D::get_debug_summary() const {
	godot::Dictionary dict;

	dict["initialized"] = bccm_renderer.initialized();
	dict["valid_publication"] = (current_cam_state.frame_epoch > 0);
	dict["submitted_streams"] = get_submitted_streams();
	dict["world_domain_topology"] = static_cast<int>(world_domain_input.topology);
	dict["world_domain_topology_name"] = world_domain_input.topology == Multinet::WorldDomainTopology::FiniteRectangle
		? "FiniteRectangle" : "ClosedSurfaceSixFace";
	dict["coordinate_wrapping"] = get_coordinate_wrapping();
	const bool is_chp_effective = world_presentation_input.chp_enabled && current_chp_view.chp_effective;
	dict["chp_requested"] = world_presentation_input.chp_enabled;
	dict["chp_effective"] = is_chp_effective;
	dict["chp_cpu_contract_valid"] = world_presentation_input.chp_enabled
		? resolved_chp_profile.is_valid()
		: world_presentation_manifest.is_valid();
	dict["chp_cpu_view_valid"] = current_chp_view.is_valid();
	dict["chp_gpu_effective"] = is_chp_effective;
	dict["chp_implementation_state"] = get_chp_status();
	dict["chp_radius_policy"] = static_cast<int>(world_presentation_manifest.chp_radius_policy);
	dict["resolved_chp_radius_km"] = static_cast<double>(world_presentation_manifest.resolved_chp_radius_mm) / 1000000.0;
	dict["chp_function_class"] = static_cast<int>(chp_profile.function_class);
	dict["chp_function_class_name"] = multinet::rendering::chp::get_function_class_name(chp_profile.function_class);
	dict["chp_requested_maximum_distance_km"] = chp_profile.requested_maximum_deformation_distance_m / 1000.0;
	dict["chp_certified_maximum_distance_km"] = resolved_chp_profile.certified_maximum_deformation_distance_m / 1000.0;
	dict["chp_distance_was_clamped"] = resolved_chp_profile.distance_was_clamped;
	dict["chp_position_error_at_limit_m"] = resolved_chp_profile.base_position_error_at_limit_m;
	dict["chp_visual_up_error_at_limit_rad"] = resolved_chp_profile.visual_up_error_at_limit_radians;
	dict["chp_camera_surface_altitude_m"] = current_chp_view.camera_surface_height_m;
	dict["chp_horizon_line_of_sight_km"] = current_chp_view.horizon_line_of_sight_m / 1000.0;
	dict["chp_horizon_surface_arc_km"] = current_chp_view.horizon_surface_arc_m / 1000.0;
	dict["chp_nominal_admitted_intrinsic_radius_km"] = current_chp_view.nominal_admitted_intrinsic_radius_m / 1000.0;
	dict["chp_kernel_contract_version"] = world_presentation_manifest.chp_kernel_version;
	dict["chp_profile_version"] = chp_profile.profile_version;
	dict["canonical_area_km2"] = get_canonical_area_km2();
	dict["logical_radius_km"] = get_logical_radius_km();
	dict["closed_face_extent_km"] = get_closed_face_extent_km();
	dict["closed_face_half_extent_km"] = get_closed_face_half_extent_km();
	dict["regions_per_face_axis"] = get_regions_per_face_axis();
	dict["actual_region_extent_m"] = get_actual_region_extent_m();
	dict["effective_bccm_lod_count"] = bccm_renderer.get_effective_level_count();
	dict["effective_bccm_lod_max"] = bccm_renderer.get_effective_level_count() > 0 ? bccm_renderer.get_effective_level_count() - 1 : 0;
	dict["domain_validation_message"] = domain_validation_message;
	dict["domain_topology_version"] = world_domain_manifest.topology_version;
	dict["domain_projection_version"] = world_domain_manifest.projection_version;
	dict["bccm_analytic_normal_version"] = Multinet::BCCM_ANALYTIC_NORMAL_VERSION_3;
	dict["domain_manifest_hash"] = static_cast<int64_t>(world_domain_manifest.domain_manifest_hash);
	dict["presentation_manifest_hash"] = static_cast<int64_t>(world_presentation_manifest.presentation_manifest_hash);
	dict["domain_manifest_hash_text"] = get_active_domain_hash_text();
	dict["presentation_manifest_hash_text"] = get_active_presentation_hash_text();
	dict["canonical_observer_face"] = static_cast<int>(current_cam_state.canonical_position.face);
	dict["canonical_observer_u_m"] = current_cam_state.canonical_position.u_m;
	dict["canonical_observer_v_m"] = current_cam_state.canonical_position.v_m;
	dict["canonical_observer_altitude_m"] = current_cam_state.canonical_position.altitude_m;
	dict["canonical_observer_frame_epoch"] = static_cast<int64_t>(current_cam_state.frame_epoch);
	dict["presentation_observer_x_m"] = current_cam_state.presentation_x_m;
	dict["presentation_observer_z_m"] = current_cam_state.presentation_z_m;
	dict["presentation_unfolding_generation"] = static_cast<int64_t>(current_cam_state.unfolding_generation);
	dict["sampling_chart_anchor_x_m"] = current_cam_state.unfolding_root_presentation_x_m;
	dict["sampling_chart_anchor_z_m"] = current_cam_state.unfolding_root_presentation_z_m;
	dict["sampling_chart_anchor_face"] = static_cast<int>(current_cam_state.unfolding_root_frame.origin.face);

	const double root_u = current_cam_state.unfolding_root_frame.origin.u_m;
	const double root_v = current_cam_state.unfolding_root_frame.origin.v_m;
	const int root_face = static_cast<int>(current_cam_state.unfolding_root_frame.origin.face);
	dict["canonical_root_face"] = root_face;
	dict["canonical_root_u_m"] = root_u;
	dict["canonical_root_v_m"] = root_v;

	const double radius_m = world_domain_manifest.closed_surface.logical_area_radius_m;
	const double half_ext_m = static_cast<double>(world_domain_manifest.closed_surface.chart_half_extent_mm) * 0.001;
	const Multinet::FramePosition64 root_dir = half_ext_m > 0.0
		? Multinet::ProjectionCOBE::map_forward(root_face, root_u / half_ext_m, root_v / half_ext_m)
		: Multinet::FramePosition64{ 1.0, 0.0, 0.0 };
	dict["logical_root_direction_x"] = root_dir.x;
	dict["logical_root_direction_y"] = root_dir.y;
	dict["logical_root_direction_z"] = root_dir.z;

	dict["root_phys_pos_double_x"] = root_dir.x * radius_m;
	dict["root_phys_pos_double_y"] = root_dir.y * radius_m;
	dict["root_phys_pos_double_z"] = root_dir.z * radius_m;

	const double root_dx = current_cam_state.presentation_x_m - current_cam_state.unfolding_root_presentation_x_m;
	const double root_dz = current_cam_state.presentation_z_m - current_cam_state.unfolding_root_presentation_z_m;
	dict["local_chart_distance_m"] = std::sqrt(root_dx * root_dx + root_dz * root_dz);
	dict["presentation_rebase_count"] = static_cast<int64_t>(current_cam_state.unfolding_generation);
	dict["presentation_local_camera_x_m"] = current_cam_state.presentation_x_m;
	dict["presentation_local_camera_z_m"] = current_cam_state.presentation_z_m;

	double freq_diag = recipe.legacy_signals.continental_frequency;
	for (uint8_t oct = 0; oct < 4; ++oct) {
		double sx = (root_dir.x * radius_m) * freq_diag;
		double sy = (root_dir.y * radius_m) * freq_diag;
		double sz = (root_dir.z * radius_m) * freq_diag;
		double fx = std::floor(sx);
		double fy = std::floor(sy);
		double fz = std::floor(sz);
		godot::String prefix = "root_lattice_oct_" + godot::String::num_int64(oct);
		dict[prefix + "_cell_x"] = static_cast<int64_t>(fx);
		dict[prefix + "_cell_y"] = static_cast<int64_t>(fy);
		dict[prefix + "_cell_z"] = static_cast<int64_t>(fz);
		dict[prefix + "_frac_x"] = sx - fx;
		dict[prefix + "_frac_y"] = sy - fy;
		dict[prefix + "_frac_z"] = sz - fz;
		freq_diag *= static_cast<double>(recipe.legacy_signals.lacunarity);
	}
	const bool is_morph_certified = (bccm_renderer.get_profile().candidate_grid_radius == 4 && bccm_renderer.get_profile().inner_hole_radius == 2);
	dict["phase_b2_morph_effective"] = is_morph_certified;
	dict["phase_b2_morph_status"] = is_morph_certified
		? godot::String("A2 / CERTIFIED r4-h2")
		: godot::String("DISABLED / UNCERTIFIED PROFILE r") + godot::String::num_int64(bccm_renderer.get_profile().candidate_grid_radius) + "-h" + godot::String::num_int64(bccm_renderer.get_profile().inner_hole_radius);
	dict["phase_b2_morph_diagnostic"] = is_morph_certified
		? godot::String("Phase-B2 Morph: A2 / CERTIFIED r4-h2")
		: godot::String("Phase-B2 Morph: DISABLED / UNCERTIFIED PROFILE r") + godot::String::num_int64(bccm_renderer.get_profile().candidate_grid_radius) + "-h" + godot::String::num_int64(bccm_renderer.get_profile().inner_hole_radius);

	dict["runtime_ground_speed_m_s"] = runtime_ground_speed_m_s;
	dict["runtime_vertical_speed_m_s"] = runtime_vertical_speed_m_s;
	dict["runtime_total_speed_m_s"] = runtime_total_speed_m_s;
	dict["runtime_last_transition_count"] = runtime_last_transition_count;
	dict["runtime_last_transition_source_face"] = runtime_last_transition_source_face;
	dict["runtime_last_transition_destination_face"] = runtime_last_transition_destination_face;
	dict["runtime_last_transition_edge"] = runtime_last_transition_edge;
	dict["runtime_outside_finite_boundary"] = runtime_outside_finite_boundary;
	dict["camera_heading_valid"] = has_camera_forward_world;
	dict["camera_heading_x"] = static_cast<double>(camera_forward_world.x);
	dict["camera_heading_z"] = static_cast<double>(camera_forward_world.z);
	const auto& streaming = bccm_renderer.get_last_streaming_diagnostics();
	dict["closed_placement_failures"] = streaming.closed_placement_failures;
	dict["canonical_duplicate_presentations_retained"] = streaming.canonical_duplicate_presentations_retained;
	dict["maximum_patch_transition_count"] = streaming.maximum_patch_transition_count;
#ifdef DEBUG_ENABLED
	dict["editor_observer_valid"] = editor_observer_state.valid;
	dict["editor_observer_face"] = static_cast<int>(editor_observer_state.canonical_position.face);
	dict["editor_observer_u_m"] = editor_observer_state.canonical_position.u_m;
	dict["editor_observer_v_m"] = editor_observer_state.canonical_position.v_m;
	dict["editor_observer_altitude_m"] = editor_observer_state.canonical_position.altitude_m;
	dict["editor_observer_frame_epoch"] = static_cast<int64_t>(editor_observer_state.frame_epoch);
	dict["editor_observer_ground_speed_m_s"] = editor_observer_state.ground_speed_m_s;
	dict["editor_observer_vertical_speed_m_s"] = editor_observer_state.vertical_speed_m_s;
	dict["editor_observer_total_speed_m_s"] = editor_observer_state.total_speed_m_s;
	dict["editor_last_transition_count"] = editor_observer_state.last_transition_count;
	dict["editor_last_transition_frame_epoch"] = static_cast<int64_t>(editor_observer_state.last_transition_frame_epoch);
	dict["editor_last_transition_initial_face"] = editor_observer_state.last_transition_initial_face;
	dict["editor_last_transition_final_face"] = editor_observer_state.last_transition_final_face;
	dict["editor_view_world_x_m"] = editor_presentation_rebase_offset_x_m +
		static_cast<double>(editor_view_snapshot.world_position.x);
	dict["editor_view_world_y_m"] = editor_view_snapshot.world_position.y;
	dict["editor_view_world_z_m"] = editor_presentation_rebase_offset_z_m +
		static_cast<double>(editor_view_snapshot.world_position.z);
	dict["editor_view_local_x_m"] = editor_view_snapshot.world_position.x;
	dict["editor_view_local_z_m"] = editor_view_snapshot.world_position.z;
	dict["editor_presentation_rebase_offset_x_m"] = editor_presentation_rebase_offset_x_m;
	dict["editor_presentation_rebase_offset_z_m"] = editor_presentation_rebase_offset_z_m;
	dict["editor_last_presentation_rebase_x_m"] = editor_last_presentation_rebase_x_m;
	dict["editor_last_presentation_rebase_z_m"] = editor_last_presentation_rebase_z_m;
	dict["editor_presentation_rebase_count"] = static_cast<int64_t>(editor_presentation_rebase_count);
	dict["editor_presentation_rebase_threshold_m"] = EDITOR_PRESENTATION_REBASE_THRESHOLD_M;
	dict["editor_presentation_anchor_x_m"] = current_cam_state.presentation_origin.x;
	dict["editor_presentation_anchor_z_m"] = current_cam_state.presentation_origin.z;
	const double editor_presentation_anchor_lag_x_m =
		static_cast<double>(editor_view_snapshot.world_position.x - current_cam_state.presentation_origin.x);
	const double editor_presentation_anchor_lag_z_m =
		static_cast<double>(editor_view_snapshot.world_position.z - current_cam_state.presentation_origin.z);
	dict["editor_presentation_anchor_lag_m"] = std::sqrt(
		editor_presentation_anchor_lag_x_m * editor_presentation_anchor_lag_x_m +
		editor_presentation_anchor_lag_z_m * editor_presentation_anchor_lag_z_m);
	dict["editor_rejected_chart_motion_count"] = static_cast<int64_t>(editor_observer_state.rejected_chart_motion_count);
	dict["editor_rejected_frame_advance_count"] = static_cast<int64_t>(editor_observer_state.rejected_frame_advance_count);
	dict["editor_ground_origin_y_m"] = editor_view_snapshot.world_position.y -
		static_cast<float>(editor_observer_state.canonical_position.altitude_m);
	const float expected_binding_origin_y = world_domain_manifest.is_finite()
		? static_cast<float>(editor_observer_state.canonical_position.altitude_m)
		: editor_view_snapshot.world_position.y -
			static_cast<float>(editor_observer_state.canonical_position.altitude_m);
	dict["editor_presentation_origin_y_m"] = current_cam_state.presentation_origin.y;
	dict["editor_altitude_tracking_error_m"] =
		expected_binding_origin_y - current_cam_state.presentation_origin.y;
	dict["editor_last_transition_source_face"] = editor_observer_state.last_transition_source_face;
	dict["editor_last_transition_destination_face"] = editor_observer_state.last_transition_destination_face;
	dict["editor_last_transition_edge"] = editor_observer_state.last_transition_edge;
	dict["editor_outside_viewport_camera"] = editor_observer_state.outside_viewport_camera;
#endif
	const double observer_u_m = current_cam_state.canonical_position.u_m;
	const double observer_v_m = current_cam_state.canonical_position.v_m;
	if (world_domain_manifest.is_finite()) {
		dict["finite_extent_x_km"] = get_world_extent_x_km();
		dict["finite_extent_z_km"] = get_world_extent_z_km();
		dict["finite_regions_x"] = world_domain_manifest.finite.regions_x;
		dict["finite_regions_z"] = world_domain_manifest.finite.regions_z;
		dict["finite_actual_region_extent_x_m"] = world_domain_manifest.finite.actual_region_extent_x_m;
		dict["finite_actual_region_extent_z_m"] = world_domain_manifest.finite.actual_region_extent_z_m;
		const double hx = static_cast<double>(world_domain_manifest.finite.half_extent_x_mm) * 0.001;
		const double hz = static_cast<double>(world_domain_manifest.finite.half_extent_z_mm) * 0.001;
		dict["finite_distance_to_x_minus_m"] = observer_u_m + hx;
		dict["finite_distance_to_x_plus_m"] = hx - observer_u_m;
		dict["finite_distance_to_z_minus_m"] = observer_v_m + hz;
		dict["finite_distance_to_z_plus_m"] = hz - observer_v_m;
	} else {
		dict["closed_equivalent_side_km"] = get_closed_equivalent_side_km();
		const double chart_max_radius_m = Multinet::closed_flat_chart_max_radius_m(world_domain_manifest);
		const double chart_dx_m = current_cam_state.presentation_x_m - current_cam_state.unfolding_root_presentation_x_m;
		const double chart_dz_m = current_cam_state.presentation_z_m - current_cam_state.unfolding_root_presentation_z_m;
		const double chart_observer_offset_m = std::sqrt(chart_dx_m * chart_dx_m + chart_dz_m * chart_dz_m);
		dict["closed_flat_chart_max_radius_m"] = chart_max_radius_m;
		dict["closed_flat_chart_observer_offset_m"] = chart_observer_offset_m;
		dict["closed_flat_chart_observer_inside"] = chart_observer_offset_m <= chart_max_radius_m;
		const double chart_outer_footprint_m = bccm_renderer.get_effective_coverage_corner_radius_m();
		const double chart_worst_sample_radius_m = chart_observer_offset_m + chart_outer_footprint_m;
		dict["closed_flat_coverage_radius_blocks"] = closed_flat_coverage_radius_blocks;
		dict["closed_flat_visible_extent_km"] = get_closed_flat_visible_extent_km();
		dict["closed_flat_chart_outer_footprint_m"] = chart_outer_footprint_m;
		dict["closed_flat_chart_worst_sample_radius_m"] = chart_worst_sample_radius_m;
		dict["closed_flat_chart_footprint_inside"] = chart_worst_sample_radius_m <= chart_max_radius_m;
		const bool analytic_chart_tracking =
			bccm_renderer.get_source_mode() == multinet::rendering::TerrainSourceMode::AnalyticBase &&
			!bccm_renderer.get_analytic_debug_prewarm_pages();
		dict["closed_flat_chart_tracking"] = analytic_chart_tracking;
		dict["closed_flat_chart_tracking_reason"] = analytic_chart_tracking
			? "AnalyticBase local chart"
			: "chart-addressed pages require stable root";
		dict["closed_face_extent_km"] = world_domain_manifest.is_valid() ? world_domain_manifest.closed_surface.area_equivalent_face_extent_m / 1000.0 : 0.0;
		dict["closed_face_half_extent_km"] = world_domain_manifest.is_valid() ? world_domain_manifest.closed_surface.chart_half_extent_mm * 0.001 / 1000.0 : 0.0;
		const double h = world_domain_manifest.is_valid() ? world_domain_manifest.closed_surface.chart_half_extent_mm * 0.001 : 0.0;
		dict["closed_distance_to_negative_u_m"] = h + observer_u_m;
		dict["closed_distance_to_positive_u_m"] = h - observer_u_m;
		dict["closed_distance_to_negative_v_m"] = h + observer_v_m;
		dict["closed_distance_to_positive_v_m"] = h - observer_v_m;

		const double heading_length = std::sqrt(
			static_cast<double>(camera_forward_world.x) * camera_forward_world.x +
			static_cast<double>(camera_forward_world.z) * camera_forward_world.z);
		const bool heading_valid = has_camera_forward_world && std::isfinite(heading_length) && heading_length > 1e-6;
		dict["closed_transition_heading_valid"] = heading_valid;
		if (heading_valid) {
			const double heading_x = static_cast<double>(camera_forward_world.x) / heading_length;
			const double heading_z = static_cast<double>(camera_forward_world.z) / heading_length;
			dict["closed_transition_heading_x"] = heading_x;
			dict["closed_transition_heading_z"] = heading_z;
			const uint8_t active_face = static_cast<uint8_t>(current_cam_state.canonical_position.face);
			const auto publish_edge = [&](const char* prefix, Multinet::SurfaceEdge edge, double edge_distance_m) {
				double direction_x = 0.0;
				double direction_z = 0.0;
				const bool direction_valid = active_face < 6 && try_flat_edge_direction(
					current_cam_state.active_frame, edge, direction_x, direction_z);
				const double heading_dot = direction_valid ? heading_x * direction_x + heading_z * direction_z : 0.0;
				const double signed_distance = std::abs(heading_dot) > 0.05 ? edge_distance_m / heading_dot : 0.0;
				dict[godot::String(prefix) + "_destination_face"] = direction_valid
					? static_cast<int>(Multinet::get_edge_transition(active_face, edge).destination_face) : -1;
				dict[godot::String(prefix) + "_distance_m"] = edge_distance_m;
				dict[godot::String(prefix) + "_direction_x"] = direction_x;
				dict[godot::String(prefix) + "_direction_z"] = direction_z;
				dict[godot::String(prefix) + "_heading_dot"] = heading_dot;
				dict[godot::String(prefix) + "_signed_distance_m"] = signed_distance;
			};
			publish_edge("closed_transition_negative_u", Multinet::SurfaceEdge::NegativeU, h + observer_u_m);
			publish_edge("closed_transition_positive_u", Multinet::SurfaceEdge::PositiveU, h - observer_u_m);
			publish_edge("closed_transition_negative_v", Multinet::SurfaceEdge::NegativeV, h + observer_v_m);
			publish_edge("closed_transition_positive_v", Multinet::SurfaceEdge::PositiveV, h - observer_v_m);
		}
	}

	auto snap = std::make_unique<multinet::rendering::RendererDiagnosticSnapshot>();
	bccm_renderer.get_diagnostic_snapshot(*snap);
	const auto detailed = bccm_renderer.get_detailed_diagnostics();
	const auto source_mode = bccm_renderer.get_source_mode();
	dict["source_mode"] = static_cast<int>(source_mode);
	dict["source_mode_name"] = source_mode == multinet::rendering::TerrainSourceMode::AnalyticBase
		? "AnalyticBase"
		: (source_mode == multinet::rendering::TerrainSourceMode::HybridAdditiveDelta ? "HybridAdditiveDelta" : "AbsoluteHeightPageDebug");
	dict["committed_publication_version"] = render_source ? render_source->get_snapshot().committed_delta_version : 0;
	dict["source_pending_count"] = detailed.source_pending_count;
	dict["source_in_flight_count"] = detailed.source_in_flight_count;
	dict["resident_delta_layers"] = detailed.resident_delta_layers;
	dict["upload_pending_delta_layers"] = detailed.upload_pending_delta_layers;
	dict["stale_previous_visible_instance_count"] = detailed.stale_delta_pages_retained;
	dict["ready_empty_visible_instance_count"] = detailed.hybrid_ready_empty_instances;
	dict["analytic_visible_instance_count"] = detailed.analytic_base_visible_instances;
	dict["hybrid_visible_instance_count"] = detailed.hybrid_visible_instances;
	dict["visible_constant_fallback_instance_count"] = detailed.visible_constant_fallback_instances;
	dict["submitted_stream_count"] = get_submitted_streams();

	uint32_t active_residency = 0;
	uint32_t max_layer = 0;
	
	uint32_t face_candidate_count[6] = {0};
	uint32_t face_visible_count[6] = {0};
	uint32_t face_non_fallback_count[6] = {0};
	uint32_t fallback_slot_valid_count = 0;

	for (uint8_t lod = 0; lod < 8; ++lod) {
		godot::String lod_prefix = "lod_" + godot::String::num_int64(lod);
		dict[lod_prefix + "_candidates"] = snap->lods[lod].candidate_count;
		dict[lod_prefix + "_visible"] = snap->lods[lod].visible_count;
		dict[lod_prefix + "_resolved_layers"] = static_cast<int64_t>(snap->lods[lod].visible_count);

		if (snap->lods[lod].slots[0].is_fallback && 
			snap->lods[lod].slots[0].state == multinet::rendering::TerrainGpuPageState::Resident && 
			snap->lods[lod].slots[0].gpu_layer == 0) {
			fallback_slot_valid_count++;
		}
		
		for (size_t i = 0; i < snap->lods[lod].candidate_count; ++i) {
			uint8_t face = static_cast<uint8_t>(snap->lods[lod].candidate_keys[i].face);
			if (face < 6) face_candidate_count[face]++;
		}

		uint32_t lod_non_fallback = 0;
		for (size_t i = 0; i < snap->lods[lod].visible_count; ++i) {
			const auto& diag = snap->lods[lod].submitted_visible_diagnostics[i];
			uint8_t face = static_cast<uint8_t>(diag.key.face);
			uint32_t layer = diag.gpu_layer;

			if (face < 6) {
				face_visible_count[face]++;
				if (layer > 0) face_non_fallback_count[face]++;
			}
			if (layer > 0) {
				++lod_non_fallback;
				max_layer = std::max(max_layer, layer);
			}
		}
		dict[lod_prefix + "_non_fallback_pages"] = lod_non_fallback;

		for (size_t i = 1; i < 128; ++i) {
			if (snap->lods[lod].slots[i].state == multinet::rendering::TerrainGpuPageState::Resident) {
				++active_residency;
			}
		}
	}

	dict["active_residency_count"] = active_residency;
	dict["max_resolved_layer"] = max_layer;

	// WP5.1 Emergency Repair live editor diagnostics
	const auto& sd = snap->streaming_diagnostics;
	dict["frame_demand_count"] = sd.frame_demand_count;
	dict["wanted_set_count"] = sd.wanted_set_count;
	dict["wanted_set_overflow"] = sd.wanted_set_overflow;
	dict["pending_poison_count"] = sd.pending_poison_count;
	dict["cancelled_retryable_count"] = sd.cancelled_retryable_count;
	dict["terminal_bootstrap_required"] = sd.terminal_bootstrap_required;
	dict["terminal_bootstrap_resident"] = sd.terminal_bootstrap_resident;
	dict["lod_7_layer_zero_visible"] = sd.lod_7_layer_zero_visible;
	dict["previous_plan_retained_due_to_flat_bootstrap"] = sd.previous_plan_retained_due_to_flat_bootstrap;
	dict["next_ring_terminal_keys_required"] = sd.next_ring_terminal_keys_required;
	dict["next_ring_terminal_keys_resident"] = sd.next_ring_terminal_keys_resident;
	dict["ring_transaction_pending"] = sd.ring_transaction_pending;
	dict["ring_transaction_age_ms"] = sd.ring_transaction_age_ms;

	for (uint8_t lod = 0; lod < 8; ++lod) {
		godot::String lod_prefix = "lod_" + godot::String::num_int64(lod);
		dict[lod_prefix + "_visible_keys"] = snap->lods[lod].visible_keys_count;
		dict[lod_prefix + "_resident_visible_keys"] = snap->lods[lod].resident_visible_keys_count;
		dict[lod_prefix + "_ready_awaiting_gpu"] = snap->lods[lod].ready_awaiting_gpu_count;
		dict[lod_prefix + "_uploaded_this_frame"] = snap->lods[lod].uploaded_this_frame_count;
		dict[lod_prefix + "_layer_zero_visible"] = snap->lods[lod].layer_zero_visible_count;
		dict[lod_prefix + "_next_snap_prefetch_keys"] = snap->lods[lod].next_snap_prefetch_keys_count;
	}

	uint8_t active_face = static_cast<uint8_t>(current_cam_state.canonical_position.face);
	const Multinet::EdgeTransition& trans = Multinet::get_edge_transition(active_face, Multinet::SurfaceEdge::PositiveU);
	uint8_t expected_dest_face = trans.destination_face;

	dict["active canonical face"] = active_face;
	dict["source-face candidate count"] = face_candidate_count[active_face];
	dict["expected destination-face candidate count"] = face_candidate_count[expected_dest_face];
	dict["visible source-face block count"] = face_visible_count[active_face];
	dict["visible destination-face block count"] = face_visible_count[expected_dest_face];
	dict["non-fallback source-face page count"] = face_non_fallback_count[active_face];
	dict["non-fallback destination-face page count"] = face_non_fallback_count[expected_dest_face];
	dict["fallback_slot_valid_count"] = fallback_slot_valid_count;
	dict["maximum submitted texture layer"] = max_layer;

	dict["freeze_update"] = freeze_update;
	dict["frozen_bccm_frame"] = static_cast<int64_t>(frozen_bccm_frame);
	dict["frozen_root_epoch"] = static_cast<int64_t>(frozen_root_epoch);
	dict["frozen_chp_profile_version"] = frozen_chp_profile_version;
	dict["frozen_frustum_snapshot_id"] = static_cast<int64_t>(frozen_frustum_snapshot_id_);
	dict["frozen_frustum_active"] = frozen_frustum_active_;

#ifdef DEBUG_ENABLED
	dict["editor_local_camera_x_m"] = static_cast<double>(editor_view_snapshot.world_position.x);
	dict["editor_local_camera_y_m"] = static_cast<double>(editor_view_snapshot.world_position.y);
	dict["editor_local_camera_z_m"] = static_cast<double>(editor_view_snapshot.world_position.z);
	dict["editor_presentation_rebase_offset_x_m"] = editor_presentation_rebase_offset_x_m;
	dict["editor_presentation_rebase_offset_z_m"] = editor_presentation_rebase_offset_z_m;
	dict["continuous_camera_x_m"] = static_cast<double>(continuous_camera_m_.x);
	dict["continuous_camera_y_m"] = static_cast<double>(continuous_camera_m_.y);
	dict["continuous_camera_z_m"] = static_cast<double>(continuous_camera_m_.z);
	dict["per_frame_camera_delta_x_m"] = static_cast<double>(last_continuous_camera_delta_m_.x);
	dict["per_frame_camera_delta_y_m"] = static_cast<double>(last_continuous_camera_delta_m_.y);
	dict["per_frame_camera_delta_z_m"] = static_cast<double>(last_continuous_camera_delta_m_.z);
	dict["editor_presentation_rebase_count"] = editor_presentation_rebase_count;
	dict["editor_last_presentation_rebase_x_m"] = editor_last_presentation_rebase_x_m;
	dict["editor_last_presentation_rebase_z_m"] = editor_last_presentation_rebase_z_m;
#endif

	const auto& cut_diag = bccm_renderer.get_cut_diagnostics();
	dict["cut_render_update_serial"] = static_cast<int64_t>(cut_diag.render_update_serial);
	dict["cut_camera_delta_x_m"] = cut_diag.camera_delta_x_m;
	dict["cut_camera_delta_z_m"] = cut_diag.camera_delta_z_m;
	dict["cut_ground_distance_moved_m"] = cut_diag.ground_plane_distance_moved_m;
	dict["cut_delta_seconds"] = cut_diag.delta_seconds;
	dict["cut_estimated_speed_m_s"] = cut_diag.estimated_speed_m_s;
	dict["cut_estimated_speed_km_s"] = cut_diag.estimated_speed_km_s;
	dict["high_speed_cut_diagnostics_enabled"] = high_speed_cut_diagnostics_enabled;
	dict["cut_active_lod_count"] = cut_diag.active_lod_count;
	dict["cut_total_instances_submitted"] = cut_diag.total_instances_submitted;
	dict["bccm_streams_submitted"] = cut_diag.bccm_streams_submitted;
	dict["cut_multimesh_buffers_rewritten"] = cut_diag.multimesh_buffers_rewritten;
	dict["cut_total_instance_bytes_uploaded"] = static_cast<int64_t>(cut_diag.total_instance_bytes_uploaded);
	dict["cut_cumulative_instance_bytes_uploaded"] = static_cast<int64_t>(bccm_renderer.get_cumulative_instance_bytes_uploaded());
	dict["cut_total_skipped_snaps"] = static_cast<int64_t>(bccm_renderer.get_total_skipped_snap_events());
	dict["cut_total_buffer_rewrites"] = static_cast<int64_t>(bccm_renderer.get_total_multimesh_buffer_rewrites());
	dict["cut_frame_skipped_snap_events"] = cut_diag.frame_skipped_snap_events;
	dict["cut_frame_largest_snap_steps"] = cut_diag.frame_largest_snap_steps;
	dict["cut_worst_lod"] = cut_diag.worst_lod;
	dict["cut_worst_axis"] = cut_diag.worst_axis;
	dict["cut_worst_candidate_turnover_pct"] = cut_diag.worst_candidate_turnover * 100.0f;

	for (uint8_t lod = 0; lod < 8; ++lod) {
		godot::String lod_p = "cut_lod_" + godot::String::num_int64(lod);
		const auto& ldiag = cut_diag.lods[lod];
		dict[lod_p + "_prev_center_bx"] = static_cast<int64_t>(ldiag.prev_center_bx);
		dict[lod_p + "_prev_center_bv"] = static_cast<int64_t>(ldiag.prev_center_bv);
		dict[lod_p + "_curr_center_bx"] = static_cast<int64_t>(ldiag.current_center_bx);
		dict[lod_p + "_curr_center_bv"] = static_cast<int64_t>(ldiag.current_center_bv);
		dict[lod_p + "_delta_center_bx"] = static_cast<int64_t>(ldiag.delta_center_bx);
		dict[lod_p + "_delta_center_bv"] = static_cast<int64_t>(ldiag.delta_center_bv);
		dict[lod_p + "_delta_center_u_m"] = ldiag.delta_center_u_m;
		dict[lod_p + "_delta_center_v_m"] = ldiag.delta_center_v_m;
		dict[lod_p + "_snap_period_m"] = ldiag.snap_period_m;
		dict[lod_p + "_snap_steps_u"] = ldiag.snap_steps_crossed_u;
		dict[lod_p + "_snap_steps_v"] = ldiag.snap_steps_crossed_v;
		dict[lod_p + "_max_snap_steps"] = ldiag.max_snap_steps_crossed;
		dict[lod_p + "_skipped_snap"] = ldiag.skipped_snap_event;
		dict[lod_p + "_hole_dx"] = ldiag.current_hole_dx;
		dict[lod_p + "_hole_dz"] = ldiag.current_hole_dz;
		dict[lod_p + "_hole_delta_dx"] = ldiag.hole_delta_dx;
		dict[lod_p + "_hole_delta_dz"] = ldiag.hole_delta_dz;
		dict[lod_p + "_hole_moved"] = ldiag.hole_movement_event;
		dict[lod_p + "_hole_steps"] = ldiag.hole_steps_crossed;
		dict[lod_p + "_cand_before"] = ldiag.candidate_count_before;
		dict[lod_p + "_cand_after"] = ldiag.candidate_count_after;
		dict[lod_p + "_cand_retained"] = ldiag.candidates_retained;
		dict[lod_p + "_cand_added"] = ldiag.candidates_added;
		dict[lod_p + "_cand_removed"] = ldiag.candidates_removed;
		dict[lod_p + "_turnover_pct"] = ldiag.turnover_fraction * 100.0f;
		dict[lod_p + "_submitted_instances"] = ldiag.submitted_instance_count;
		dict[lod_p + "_buffer_changed"] = ldiag.instance_buffer_changed;
		dict[lod_p + "_bytes_uploaded"] = static_cast<int64_t>(ldiag.instance_bytes_uploaded);
	}


	dict["live_camera_x_m"] = current_cam_state.presentation_x_m;
	dict["live_camera_y_m"] = current_cam_state.canonical_position.altitude_m;
	dict["live_camera_z_m"] = current_cam_state.presentation_z_m;
	dict["live_chp_camera_altitude_m"] = current_chp_view.camera_surface_height_m;

	return dict;
}

#ifdef DEBUG_ENABLED
void MultinetBCCMNode3D::update_editor_observer_from_editor_camera() {
	if (!editor_view_snapshot.valid || !world_domain_manifest.is_valid()) {
		editor_observer_state.valid = false;
		return;
	}
	const godot::Vector3 current = editor_view_snapshot.world_position;
	if (editor_observer_state.initialized && !world_domain_manifest.is_finite()) {
		// Presentation placement is an editor-camera contract. A failed canonical
		// step must remain diagnosable, but it must never strand the visible grid at
		// its previous X/Z anchor while the editor camera keeps moving.
		editor_observer_state.presentation_origin_world.x = current.x;
		editor_observer_state.presentation_origin_world.z = current.z;
	}
	const uint64_t now_us = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count());
	const bool domain_changed = editor_observer_state.domain_manifest_hash != world_domain_manifest.domain_manifest_hash;
	if (!editor_observer_state.initialized || domain_changed) {
		editor_observer_state = EditorCanonicalObserverState{};
		editor_observer_state.initialized = true;
		editor_observer_state.topology = world_domain_input.topology;
		editor_observer_state.domain_manifest_hash = world_domain_manifest.domain_manifest_hash;
		editor_observer_state.canonical_position.face = Multinet::SurfaceFace::PositiveX;
		if (world_domain_manifest.is_finite()) {
			const double half_x_m = static_cast<double>(world_domain_manifest.finite.half_extent_x_mm) * 0.001;
			const double half_z_m = static_cast<double>(world_domain_manifest.finite.half_extent_z_mm) * 0.001;
			editor_observer_state.canonical_position.u_m = std::clamp(static_cast<double>(current.x), -half_x_m, half_x_m);
			editor_observer_state.canonical_position.v_m = std::clamp(static_cast<double>(current.z), -half_z_m, half_z_m);
			editor_observer_state.outside_viewport_camera =
				std::abs(static_cast<double>(current.x) - editor_observer_state.canonical_position.u_m) > 1e-6 ||
				std::abs(static_cast<double>(current.z) - editor_observer_state.canonical_position.v_m) > 1e-6;
		} else {
			editor_observer_state.canonical_position.u_m = 0.0;
			editor_observer_state.canonical_position.v_m = 0.0;
		}
		editor_observer_state.canonical_position.altitude_m = static_cast<double>(current.y);
		editor_observer_state.canonical_position.topology_version = world_domain_manifest.topology_version;
		editor_observer_state.canonical_position.projection_version = world_domain_manifest.projection_version;
		editor_observer_state.frame_epoch = 1;
		editor_observer_state.active_frame = make_editor_frame_for_face(
			world_domain_input.topology,
			editor_observer_state.canonical_position.face,
		editor_observer_state.canonical_position,
		editor_observer_state.frame_epoch,
			world_domain_manifest.topology_version,
			world_domain_manifest.projection_version
		);
		editor_observer_state.unfolding_root_frame = editor_observer_state.active_frame;
		editor_observer_state.unfolding_root_frame.origin.altitude_m = 0.0;
		if (!world_domain_manifest.is_finite() && (current.x != 0.0f || current.z != 0.0f)) {
			// Preserve the editor's existing flat X/Z location when wrapping is
			// enabled or rebuilt. Starting every closed world at canonical zero
			// made toggles teleport the terrain under a stationary editor camera.
			Multinet::SurfacePosition64 initial_position{};
			Multinet::SurfaceFrame initial_frame{};
			uint32_t initial_transitions = 0;
			const Multinet::FramePosition64 initial_delta{
				static_cast<double>(current.x), 0.0, static_cast<double>(current.z)
			};
			if (!Multinet::try_advance_domain_surface_frame(
				initial_delta,
				world_domain_manifest,
				editor_observer_state.active_frame,
				initial_frame,
				initial_position,
				initial_transitions
			)) {
				editor_observer_state.valid = false;
				return;
			}
			editor_observer_state.canonical_position = initial_position;
			editor_observer_state.active_frame = initial_frame;
			editor_observer_state.frame_epoch = initial_frame.frame_epoch;
		}
		editor_observer_state.unfolding_root_frame = editor_observer_state.active_frame;
		editor_observer_state.unfolding_root_frame.origin.altitude_m = 0.0;
		editor_observer_state.unfolding_root_presentation_x_m = static_cast<double>(current.x);
		editor_observer_state.unfolding_root_presentation_z_m = static_cast<double>(current.z);
		refresh_tracking_logical_chart(
			world_domain_manifest,
			editor_observer_state.unfolding_root_frame,
			editor_observer_state.has_logical_chart,
			editor_observer_state.logical_chart_root_direction,
			editor_observer_state.logical_chart_presentation_x_tangent,
			editor_observer_state.logical_chart_presentation_z_tangent);
		editor_observer_state.presentation_origin_world = world_domain_manifest.is_finite()
			? godot::Vector3(
				static_cast<float>(editor_observer_state.canonical_position.u_m),
				static_cast<float>(editor_observer_state.canonical_position.altitude_m),
				static_cast<float>(editor_observer_state.canonical_position.v_m))
			: godot::Vector3(current.x, 0.0f, current.z);
		editor_observer_state.presentation_generation = 1;
		editor_observer_state.last_editor_camera_world_position = current;
		editor_observer_state.has_last_editor_camera_world_position = true;
		editor_observer_state.last_update_monotonic_us = now_us;
		editor_observer_state.valid = true;
	} else if (editor_observer_state.has_last_editor_camera_world_position) {
		const godot::Vector3 delta = current - editor_observer_state.last_editor_camera_world_position;
		const double delta_seconds = editor_observer_state.last_update_monotonic_us > 0 &&
			now_us > editor_observer_state.last_update_monotonic_us
			? static_cast<double>(now_us - editor_observer_state.last_update_monotonic_us) / 1000000.0
			: 0.0;
		if (delta_seconds > 0.0) {
			const double dx = static_cast<double>(delta.x);
			const double dy = static_cast<double>(delta.y);
			const double dz = static_cast<double>(delta.z);
			editor_observer_state.ground_speed_m_s = std::sqrt(dx * dx + dz * dz) / delta_seconds;
			editor_observer_state.vertical_speed_m_s = dy / delta_seconds;
			editor_observer_state.total_speed_m_s = std::sqrt(dx * dx + dy * dy + dz * dz) / delta_seconds;
		} else {
			editor_observer_state.ground_speed_m_s = 0.0;
			editor_observer_state.vertical_speed_m_s = 0.0;
			editor_observer_state.total_speed_m_s = 0.0;
		}
		// Finite terrain owns Godot world X/Z directly. The editor camera remains
		// free outside the rectangle while canonical diagnostics stop at its edge.
		// Closed mode continues to transport editor intent through the active frame.
		Multinet::FramePosition64 local_delta{};
		bool finite_target_outside = false;
		if (world_domain_manifest.is_finite()) {
			const double half_x_m = static_cast<double>(world_domain_manifest.finite.half_extent_x_mm) * 0.001;
			const double half_z_m = static_cast<double>(world_domain_manifest.finite.half_extent_z_mm) * 0.001;
			const double desired_u_m = static_cast<double>(current.x);
			const double desired_v_m = static_cast<double>(current.z);
			const double target_u_m = std::clamp(desired_u_m, -half_x_m, half_x_m);
			const double target_v_m = std::clamp(desired_v_m, -half_z_m, half_z_m);
			finite_target_outside = std::abs(desired_u_m - target_u_m) > 1e-6 ||
				std::abs(desired_v_m - target_v_m) > 1e-6;
			local_delta = Multinet::FramePosition64{
				target_u_m - editor_observer_state.canonical_position.u_m,
				static_cast<double>(current.y) - editor_observer_state.canonical_position.altitude_m,
				target_v_m - editor_observer_state.canonical_position.v_m
			};
		} else {
			if (!try_map_closed_presentation_motion(
				world_domain_manifest,
				editor_observer_state.canonical_position,
				editor_observer_state.has_logical_chart,
				editor_observer_state.logical_chart_root_direction,
				editor_observer_state.logical_chart_presentation_x_tangent,
				editor_observer_state.logical_chart_presentation_z_tangent,
				static_cast<double>(delta.x),
				static_cast<double>(delta.z),
				local_delta
			)) {
				// The chart is part of the Hybrid/page presentation contract. Do not
				// silently fall back to cube-frame controls and split direction again.
				++editor_observer_state.rejected_chart_motion_count;
				editor_observer_state.last_editor_camera_world_position = current;
				editor_observer_state.last_update_monotonic_us = now_us;
				return;
			}
		}
		local_delta.y = static_cast<double>(delta.y);
		Multinet::SurfacePosition64 next_position;
		Multinet::SurfaceFrame next_frame;
		uint32_t transition_count = 0;
		const Multinet::SurfaceFace transition_initial_face = editor_observer_state.canonical_position.face;
		Multinet::SurfaceFace last_source = editor_observer_state.canonical_position.face;
		Multinet::SurfaceFace last_destination = last_source;
		Multinet::SurfaceEdge last_edge = Multinet::SurfaceEdge::NegativeU;
		if (Multinet::try_advance_domain_surface_frame(
			local_delta,
			world_domain_manifest,
			editor_observer_state.active_frame,
			next_frame,
			next_position,
			transition_count,
			&last_source,
			&last_destination,
			&last_edge
		)) {
			editor_observer_state.outside_viewport_camera = finite_target_outside;
			editor_observer_state.canonical_position = next_position;
			editor_observer_state.active_frame = next_frame;
			editor_observer_state.frame_epoch = next_frame.frame_epoch;
			if (world_domain_manifest.is_finite()) {
				editor_observer_state.presentation_origin_world = godot::Vector3(
					static_cast<float>(next_position.u_m),
					static_cast<float>(next_position.altitude_m),
					static_cast<float>(next_position.v_m)
				);
			} else {
				editor_observer_state.presentation_origin_world = godot::Vector3(
					current.x,
					current.y - static_cast<float>(next_position.altitude_m),
					current.z
				);
				const bool analytic_chart_can_track =
					bccm_renderer.get_source_mode() == multinet::rendering::TerrainSourceMode::AnalyticBase &&
					!bccm_renderer.get_analytic_debug_prewarm_pages();
				if (analytic_chart_can_track) {
					// Move by the real observer delta, not a threshold jump. The old
					// frozen chart caused the curved colour seam and pathological trig
					// cost after long editor travel.
					editor_observer_state.unfolding_root_frame = next_frame;
					editor_observer_state.unfolding_root_frame.origin.altitude_m = 0.0;
					editor_observer_state.unfolding_root_presentation_x_m = static_cast<double>(current.x);
					editor_observer_state.unfolding_root_presentation_z_m = static_cast<double>(current.z);
					refresh_tracking_logical_chart(
						world_domain_manifest,
						editor_observer_state.unfolding_root_frame,
						editor_observer_state.has_logical_chart,
						editor_observer_state.logical_chart_root_direction,
						editor_observer_state.logical_chart_presentation_x_tangent,
						editor_observer_state.logical_chart_presentation_z_tangent);
				}
			}
			if (transition_count > 0) {
				editor_observer_state.last_transition_count = transition_count;
				editor_observer_state.last_transition_frame_epoch = next_frame.frame_epoch;
				editor_observer_state.last_transition_initial_face = static_cast<int>(transition_initial_face);
				editor_observer_state.last_transition_final_face = static_cast<int>(next_position.face);
				editor_observer_state.last_transition_source_face = static_cast<int>(last_source);
				editor_observer_state.last_transition_destination_face = static_cast<int>(last_destination);
				editor_observer_state.last_transition_edge = static_cast<int>(last_edge);
			}
		} else {
			if (world_domain_manifest.is_finite()) {
				editor_observer_state.outside_viewport_camera = true;
			} else {
				++editor_observer_state.rejected_frame_advance_count;
			}
		}
	}
	editor_observer_state.last_editor_camera_world_position = current;
	editor_observer_state.has_last_editor_camera_world_position = true;
	editor_observer_state.last_update_monotonic_us = now_us;
	editor_view_snapshot.editor_frame_epoch = editor_observer_state.frame_epoch;
	editor_view_snapshot.last_editor_face = static_cast<int>(editor_observer_state.canonical_position.face);
}

void MultinetBCCMNode3D::publish_editor_view_camera(godot::Camera3D* p_editor_camera) {
	if (!Engine::get_singleton()->is_editor_hint() || !p_editor_camera) {
		editor_view_snapshot.valid = false;
		return;
	}

	// Closed presentation uses a floating editor origin. Put the accumulated
	// flat editor location back when returning to finite mode so the fixed
	// rectangle remains at the real Godot origin instead of following the view.
	if (world_domain_manifest.is_finite() && editor_observer_state.initialized &&
		editor_observer_state.topology != Multinet::WorldDomainTopology::FiniteRectangle &&
		(editor_presentation_rebase_offset_x_m != 0.0 || editor_presentation_rebase_offset_z_m != 0.0)) {
		godot::Transform3D restored_transform = p_editor_camera->get_global_transform();
		restored_transform.origin.x = static_cast<godot::real_t>(
			static_cast<double>(restored_transform.origin.x) + editor_presentation_rebase_offset_x_m);
		restored_transform.origin.z = static_cast<godot::real_t>(
			static_cast<double>(restored_transform.origin.z) + editor_presentation_rebase_offset_z_m);
		p_editor_camera->set_global_transform(restored_transform);
		// Godot may defer the viewport camera's transform cache. Flush it before
		// extracting the frustum so culling and the mirrored scene camera share the
		// same restored origin in this frame.
		p_editor_camera->force_update_transform();
		editor_presentation_rebase_offset_x_m = 0.0;
		editor_presentation_rebase_offset_z_m = 0.0;
	}

	// Copy position and frustum planes from the actual editor viewport camera.
	// No Camera3D pointer is retained beyond this function.
	editor_view_snapshot.world_position = p_editor_camera->get_global_position();
	editor_view_snapshot.camera_forward_world = -p_editor_camera->get_global_transform().basis.get_column(2);
	camera_forward_world = editor_view_snapshot.camera_forward_world;
	const double heading_length = std::sqrt(
		static_cast<double>(camera_forward_world.x) * camera_forward_world.x +
		static_cast<double>(camera_forward_world.z) * camera_forward_world.z);
	has_camera_forward_world = std::isfinite(heading_length) && heading_length > 1e-6;
	editor_view_snapshot.frustum = multinet::rendering::FrustumPlanes::extract_from_camera(p_editor_camera);
	editor_view_snapshot.valid = editor_view_snapshot.frustum.valid;

	if (editor_view_snapshot.valid) {
		// publication_serial: increments every valid editor frame.
		// Never used as frame_epoch — kept separate for bookkeeping only.
		editor_view_snapshot.publication_serial++;

		// The observer update below owns canonical frame identity and epoch.
		update_editor_observer_from_editor_camera();

		const double local_x_m = static_cast<double>(editor_view_snapshot.world_position.x);
		const double local_z_m = static_cast<double>(editor_view_snapshot.world_position.z);
		if (!world_domain_manifest.is_finite() && editor_observer_state.valid &&
			(std::abs(local_x_m) >= EDITOR_PRESENTATION_REBASE_THRESHOLD_M ||
			 std::abs(local_z_m) >= EDITOR_PRESENTATION_REBASE_THRESHOLD_M)) {
			// Consume only whole outer-ring lattice periods. Re-centering by an
			// arbitrary camera offset changes every presentation block index (for
			// example 4,200 m -> 0 m), which makes the old and new ring sets overlap
			// while the GPU submission is in flight. 4,096 m is the LOD7 block size
			// for the standard profile and is an integer multiple of every finer LOD.
			// Canonical position, frame identity, and page keys remain authoritative.
			godot::Transform3D rebased_transform = p_editor_camera->get_global_transform();
			const auto quantized_rebase_shift = [](double local_m) {
				if (local_m >= EDITOR_PRESENTATION_REBASE_THRESHOLD_M) {
					return std::floor(local_m / EDITOR_PRESENTATION_REBASE_THRESHOLD_M) *
						EDITOR_PRESENTATION_REBASE_THRESHOLD_M;
				}
				if (local_m <= -EDITOR_PRESENTATION_REBASE_THRESHOLD_M) {
					return std::ceil(local_m / EDITOR_PRESENTATION_REBASE_THRESHOLD_M) *
						EDITOR_PRESENTATION_REBASE_THRESHOLD_M;
				}
				return 0.0;
			};
			const double shift_x_m = quantized_rebase_shift(local_x_m);
			const double shift_z_m = quantized_rebase_shift(local_z_m);
			const godot::Vector3 shift{
				static_cast<godot::real_t>(shift_x_m),
				0.0f,
				static_cast<godot::real_t>(shift_z_m)
			};
			rebased_transform.origin.x -= static_cast<godot::real_t>(shift_x_m);
			rebased_transform.origin.z -= static_cast<godot::real_t>(shift_z_m);
			p_editor_camera->set_global_transform(rebased_transform);
			// Publish the post-rebase transform immediately; otherwise get_frustum()
			// can describe the pre-rebase camera for one frame and cull the side rings.
			p_editor_camera->force_update_transform();

			editor_presentation_rebase_offset_x_m += static_cast<double>(shift.x);
			editor_presentation_rebase_offset_z_m += static_cast<double>(shift.z);
			editor_last_presentation_rebase_x_m = static_cast<double>(shift.x);
			editor_last_presentation_rebase_z_m = static_cast<double>(shift.z);
			++editor_presentation_rebase_count;

			editor_view_snapshot.world_position -= shift;
			editor_observer_state.last_editor_camera_world_position -= shift;
			editor_observer_state.presentation_origin_world -= shift;
			editor_observer_state.unfolding_root_presentation_x_m -= static_cast<double>(shift.x);
			editor_observer_state.unfolding_root_presentation_z_m -= static_cast<double>(shift.z);
			editor_view_snapshot.frustum = multinet::rendering::FrustumPlanes::extract_from_camera(p_editor_camera);
			editor_view_snapshot.valid = editor_view_snapshot.frustum.valid;
		}

		if (freeze_update && frozen_frustum_active_ && !bccm_renderer.has_frozen_frustum_visualization() && bccm_renderer.initialized()) {
			const float fov = p_editor_camera->get_fov();
			const float near_m = p_editor_camera->get_near();
			const float far_m = p_editor_camera->get_far();
			float aspect = 16.0f / 9.0f;
			if (get_viewport()) {
				const godot::Vector2 size = get_viewport()->get_visible_rect().size;
				if (size.y > 0.0f) aspect = size.x / size.y;
			}
			bccm_renderer.set_frozen_frustum_visualization(p_editor_camera->get_global_transform(), fov, near_m, far_m, aspect);
		}
	}
}
#endif

} // namespace godot
