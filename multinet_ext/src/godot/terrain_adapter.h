#ifndef MULTINET_TERRAIN_ADAPTER_H
#define MULTINET_TERRAIN_ADAPTER_H

#include <godot_cpp/classes/node3d.hpp>
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_renderer.h"
#include "multinet/rendering/chp/chp_certification.h"
#include "multinet/rendering/chp/chp_view.h"
#include "multinet/world/terrain/outputs/rendering/concrete_terrain_render_source.h"
#include "multinet/core/jobs/bounded_background_job_executor.h"
#include <memory>

namespace godot {

class Camera3D;

#ifdef DEBUG_ENABLED
// EditorViewSnapshot: published every editor frame from publish_editor_view_camera.
// - publication_serial: increments every time a new valid snapshot arrives.
//   Used only for bookkeeping; never passed as frame_epoch.
// - editor_frame_epoch: increments only when the SurfaceFrame identity changes
//   (e.g., when the active face index changes). Stable between face transitions.
//   Passed as frame_epoch to set_canonical_camera_state.
struct EditorViewSnapshot {
	godot::Vector3 world_position{};
	godot::Vector3 camera_forward_world{ 0.0f, 0.0f, -1.0f };
	multinet::rendering::FrustumPlanes frustum{};
	uint64_t publication_serial{ 0 };
	uint64_t editor_frame_epoch{ 0 };
	int last_editor_face{ -1 }; // tracks face to detect SurfaceFrame changes
	bool valid{ false };
};

struct EditorCanonicalObserverState {
	bool initialized{ false };
	Multinet::WorldDomainTopology topology{ Multinet::WorldDomainTopology::ClosedSurfaceSixFace };
	Multinet::SurfacePosition64 canonical_position{};
	Multinet::SurfaceFrame active_frame{};
	Multinet::SurfaceFrame unfolding_root_frame{};
	double unfolding_root_presentation_x_m{ 0.0 };
	double unfolding_root_presentation_z_m{ 0.0 };
	Multinet::FramePosition64 logical_chart_root_direction{};
	Multinet::FramePosition64 logical_chart_presentation_x_tangent{};
	Multinet::FramePosition64 logical_chart_presentation_z_tangent{};
	bool has_logical_chart{ false };
	godot::Vector3 presentation_origin_world{};
	godot::Vector3 last_editor_camera_world_position{};
	bool has_last_editor_camera_world_position{ false };
	uint64_t domain_manifest_hash{ 0 };
	uint64_t publication_serial{ 0 };
	uint64_t frame_epoch{ 0 };
	// The chart and its flat anchor are immutable within one presentation
	// generation. Moving either one reprojects already-visible terrain.
	uint64_t presentation_generation{ 1 };
	uint64_t last_update_monotonic_us{ 0 };
	uint64_t rejected_chart_motion_count{ 0 };
	uint64_t rejected_frame_advance_count{ 0 };
	double ground_speed_m_s{ 0.0 };
	double vertical_speed_m_s{ 0.0 };
	double total_speed_m_s{ 0.0 };
	uint32_t last_transition_count{ 0 };
	// This is a receipt for the most recent real topology event, not a
	// per-viewport-tick flag. A quiet editor tick must not erase it before the
	// person driving the viewport has a chance to inspect the crossing.
	uint64_t last_transition_frame_epoch{ 0 };
	int last_transition_initial_face{ -1 };
	int last_transition_final_face{ -1 };
	int last_transition_source_face{ -1 };
	int last_transition_destination_face{ -1 };
	int last_transition_edge{ -1 };
	bool outside_viewport_camera{ false };
	bool valid{ false };
};
#endif

class MultinetBCCMNode3D : public Node3D {
	GDCLASS(MultinetBCCMNode3D, Node3D);

private:
	multinet::rendering::BlockClipmapRenderer bccm_renderer;

#ifdef DEBUG_ENABLED
	EditorViewSnapshot editor_view_snapshot;
	EditorCanonicalObserverState editor_observer_state;
	double editor_presentation_rebase_offset_x_m{ 0.0 };
	double editor_presentation_rebase_offset_z_m{ 0.0 };
	double editor_last_presentation_rebase_x_m{ 0.0 };
	double editor_last_presentation_rebase_z_m{ 0.0 };
	uint64_t editor_presentation_rebase_count{ 0 };
	std::shared_ptr<Multinet::TerrainCommittedDeltaField> debug_held_delta_field;
	uint32_t debug_publication_version{ 1 };
#endif

	Multinet::TerrainRecipe recipe;
	Multinet::WorldDomainInput world_domain_input{};
	Multinet::WorldDomainManifest world_domain_manifest{};
	Multinet::WorldPresentationInput world_presentation_input{};
	Multinet::WorldPresentationManifest world_presentation_manifest{};
	multinet::rendering::chp::CurvedHorizonProfile chp_profile{};
	multinet::rendering::chp::ResolvedCurvedHorizonProfile resolved_chp_profile{};
	multinet::rendering::chp::CurvedHorizonView current_chp_view{};
	bool square_world{ true };
	bool finite_aspect_history_valid{ false };
	bool finite_aspect_history_square_world{ false };
	uint64_t finite_aspect_history_x_m{ 0 };
	uint64_t finite_aspect_history_z_m{ 0 };
	bool face_colors_enabled{ true };
	bool diamond_triangulation_enabled{ true };
	int closed_flat_coverage_radius_blocks{ 4 };
	Multinet::WorldScaleManifest manifest;
	std::unique_ptr<Multinet::ConcreteTerrainRenderSource> render_source;

	// Background job executor — owned here for WP5.
	// Must be shut down before render_source is destroyed.
	std::unique_ptr<Multinet::BoundedBackgroundJobExecutor> executor;

	multinet::rendering::BCCMCameraState current_cam_state;
	multinet::rendering::BCCMSourceExpectation source_expectation;

	bool source_dirty{ true };
	bool manifest_built{ false };
	bool freeze_update{ false };
	godot::String domain_validation_message{ "OK" };
	double runtime_ground_speed_m_s{ 0.0 };
	double runtime_vertical_speed_m_s{ 0.0 };
	double runtime_total_speed_m_s{ 0.0 };
	uint32_t runtime_last_transition_count{ 0 };
	int runtime_last_transition_source_face{ -1 };
	int runtime_last_transition_destination_face{ -1 };
	int runtime_last_transition_edge{ -1 };
	bool runtime_outside_finite_boundary{ false };
	godot::Vector3 camera_forward_world{ 0.0f, 0.0f, -1.0f };
	bool has_camera_forward_world{ false };

	// Deferred one-shot rebuild flag. Set by request_recipe_rebuild().
	// Applied at the next _process() tick BEFORE update(). Never calls free_rendering().
	bool recipe_rebuild_pending{ false };
	bool domain_rebuild_pending{ false };
	bool presentation_rebuild_pending{ false };

	godot::NodePath camera_target;

	void init_rendering();
	void free_rendering();
	void rebuild_source();
	void build_source_expectation();
	void refresh_source_expectation_from_source();
	[[nodiscard]] bool configure_bccm_profile();
	void refresh_chp_contract();
	void refresh_chp_view();

	// Marks source_dirty and recipe_rebuild_pending.
	// Setters call this — not free_rendering().
	void request_recipe_rebuild();

	// Performs one deferred rebuild cycle:
	//   cleanup renderer, reset source, preserve executor,
	//   finalize recipe, reconstruct source, rebuild expectation,
	//   reinitialize renderer.
	void apply_recipe_rebuild();

protected:
	static void _bind_methods();
	void _validate_property(PropertyInfo& p_property) const;
	void _notification(int p_what);

public:
	MultinetBCCMNode3D();
	~MultinetBCCMNode3D();

	// Called by the CanonicalObserverController3D to inject canonical state.
	// Must NOT derive from Camera3D.global_transform.
	void set_canonical_camera_state(
		const Multinet::SurfacePosition64& p_canonical_pos,
		const Multinet::SurfaceFrame& p_active_frame,
		uint64_t p_frame_epoch
	);

	// GDScript-callable binding: constructs SurfacePosition64 and SurfaceFrame
	// from primitive values so the observer controller does not need C++ structs.
	// face: 0=PositiveX 1=NegativeX 2=PositiveY 3=NegativeY 4=PositiveZ 5=NegativeZ
	void set_canonical_camera_state_from_values(
		int p_face,
		double p_u_m,
		double p_v_m,
		double p_altitude_m,
		uint64_t p_frame_epoch
	);
	godot::Dictionary advance_canonical_observer(
		double p_presentation_dx,
		double p_altitude_dy,
		double p_presentation_dz,
		double p_delta_seconds
	);

	// Inspector recipe properties.
	// "seed"      -> recipe.identity.world_seed       (serialized name preserved from test.tscn)
	// "frequency" -> legacy_signals.continental_frequency (serialized name preserved from test.tscn)
	void set_seed(uint32_t p_seed);
	uint32_t get_seed() const;

	void set_source_mode(int p_mode);
	int get_source_mode() const;

	void set_analytic_debug_prewarm_pages(bool p_prewarm);
	bool get_analytic_debug_prewarm_pages() const;

	void set_face_colors_enabled(bool p_enabled);
	bool get_face_colors_enabled() const;
	void set_diamond_triangulation_enabled(bool p_enabled);
	bool get_diamond_triangulation_enabled() const;

	void set_frequency(float p_freq);
	float get_frequency() const;

	void set_regional_frequency(float p_freq);
	float get_regional_frequency() const;

	void set_detail_frequency(float p_freq);
	float get_detail_frequency() const;

	void set_min_elevation_m(float p_elev);
	float get_min_elevation_m() const;

	void set_max_elevation_m(float p_elev);
	float get_max_elevation_m() const;

	void set_octave_count(int p_count);
	int get_octave_count() const;

	void set_persistence(float p_val);
	float get_persistence() const;

	void set_lacunarity(float p_val);
	float get_lacunarity() const;

	void set_coordinate_wrapping(bool p_enabled);
	bool get_coordinate_wrapping() const;
	void set_square_world(bool p_enabled);
	bool get_square_world() const;
	void set_world_side_km(double p_km);
	double get_world_side_km() const;
	void set_world_extent_x_km(double p_km);
	double get_world_extent_x_km() const;
	void set_world_extent_z_km(double p_km);
	double get_world_extent_z_km() const;
	void set_closed_equivalent_side_km(double p_km);
	double get_closed_equivalent_side_km() const;
	void set_closed_flat_coverage_radius_blocks(int p_radius);
	int get_closed_flat_coverage_radius_blocks() const;
	double get_closed_flat_visible_extent_km() const;
	void set_chp_enabled(bool p_enabled);
	bool get_chp_enabled() const;
	void set_chp_radius_policy(int p_policy);
	int get_chp_radius_policy() const;
	void set_chp_explicit_radius_km(double p_km);
	double get_chp_explicit_radius_km() const;

	// Read-only derived world-domain diagnostics.
	double get_canonical_area_km2() const;
	double get_logical_radius_km() const;
	double get_closed_face_extent_km() const;
	double get_closed_face_half_extent_km() const;
	uint32_t get_regions_per_face_axis() const;
	double get_actual_region_extent_m() const;
	uint64_t get_active_domain_hash() const;
	uint64_t get_active_presentation_hash() const;
	godot::String get_active_domain_hash_text() const;
	godot::String get_active_presentation_hash_text() const;
	godot::String get_domain_validation_message() const;
	godot::String get_chp_status() const;

	void set_camera_target(const godot::NodePath& p_path);
	godot::NodePath get_camera_target() const;

	void set_freeze_update(bool p_freeze) { freeze_update = p_freeze; }
	bool get_freeze_update() const { return freeze_update; }

	uint32_t get_candidate_count(int p_lod) const;
	uint32_t get_visible_count(int p_lod) const;
	uint32_t get_submitted_streams() const;

#ifdef DEBUG_ENABLED
	// C++ debug publisher for smoke testing (computes valid SurfacePosition64 and SurfaceFrame)
	void debug_publish_synthetic_edge_camera(godot::Camera3D* p_camera, double p_signed_distance_to_edge_m, uint64_t p_epoch);
	void publish_editor_view_camera(godot::Camera3D* p_editor_camera);
	void update_editor_observer_from_editor_camera();
	godot::Dictionary debug_publish_diagnostic_delta(
		int p_face,
		double p_u_m,
		double p_v_m,
		double p_radius_m,
		float p_amplitude_m,
		uint32_t p_content_version,
		bool p_hold_generation
	);
	void debug_publish_null_delta();
	void debug_release_held_delta_generation();
	godot::Dictionary debug_get_block_state(int p_face, int p_block_u, int p_block_v, int p_lod) const;
#endif

	// Read-only Variant summary dictionary for runtime smoke assertions
	godot::Dictionary get_debug_summary() const;
};

} // namespace godot

#endif // MULTINET_TERRAIN_ADAPTER_H
