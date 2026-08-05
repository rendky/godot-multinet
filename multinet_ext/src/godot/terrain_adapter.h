#ifndef MULTINET_TERRAIN_ADAPTER_H
#define MULTINET_TERRAIN_ADAPTER_H

#include <godot_cpp/classes/node3d.hpp>
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_renderer.h"
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
	multinet::rendering::FrustumPlanes frustum{};
	uint64_t publication_serial{ 0 };
	uint64_t editor_frame_epoch{ 0 };
	int last_editor_face{ -1 }; // tracks face to detect SurfaceFrame changes
	bool valid{ false };
};
#endif

class MultinetBCCMNode3D : public Node3D {
	GDCLASS(MultinetBCCMNode3D, Node3D);

private:
	multinet::rendering::BlockClipmapRenderer bccm_renderer;

#ifdef DEBUG_ENABLED
	EditorViewSnapshot editor_view_snapshot;
#endif

	Multinet::TerrainRecipe recipe;
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

	// Deferred one-shot rebuild flag. Set by request_recipe_rebuild().
	// Applied at the next _process() tick BEFORE update(). Never calls free_rendering().
	bool recipe_rebuild_pending{ false };

	godot::NodePath camera_target;

	void init_rendering();
	void free_rendering();
	void rebuild_source();
	void build_source_expectation();

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

	// Inspector recipe properties.
	// "seed"      -> recipe.identity.world_seed       (serialized name preserved from test.tscn)
	// "frequency" -> legacy_signals.continental_frequency (serialized name preserved from test.tscn)
	void set_seed(uint32_t p_seed);
	uint32_t get_seed() const;

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
#endif

	// Read-only Variant summary dictionary for runtime smoke assertions
	godot::Dictionary get_debug_summary() const;
};

} // namespace godot

#endif // MULTINET_TERRAIN_ADAPTER_H
