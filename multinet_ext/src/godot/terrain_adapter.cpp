#include "godot/terrain_adapter.h"
#include "multinet/core/spatial/surface_topology.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <iostream>

namespace godot {

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

#ifdef DEBUG_ENABLED
	ClassDB::bind_method(D_METHOD("debug_publish_synthetic_edge_camera", "camera", "signed_distance_to_edge_m", "epoch"), &MultinetBCCMNode3D::debug_publish_synthetic_edge_camera);
	ClassDB::bind_method(D_METHOD("publish_editor_view_camera", "editor_camera"), &MultinetBCCMNode3D::publish_editor_view_camera);
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
	ADD_GROUP("", "");
	ClassDB::bind_method(D_METHOD("set_camera_target", "path"), &MultinetBCCMNode3D::set_camera_target);
	ClassDB::bind_method(D_METHOD("get_camera_target"), &MultinetBCCMNode3D::get_camera_target);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "camera_target"), "set_camera_target", "get_camera_target");
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

	// 1. Clean up renderer resources.
	bccm_renderer.cleanup();

	// 2. Shut down and reset the old source (in-flight jobs drain first).
	if (render_source) {
		render_source->shutdown();
		render_source.reset();
	}

	// 3. Executor is preserved — not destroyed.

	// 4. Finalize recipe against manifest (rebuilds hash).
	Multinet::WorldScaleInput input{};
	manifest = Multinet::build_world_scale_manifest(input);
	if (!manifest.is_valid()) {
		std::cerr << "[MultinetBCCMNode3D] apply_recipe_rebuild: invalid manifest." << std::endl;
		return;
	}
	if (!Multinet::finalize_terrain_recipe(recipe, manifest)) {
		std::cerr << "[MultinetBCCMNode3D] apply_recipe_rebuild: finalize_terrain_recipe failed." << std::endl;
		return;
	}

	// 5. Ensure executor is running.
	if (!executor) {
		executor = std::make_unique<Multinet::BoundedBackgroundJobExecutor>();
	}

	// 6. Construct new ConcreteTerrainRenderSource.
	render_source = std::make_unique<Multinet::ConcreteTerrainRenderSource>(
		recipe, manifest, *executor
	);

	// 7. Rebuild BCCMSourceExpectation.
	build_source_expectation();

	manifest_built = true;
	source_dirty = false;

	// 8. Reinitialize renderer.
	if (get_world_3d().is_valid()) {
		auto* rs = godot::RenderingServer::get_singleton();
		Multinet::TerrainFallbackBounds fb = render_source->get_snapshot().fallback_bounds;
		if (!bccm_renderer.initialize(rs, get_world_3d()->get_scenario(), manifest, recipe.identity, fb)) {
			std::cerr << "[MultinetBCCMNode3D] apply_recipe_rebuild: renderer initialization failed." << std::endl;
		}
	}
}

// ---------------------------------------------------------------------------
// Internal pipeline
// ---------------------------------------------------------------------------

void MultinetBCCMNode3D::build_source_expectation() {
	source_expectation.recipe_identity = recipe.identity;
	source_expectation.world_manifest_hash = manifest.manifest_hash;
	source_expectation.topology_version = manifest.topology_version;
	source_expectation.projection_version = manifest.projection_version;
	source_expectation.terrain_version = 1;
	source_expectation.source_version = 1;
}

void MultinetBCCMNode3D::rebuild_source() {
	if (!source_dirty) return;

	// 1. Build manifest first — abort if invalid.
	Multinet::WorldScaleInput input{};
	manifest = Multinet::build_world_scale_manifest(input);
	if (!manifest.is_valid()) {
		return;
	}

	// 2. Finalize recipe against the manifest — abort if it fails.
	if (!Multinet::finalize_terrain_recipe(recipe, manifest)) {
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
		recipe, manifest, *executor
	);

	build_source_expectation();

	manifest_built = true;
	source_dirty = false;
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

	Multinet::SurfaceFrame frame;
	frame.origin = pos;
	frame.origin.u_m = 0.0;
	frame.origin.v_m = 0.0;
	frame.origin.altitude_m = 0.0;
	frame.tangent_basis.u_axis  = { 1.0, 0.0, 0.0 };
	frame.tangent_basis.v_axis  = { 0.0, 0.0, 1.0 };
	frame.tangent_basis.up_axis = { 0.0, 1.0, 0.0 };
	frame.frame_epoch = p_frame_epoch;
	frame.topology_version = manifest.topology_version;
	frame.projection_version = manifest.projection_version;

	set_canonical_camera_state(pos, frame, p_frame_epoch);
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
				if (!bccm_renderer.initialize(rs, get_world_3d()->get_scenario(), manifest, recipe.identity, fb)) {
					std::cerr << "[MultinetBCCMNode3D] Renderer initialization failed (process)." << std::endl;
				}
			}

#ifdef DEBUG_ENABLED
			// Editor debug path: direct-plane frustum culling from actual editor viewport camera.
			// IMPORTANT: The PositiveX coordinate mapping below is debug-only scaffolding.
			// It is not the runtime canonical observer and is not the future CHP authority.
			if (manifest_built && editor_view_snapshot.valid) {
				godot::Vector3 cam_pos = editor_view_snapshot.world_position;
				// editor_frame_epoch is stable while the SurfaceFrame is unchanged.
				// It is separate from publication_serial (which increments every frame).
				set_canonical_camera_state_from_values(
					0,
					static_cast<double>(cam_pos.x),
					static_cast<double>(cam_pos.z),
					static_cast<double>(cam_pos.y > 0.0f ? cam_pos.y : 50.0f),
					editor_view_snapshot.editor_frame_epoch
				);

				if (bccm_renderer.initialized() && render_source && current_cam_state.frame_epoch > 0) {
					if (!freeze_update) {
						bccm_renderer.update_with_view(
							editor_view_snapshot.world_position,
							editor_view_snapshot.frustum,
							manifest,
							current_cam_state,
							source_expectation,
							render_source.get()
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
				if (recipe_rebuild_pending) apply_recipe_rebuild();
				if (source_dirty || !render_source) rebuild_source();
				if (!bccm_renderer.initialized() && get_world_3d().is_valid() && manifest_built && render_source) {
					auto* rs = godot::RenderingServer::get_singleton();
					Multinet::TerrainFallbackBounds fb = render_source->get_snapshot().fallback_bounds;
					if (!bccm_renderer.initialize(rs, get_world_3d()->get_scenario(), manifest, recipe.identity, fb)) {
						std::cerr << "[MultinetBCCMNode3D] Renderer initialization failed." << std::endl;
					}
				}

				if (bccm_renderer.initialized() && render_source && current_cam_state.frame_epoch > 0) {
					if (!freeze_update) {
						bccm_renderer.update(cam, manifest, current_cam_state, source_expectation, render_source.get());
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
		if (!bccm_renderer.initialize(rs, get_world_3d()->get_scenario(), manifest, recipe.identity, fb)) {
			std::cerr << "[MultinetBCCMNode3D] Renderer initialization failed (init_rendering)." << std::endl;
		}
	}
}

void MultinetBCCMNode3D::free_rendering() {
	bccm_renderer.cleanup();

	// Shut down the source before the executor, so in-flight jobs can finish.
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

	auto snap = std::make_unique<multinet::rendering::RendererDiagnosticSnapshot>();
	bccm_renderer.get_diagnostic_snapshot(*snap);

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

			if (current_cam_state.frame_epoch == 30 && lod == 0) {
				std::cerr << "[FRAME30-DIAG] lod=" << (int)lod << " i=" << i << " face=" << (int)face << " layer=" << layer << std::endl;
			}
			
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

	return dict;
}

#ifdef DEBUG_ENABLED
void MultinetBCCMNode3D::publish_editor_view_camera(godot::Camera3D* p_editor_camera) {
	if (!Engine::get_singleton()->is_editor_hint() || !p_editor_camera) {
		editor_view_snapshot.valid = false;
		return;
	}

	// Copy position and frustum planes from the actual editor viewport camera.
	// No Camera3D pointer is retained beyond this function.
	editor_view_snapshot.world_position = p_editor_camera->get_global_position();
	editor_view_snapshot.frustum = multinet::rendering::FrustumPlanes::extract_from_camera(p_editor_camera);
	editor_view_snapshot.valid = editor_view_snapshot.frustum.valid;

	if (editor_view_snapshot.valid) {
		// publication_serial: increments every valid editor frame.
		// Never used as frame_epoch — kept separate for bookkeeping only.
		editor_view_snapshot.publication_serial++;

		// editor_frame_epoch: increments only when the SurfaceFrame identity changes.
		// The editor debug path always maps to PositiveX (face index 0).
		// So the epoch is set to 1 on first valid snapshot and stays stable
		// until the face changes (which is debug-only scaffolding and not
		// the runtime canonical observer or future CHP authority).
		constexpr int k_editor_debug_face = 0; // PositiveX — debug scaffolding only
		if (editor_view_snapshot.last_editor_face != k_editor_debug_face) {
			editor_view_snapshot.last_editor_face = k_editor_debug_face;
			editor_view_snapshot.editor_frame_epoch++;
		}
		// If editor_frame_epoch is still 0 (first snapshot), initialise to 1.
		if (editor_view_snapshot.editor_frame_epoch == 0) {
			editor_view_snapshot.editor_frame_epoch = 1;
		}
	}
}
#endif

} // namespace godot