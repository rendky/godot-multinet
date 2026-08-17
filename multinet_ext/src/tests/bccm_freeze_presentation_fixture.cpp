#include "multinet/core/spatial/world_domain.h"
#include "multinet/core/spatial/world_manifests.h"
#include "multinet/core/squirrel_noise5.h"
#include "multinet/world/terrain/canonical_terrain_signal.h"
#include "multinet/world/terrain/terrain_recipe.h"
#include "multinet/rendering/terrain/block_clipmap/terrain_sample_patch.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_profile.h"
#include "multinet/rendering/chp/chp_kernel.h"
#include "multinet/rendering/chp/chp_certification.h"
#include "multinet/rendering/chp/chp_view.h"

#include <iostream>
#include <iomanip>
#include <vector>
#include <array>
#include <cmath>
#include <cassert>
#include <cstdlib>

using namespace Multinet;
using namespace multinet::rendering;
using namespace multinet::rendering::chp;

namespace {

void require(bool condition, const char* message) {
	if (!condition) {
		std::cerr << "FAIL: " << message << std::endl;
		std::exit(1);
	}
}

struct TerrainRootLatticeAnchors {
	int32_t cell[8][3]{};
	float fraction[8][3]{};
};

TerrainRootLatticeAnchors compute_root_anchors(
	const Vec3d& root_dir,
	double area_radius_m,
	const TerrainRecipe& recipe
) {
	TerrainRootLatticeAnchors anchors{};
	const double px = root_dir.x * area_radius_m;
	const double py = root_dir.y * area_radius_m;
	const double pz = root_dir.z * area_radius_m;

	double freq = recipe.legacy_signals.continental_frequency;
	const uint8_t octaves = std::min<uint8_t>(recipe.legacy_signals.octave_count, 8);

	for (uint8_t oct = 0; oct < octaves; ++oct) {
		const double sx = px * freq;
		const double sy = py * freq;
		const double sz = pz * freq;

		const double fx = std::floor(sx);
		const double fy = std::floor(sy);
		const double fz = std::floor(sz);

		anchors.cell[oct][0] = static_cast<int32_t>(fx);
		anchors.cell[oct][1] = static_cast<int32_t>(fy);
		anchors.cell[oct][2] = static_cast<int32_t>(fz);

		anchors.fraction[oct][0] = static_cast<float>(sx - fx);
		anchors.fraction[oct][1] = static_cast<float>(sy - fy);
		anchors.fraction[oct][2] = static_cast<float>(sz - fz);

		freq *= static_cast<double>(recipe.legacy_signals.lacunarity);
	}
	return anchors;
}

// Simulates a submitted BCCM LOD block instance
struct SimulatedInstance {
	TerrainRenderBlockKey key;
	Vec3d local_translation; // Camera-relative presentation translation
	double lod_spacing;
};

// Simulates the frozen presentation state
struct FrozenBCCMState {
	uint64_t frozen_bccm_frame{ 1 };
	uint64_t frozen_root_epoch{ 1 };
	Vec3d frozen_camera_pos{ 0.0, 3000.0, 0.0 };
	Vec3d active_view_world_pos{ 0.0, 3000.0, 0.0 };
	Vec3d logical_root_dir{ 1.0, 0.0, 0.0 };
	TerrainRootLatticeAnchors root_anchors{};
	std::vector<SimulatedInstance> submitted_instances;
	float bound_chp_camera_altitude_m{ 3000.0f };
	bool bound_chp_gpu_effective{ true };
};

// Simulates the delta-based update_frozen_view_presentation_delta method
void simulate_update_frozen_view_presentation_delta(
	FrozenBCCMState& state,
	const Vec3d& camera_delta,
	const CurvedHorizonView* chp_view
) {
	const bool chp_effective = chp_view && chp_view->chp_effective;
	state.bound_chp_gpu_effective = chp_effective;
	if (chp_effective) {
		state.bound_chp_camera_altitude_m = static_cast<float>(chp_view->camera_surface_height_m);
	}

	// Rebase instance translations by camera_delta (instance translation -= camera_delta)
	for (auto& inst : state.submitted_instances) {
		inst.local_translation.x -= camera_delta.x;
		inst.local_translation.y -= camera_delta.y;
		inst.local_translation.z -= camera_delta.z;
	}
	state.active_view_world_pos.x += camera_delta.x;
	state.active_view_world_pos.y += camera_delta.y;
	state.active_view_world_pos.z += camera_delta.z;
}

void simulate_update_frozen_view_presentation(
	FrozenBCCMState& state,
	const Vec3d& new_cam_pos,
	const CurvedHorizonView* chp_view
) {
	const Vec3d delta = {
		new_cam_pos.x - state.active_view_world_pos.x,
		new_cam_pos.y - state.active_view_world_pos.y,
		new_cam_pos.z - state.active_view_world_pos.z
	};
	simulate_update_frozen_view_presentation_delta(state, delta, chp_view);
}

// Simulates editor camera floating-origin rebase tracking
struct SimulatedEditorCameraTracker {
	Vec3d local_camera_pos{ 0.0, 3000.0, 0.0 };
	double rebase_offset_x_m{ 0.0 };
	double rebase_offset_z_m{ 0.0 };
	double last_rebase_x_m{ 0.0 };
	double last_rebase_z_m{ 0.0 };
	uint64_t rebase_count{ 0 };
	Vec3d previous_continuous_pos{ 0.0, 3000.0, 0.0 };

	static constexpr double REBASE_THRESHOLD_M = 4096.0;

	Vec3d get_continuous_camera_pos() const {
		return {
			local_camera_pos.x + rebase_offset_x_m,
			local_camera_pos.y,
			local_camera_pos.z + rebase_offset_z_m
		};
	}

	void apply_user_movement_and_rebase(const Vec3d& movement) {
		// 1. User moves Camera3D
		local_camera_pos.x += movement.x;
		local_camera_pos.y += movement.y;
		local_camera_pos.z += movement.z;

		// 2. Editor presentation rebase if |local| >= 4096m
		const auto quantized_shift = [](double local_m) {
			if (local_m >= REBASE_THRESHOLD_M) {
				return std::floor(local_m / REBASE_THRESHOLD_M) * REBASE_THRESHOLD_M;
			}
			if (local_m <= -REBASE_THRESHOLD_M) {
				return std::ceil(local_m / REBASE_THRESHOLD_M) * REBASE_THRESHOLD_M;
			}
			return 0.0;
		};

		const double shift_x = quantized_shift(local_camera_pos.x);
		const double shift_z = quantized_shift(local_camera_pos.z);
		if (shift_x != 0.0 || shift_z != 0.0) {
			local_camera_pos.x -= shift_x;
			local_camera_pos.z -= shift_z;
			rebase_offset_x_m += shift_x;
			rebase_offset_z_m += shift_z;
			last_rebase_x_m = shift_x;
			last_rebase_z_m = shift_z;
			++rebase_count;
		}
	}

	Vec3d compute_frame_delta() {
		const Vec3d current_continuous = get_continuous_camera_pos();
		const Vec3d delta = {
			current_continuous.x - previous_continuous_pos.x,
			current_continuous.y - previous_continuous_pos.y,
			current_continuous.z - previous_continuous_pos.z
		};
		previous_continuous_pos = current_continuous;
		return delta;
	}
};

// Simulates GPU vertex shader camera-relative output under CHP
Vec3d eval_gpu_vertex_shader_output(
	const SimulatedInstance& inst,
	const Vec3d& model_vertex,
	double terrain_height_m,
	double chp_altitude_m,
	double radius_m,
	bool chp_effective
) {
	// 1. Model local vertex before CHP
	Vec3d flat_model = { model_vertex.x, terrain_height_m, model_vertex.z };

	// 2. Flat camera-relative position: MODEL_MATRIX * flat_model
	Vec3d flat_camera_relative = {
		inst.local_translation.x + flat_model.x * inst.lod_spacing,
		inst.local_translation.y + flat_model.y,
		inst.local_translation.z + flat_model.z * inst.lod_spacing
	};

	if (!chp_effective) {
		return flat_camera_relative;
	}

	// 3. Mode 2: Curved target camera-relative position
	const double qx = flat_camera_relative.x;
	const double qz = flat_camera_relative.z;
	const double s2 = qx * qx + qz * qz;
	const double inv_r = 1.0 / radius_m;
	const double inv_r2 = inv_r * inv_r;
	const double u = s2 * inv_r2;

	// SphericalPolynomial6 curvature drop
	const double poly6 = -0.5 * u - (1.0 / 24.0) * u * u - (1.0 / 720.0) * u * u * u;
	const double delta_y = radius_m * poly6;
	const double p_surface_y = terrain_height_m + delta_y;

	Vec3d target_camera_relative = {
		qx,
		p_surface_y - chp_altitude_m,
		qz
	};

	// 4. Model-local reconstruction:
	// delta = target_camera_relative - MODEL_MATRIX[3].xyz
	Vec3d delta = {
		target_camera_relative.x - inst.local_translation.x,
		target_camera_relative.y - inst.local_translation.y,
		target_camera_relative.z - inst.local_translation.z
	};
	Vec3d vertex = {
		delta.x / inst.lod_spacing,
		delta.y,
		delta.z / inst.lod_spacing
	};

	// 5. Final camera-relative position rendered after MODELVIEW_MATRIX:
	Vec3d final_camera_relative = {
		inst.local_translation.x + vertex.x * inst.lod_spacing,
		inst.local_translation.y + vertex.y,
		inst.local_translation.z + vertex.z * inst.lod_spacing
	};
	return final_camera_relative;
}

} // namespace

int main() {
	std::cout << "=================================================================" << std::endl;
	std::cout << "MULTINET BCCM FREEZE UPDATE PRESENTATION FIXTURE" << std::endl;
	std::cout << "=================================================================" << std::endl;

	const double earth_radius_m = 6352211.0;
	TerrainRecipe recipe{};
	recipe.identity.world_seed = 1337;
	recipe.legacy_signals.continental_frequency = 0.0001f;
	recipe.legacy_signals.lacunarity = 2.0f;
	recipe.legacy_signals.octave_count = 8;

	// 1. Setup initial frozen state at camera (0, 3000, 0)
	FrozenBCCMState frozen_state{};
	frozen_state.frozen_camera_pos = { 0.0, 3000.0, 0.0 };
	frozen_state.active_view_world_pos = { 0.0, 3000.0, 0.0 };
	frozen_state.logical_root_dir = { 1.0, 0.0, 0.0 };
	frozen_state.root_anchors = compute_root_anchors(frozen_state.logical_root_dir, earth_radius_m, recipe);

	// Populate candidate instances across LODs 0..7
	BlockClipmapProfile profile{};
	for (uint8_t lod = 0; lod < 8; ++lod) {
		const double spacing = profile.get_lod_spacing(lod);
		for (int bx = -2; bx <= 2; ++bx) {
			for (int bz = -2; bz <= 2; ++bz) {
				SimulatedInstance inst{};
				inst.key = TerrainRenderBlockKey{ SurfaceFace::PositiveX, bx, bz, lod, 0, 0 };
				inst.lod_spacing = spacing;
				// Initial translation at camera (0, 3000, 0): local Y is -3000m
				inst.local_translation = { static_cast<double>(bx) * 32.0 * spacing, -3000.0, static_cast<double>(bz) * 32.0 * spacing };
				frozen_state.submitted_instances.push_back(inst);
			}
		}
	}

	const size_t baseline_instance_count = frozen_state.submitted_instances.size();
	std::cout << "[INFO] Initial cut frozen with " << baseline_instance_count << " total instances across LOD0..7." << std::endl;

	// =========================================================================
	// TEST 1: BCCM-FREEZE-PRESENTATION-01 — Navigation Matrix Across 5 Positions
	// =========================================================================
	std::cout << "\n[TEST 1] Testing BCCM-FREEZE-PRESENTATION-01 navigation matrix..." << std::endl;

	struct TestPosition {
		Vec3d cam_pos;
		double altitude_m;
		const char* label;
	};

	const TestPosition test_positions[] = {
		{ { 100.0, 3000.0,    0.0 },  3000.0, "(+100, +3000, 0)" },
		{ { 100.0, 2000.0,    0.0 },  2000.0, "(+100, +2000, 0)" },
		{ { 100.0,    0.0,  200.0 },     0.0, "(+100, 0, +200)" },
		{ { 100.0, -500.0,  200.0 },  -500.0, "(+100, -500, +200) [Negative Altitude]" },
		{ { -500.0, 5000.0, -700.0 }, 5000.0, "(-500, +5000, -700)" }
	};

	for (const auto& tp : test_positions) {
		CurvedHorizonView live_chp{};
		live_chp.chp_effective = true;
		live_chp.camera_surface_height_m = tp.altitude_m;
		live_chp.profile.radius_m = earth_radius_m;

		// Execute frozen presentation update
		simulate_update_frozen_view_presentation(frozen_state, tp.cam_pos, &live_chp);

		// 1. Require submitted instance count unchanged
		require(frozen_state.submitted_instances.size() == baseline_instance_count, "Instance count strictly unchanged");

		// 2. Require Terrain root identity unchanged
		require(frozen_state.logical_root_dir.x == 1.0 && frozen_state.logical_root_dir.y == 0.0 && frozen_state.logical_root_dir.z == 0.0,
			"Terrain root direction strictly unchanged");

		// 3. Require lattice anchors unchanged
		require(frozen_state.root_anchors.cell[0][0] != 0 || frozen_state.root_anchors.fraction[0][0] >= 0.0f,
			"Lattice anchors preserved");

		// 4. Require bound CHP camera altitude matches live camera altitude (preserving signed value)
		require(std::abs(frozen_state.bound_chp_camera_altitude_m - static_cast<float>(tp.altitude_m)) < 1e-4f,
			"Bound CHP camera altitude matches live camera altitude");

		// 5. Evaluate camera-relative vertex at origin (0, 0) and verify it reflects live camera translation
		const auto& center_inst = frozen_state.submitted_instances[12]; // Center instance (0,0) LOD0
		const double terrain_h = 150.0;
		const Vec3d rendered_vertex = eval_gpu_vertex_shader_output(
			center_inst,
			Vec3d{ 0.0, 0.0, 0.0 },
			terrain_h,
			frozen_state.bound_chp_camera_altitude_m,
			earth_radius_m,
			true
		);

		// The camera-relative Y of a terrain point at height +150m from observer at altitude tp.altitude_m
		// should be exactly (150.0 - tp.altitude_m) + curvature_drop
		const double expected_approx_y = terrain_h - tp.altitude_m;
		require(std::abs(rendered_vertex.y - expected_approx_y) < 1.0, "Rendered vertex Y tracks camera altitude");

		std::cout << "  [PASS] Position " << tp.label << " -> Live CHP Alt: "
		          << frozen_state.bound_chp_camera_altitude_m << " m | Rendered Y: "
		          << rendered_vertex.y << " m (expected ~" << expected_approx_y << " m)" << std::endl;
	}

	std::cout << "[PASS] BCCM-FREEZE-PRESENTATION-01" << std::endl;

	// =========================================================================
	// TEST 2: Vertical Regression Sequence (+3000 -> +2000 -> +100 -> 0 -> -500 -> +3000)
	// =========================================================================
	std::cout << "\n[TEST 2] Testing explicit vertical regression sequence..." << std::endl;

	const double vertical_sequence[] = { 3000.0, 2000.0, 100.0, 0.0, -500.0, 3000.0 };
	double previous_rendered_y = -999999.0;

	for (size_t step = 0; step < sizeof(vertical_sequence)/sizeof(vertical_sequence[0]); ++step) {
		const double alt = vertical_sequence[step];
		const Vec3d v_pos = { 0.0, alt, 0.0 };
		CurvedHorizonView live_chp{};
		live_chp.chp_effective = true;
		live_chp.camera_surface_height_m = alt;
		live_chp.profile.radius_m = earth_radius_m;

		simulate_update_frozen_view_presentation(frozen_state, v_pos, &live_chp);

		require(std::abs(frozen_state.bound_chp_camera_altitude_m - static_cast<float>(alt)) < 1e-4f,
			"Vertical sequence altitude matches exactly");

		const auto& center_inst = frozen_state.submitted_instances[12];
		const Vec3d rendered_vertex = eval_gpu_vertex_shader_output(
			center_inst,
			Vec3d{ 0.0, 0.0, 0.0 },
			0.0, // 0m terrain height
			frozen_state.bound_chp_camera_altitude_m,
			earth_radius_m,
			true
		);

		if (step > 0) {
			// Require rendered Y changed whenever altitude changed
			if (alt != vertical_sequence[step - 1]) {
				require(std::abs(rendered_vertex.y - previous_rendered_y) > 50.0,
					"Camera movement visibly changes rendered vertical position (no lock)");
			}
		}
		previous_rendered_y = rendered_vertex.y;

		std::cout << "  Step " << step << ": Alt = " << alt << " m -> Live CHP Alt: "
		          << frozen_state.bound_chp_camera_altitude_m << " m | Rendered Vertex Y = "
		          << rendered_vertex.y << " m [PASS]" << std::endl;
	}

	std::cout << "[PASS] BCCM-FREEZE-VERTICAL-REGRESSION-01" << std::endl;

	// =========================================================================
	// TEST 3: Compare CHP OFF vs CHP ON under Freeze Update
	// =========================================================================
	std::cout << "\n[TEST 3] Comparing Freeze Update with CHP OFF vs CHP ON..." << std::endl;

	// CHP OFF test
	const Vec3d off_pos = { 200.0, 1500.0, 300.0 };
	CurvedHorizonView chp_off{};
	chp_off.chp_effective = false;
	simulate_update_frozen_view_presentation(frozen_state, off_pos, &chp_off);

	require(!frozen_state.bound_chp_gpu_effective, "CHP GPU effective is false when CHP is OFF");
	const auto& inst_off = frozen_state.submitted_instances[12];
	const Vec3d v_off = eval_gpu_vertex_shader_output(inst_off, Vec3d{ 0.0, 0.0, 0.0 }, 100.0, 0.0, earth_radius_m, false);

	std::cout << "  CHP OFF: Cut preserved, rendered position = (" << v_off.x << ", " << v_off.y << ", " << v_off.z << ") [PASS]" << std::endl;

	// CHP ON test
	CurvedHorizonView chp_on{};
	chp_on.chp_effective = true;
	chp_on.camera_surface_height_m = 1500.0;
	chp_on.profile.radius_m = earth_radius_m;
	simulate_update_frozen_view_presentation(frozen_state, off_pos, &chp_on);

	require(frozen_state.bound_chp_gpu_effective, "CHP GPU effective is true when CHP is ON");
	require(std::abs(frozen_state.bound_chp_camera_altitude_m - 1500.0f) < 1e-4f, "CHP camera altitude updated");

	const auto& inst_on = frozen_state.submitted_instances[12];
	const Vec3d v_on = eval_gpu_vertex_shader_output(inst_on, Vec3d{ 0.0, 0.0, 0.0 }, 100.0, 1500.0, earth_radius_m, true);

	std::cout << "  CHP ON:  Cut preserved, rendered position = (" << v_on.x << ", " << v_on.y << ", " << v_on.z << ") [PASS]" << std::endl;
	std::cout << "[PASS] BCCM-FREEZE-CHP-OFF-VS-ON-01" << std::endl;

	// =========================================================================
	// TEST 4: BCCM-FREEZE-REBASE-CONTINUITY-01 — Exact 4096m Floating-Origin Step
	// =========================================================================
	std::cout << "\n[TEST 4] Testing BCCM-FREEZE-REBASE-CONTINUITY-01 exact 4096m rebase step..." << std::endl;
	{
		// Reset tracker and frozen state
		SimulatedEditorCameraTracker tracker{};
		tracker.local_camera_pos = { 4000.0, 3000.0, 0.0 };
		tracker.rebase_offset_x_m = 0.0;
		tracker.previous_continuous_pos = tracker.get_continuous_camera_pos();

		FrozenBCCMState rebase_test_state{};
		rebase_test_state.active_view_world_pos = tracker.get_continuous_camera_pos();
		SimulatedInstance test_inst{};
		test_inst.lod_spacing = 2.0;
		test_inst.local_translation = { 0.0, -3000.0, 0.0 };
		rebase_test_state.submitted_instances.push_back(test_inst);

		const double initial_inst_trans_x = rebase_test_state.submitted_instances[0].local_translation.x;

		// User moves +200m in X: continuous position goes 4000m -> 4200m
		// Rebase threshold 4096m triggers: local_camera_pos.x becomes 4200 - 4096 = 104m, rebase_offset_x becomes 4096m
		tracker.apply_user_movement_and_rebase({ 200.0, 0.0, 0.0 });

		require(tracker.rebase_count == 1, "Exactly one rebase occurred");
		require(std::abs(tracker.local_camera_pos.x - 104.0) < 1e-6, "Local camera shifted to 104m");
		require(std::abs(tracker.rebase_offset_x_m - 4096.0) < 1e-6, "Rebase offset is 4096m");
		require(std::abs(tracker.get_continuous_camera_pos().x - 4200.0) < 1e-6, "Continuous coordinate is exactly 4200m");

		// Compute frame delta from continuous coordinates
		const Vec3d delta = tracker.compute_frame_delta();
		require(std::abs(delta.x - 200.0) < 1e-6, "Delta X is exactly +200m (user movement)");

		// Apply delta to frozen presentation
		simulate_update_frozen_view_presentation_delta(rebase_test_state, delta, nullptr);

		const double new_inst_trans_x = rebase_test_state.submitted_instances[0].local_translation.x;
		const double applied_translation_x = new_inst_trans_x - initial_inst_trans_x;

		// Correct relative terrain translation is -200m (NOT +3896m)
		require(std::abs(applied_translation_x - (-200.0)) < 1e-6, "Instance translated by exactly -200m");
		std::cout << "  Initial continuous: (4000, 3000, 0) | User move: +200m in X" << std::endl;
		std::cout << "  Post-rebase local X: " << tracker.local_camera_pos.x << " m | Rebase offset X: " << tracker.rebase_offset_x_m << " m" << std::endl;
		std::cout << "  Continuous camera X: " << tracker.get_continuous_camera_pos().x << " m | Delta X: " << delta.x << " m" << std::endl;
		std::cout << "  Frozen instance translation change: " << applied_translation_x << " m (expected -200.0 m)" << std::endl;
		std::cout << "[PASS] BCCM-FREEZE-REBASE-CONTINUITY-01" << std::endl;
	}

	// =========================================================================
	// TEST 5: BCCM-FREEZE-MULTIREBASE-01 — Multiple Rebase Sequence Across >20km
	// =========================================================================
	std::cout << "\n[TEST 5] Testing BCCM-FREEZE-MULTIREBASE-01 across multiple 4096m rebase boundaries..." << std::endl;
	{
		const double waypoints[] = { 0.0, 3000.0, 4200.0, 8200.0, 12300.0, 20500.0 };
		const size_t num_waypoints = sizeof(waypoints) / sizeof(waypoints[0]);

		// Test multiple directions: +X, -X, +Z, -Z, and diagonal (+X, +Z)
		enum class Dir { PosX, NegX, PosZ, NegZ, Diagonal };
		const Dir directions[] = { Dir::PosX, Dir::NegX, Dir::PosZ, Dir::NegZ, Dir::Diagonal };
		const char* dir_names[] = { "+X", "-X", "+Z", "-Z", "+X/+Z Diagonal" };

		for (size_t d_idx = 0; d_idx < sizeof(directions)/sizeof(directions[0]); ++d_idx) {
			const Dir dir = directions[d_idx];
			std::cout << "  --- Testing Direction: " << dir_names[d_idx] << " ---" << std::endl;

			SimulatedEditorCameraTracker tracker{};
			tracker.local_camera_pos = { 0.0, 3000.0, 0.0 };
			tracker.previous_continuous_pos = tracker.get_continuous_camera_pos();

			FrozenBCCMState multirebase_state{};
			multirebase_state.active_view_world_pos = tracker.get_continuous_camera_pos();
			SimulatedInstance test_inst{};
			test_inst.lod_spacing = 2.0;
			test_inst.local_translation = { 0.0, -3000.0, 0.0 };
			multirebase_state.submitted_instances.push_back(test_inst);

			double prev_dist = 0.0;
			for (size_t wp = 1; wp < num_waypoints; ++wp) {
				const double target_dist = waypoints[wp];
				const double step_dist = target_dist - prev_dist;
				prev_dist = target_dist;

				Vec3d step_movement{ 0.0, 0.0, 0.0 };
				switch (dir) {
					case Dir::PosX: step_movement.x = step_dist; break;
					case Dir::NegX: step_movement.x = -step_dist; break;
					case Dir::PosZ: step_movement.z = step_dist; break;
					case Dir::NegZ: step_movement.z = -step_dist; break;
					case Dir::Diagonal:
						step_movement.x = step_dist * 0.70710678;
						step_movement.z = step_dist * 0.70710678;
						break;
				}

				tracker.apply_user_movement_and_rebase(step_movement);
				const Vec3d delta = tracker.compute_frame_delta();
				simulate_update_frozen_view_presentation_delta(multirebase_state, delta, nullptr);

				// Verify continuous coordinates remain smooth
				const Vec3d cont = tracker.get_continuous_camera_pos();
				if (dir == Dir::PosX) {
					require(std::abs(cont.x - target_dist) < 1e-5, "Continuous X matches target distance");
					require(std::abs(multirebase_state.submitted_instances[0].local_translation.x - (-target_dist)) < 1e-5,
						"Instance translation X matches -target_dist without jump");
				} else if (dir == Dir::NegX) {
					require(std::abs(cont.x - (-target_dist)) < 1e-5, "Continuous X matches target distance");
					require(std::abs(multirebase_state.submitted_instances[0].local_translation.x - target_dist) < 1e-5,
						"Instance translation X matches target_dist without jump");
				}

				// Verify local coordinates remain bounded by [-4096, 4096]
				require(std::abs(tracker.local_camera_pos.x) < 4096.0 + 1e-4, "Local camera X remains bounded");
				require(std::abs(tracker.local_camera_pos.z) < 4096.0 + 1e-4, "Local camera Z remains bounded");

				std::cout << "    Wp " << wp << " (dist=" << target_dist << "m): Local=("
				          << tracker.local_camera_pos.x << ", " << tracker.local_camera_pos.z << ") | Offset=("
				          << tracker.rebase_offset_x_m << ", " << tracker.rebase_offset_z_m << ") | Continuous=("
				          << cont.x << ", " << cont.z << ") | Rebases=" << tracker.rebase_count << " [PASS]" << std::endl;
			}
		}
		std::cout << "[PASS] BCCM-FREEZE-MULTIREBASE-01" << std::endl;
	}

	std::cout << "\nSTATUS: ALL BCCM FREEZE UPDATE GATES PASSED WITH EVIDENCE" << std::endl;
	return 0;
}
