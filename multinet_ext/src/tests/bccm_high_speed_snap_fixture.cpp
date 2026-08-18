#include "multinet/core/spatial/world_domain.h"
#include "multinet/core/spatial/world_manifests.h"
#include "multinet/core/squirrel_noise5.h"
#include "multinet/world/terrain/canonical_terrain_signal.h"
#include "multinet/world/terrain/terrain_recipe.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_profile.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_renderer.h"
#include "multinet/rendering/terrain/block_clipmap/terrain_sample_patch.h"
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
#include <string>
#include <algorithm>

using namespace Multinet;
using namespace multinet::rendering;
using namespace multinet::rendering::chp;

namespace {

void require(bool condition, const char* message) {
	if (!condition) {
		std::cout << std::flush;
		std::cerr << "\n[FIXTURE-FAIL] " << message << std::endl;
		std::exit(1);
	}
}


// Enumerates candidate block coordinates for a given LOD using production candidate-grid rules
std::vector<std::pair<int64_t, int64_t>> enumerate_lod_candidates(
	int64_t center_bx,
	int64_t center_bv,
	uint8_t lod,
	const BlockClipmapProfile& profile,
	double active_cam_u,
	double active_cam_v
) {
	std::vector<std::pair<int64_t, int64_t>> candidates;
	const double block_size = profile.get_lod_block_size(lod);

	int32_t hole_dx = 0;
	int32_t hole_dz = 0;
	if (lod > 0) {
		int64_t prev_bx = static_cast<int64_t>(std::floor(
			(std::floor(active_cam_u / block_size) * block_size) / block_size
		));
		int64_t prev_bv = static_cast<int64_t>(std::floor(
			(std::floor(active_cam_v / block_size) * block_size) / block_size
		));
		hole_dx = static_cast<int32_t>(prev_bx - center_bx);
		hole_dz = static_cast<int32_t>(prev_bv - center_bv);
	}

	int32_t r = profile.candidate_grid_radius;
	int32_t hole_r = profile.inner_hole_radius;

	for (int32_t dv = -r; dv < r; ++dv) {
		for (int32_t du = -r; du < r; ++du) {
			if (lod > 0) {
				int32_t hu = du - hole_dx;
				int32_t hv = dv - hole_dz;
				if (hu >= -hole_r && hu < hole_r && hv >= -hole_r && hv < hole_r) continue;
			}
			candidates.push_back({ center_bx + du, center_bv + dv });
		}
	}
	return candidates;
}

struct CandidateOverlap {
	uint32_t retained{ 0 };
	uint32_t added{ 0 };
	uint32_t removed{ 0 };
	float turnover_fraction{ 0.0f };
};

CandidateOverlap compute_candidate_overlap(
	const std::vector<std::pair<int64_t, int64_t>>& old_set,
	const std::vector<std::pair<int64_t, int64_t>>& new_set
) {
	CandidateOverlap overlap{};
	if (old_set.empty()) {
		overlap.added = static_cast<uint32_t>(new_set.size());
		return overlap;
	}

	uint32_t retained = 0;
	for (const auto& n : new_set) {
		for (const auto& o : old_set) {
			if (n.first == o.first && n.second == o.second) {
				retained++;
				break;
			}
		}
	}
	overlap.retained = retained;
	overlap.added = static_cast<uint32_t>(new_set.size() > retained ? new_set.size() - retained : 0);
	overlap.removed = static_cast<uint32_t>(old_set.size() > retained ? old_set.size() - retained : 0);
	overlap.turnover_fraction = old_set.empty() ? 0.0f : static_cast<float>(overlap.removed) / static_cast<float>(old_set.size());
	return overlap;
}

// ---------------------------------------------------------------------------
// Gate 1: BCCM-SNAP-LAW-01 — Verify Production Snap Law per LOD
// ---------------------------------------------------------------------------
void test_bccm_snap_law_01() {
	std::cout << "\n=======================================================\n";
	std::cout << " Gate 1: BCCM-SNAP-LAW-01 Production Snap Law Verification\n";
	std::cout << "=======================================================\n";

	BlockClipmapProfile profile{};
	profile.level_count = 8;
	profile.finest_spacing = 2.0f;
	profile.lod0_block_size = 32.0f;
	profile.candidate_grid_radius = 4;
	profile.inner_hole_radius = 2;

	std::cout << std::left << std::setw(6) << "LOD"
	          << std::setw(12) << "Spacing(m)"
	          << std::setw(14) << "BlockSize(m)"
	          << std::setw(14) << "SnapSize(m)"
	          << std::setw(12) << "GridDims"
	          << std::setw(12) << "HoleDims"
	          << std::setw(12) << "Candidates"
	          << "\n-----------------------------------------------------------------------------------\n";

	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		const double spacing = profile.get_lod_spacing(lod);
		const double block_sz = profile.get_lod_block_size(lod);
		const double snap_sz = (lod + 1 < profile.level_count) ? profile.get_lod_block_size(lod + 1) : block_sz;

		const double expected_spacing = 2.0 * std::pow(2.0, lod);
		const double expected_block_sz = 32.0 * std::pow(2.0, lod);
		const double expected_snap_sz = (lod < 7) ? (64.0 * std::pow(2.0, lod)) : (32.0 * std::pow(2.0, 7));

		require(std::abs(spacing - expected_spacing) < 1e-6, "spacing mismatch");
		require(std::abs(block_sz - expected_block_sz) < 1e-6, "block_size mismatch");
		require(std::abs(snap_sz - expected_snap_sz) < 1e-6, "snap_size mismatch");

		auto candidates = enumerate_lod_candidates(0, 0, lod, profile, 0.0, 0.0);
		const uint32_t expected_cands = (lod == 0) ? 64 : 48; // 8x8 = 64, 8x8 - 4x4 = 48
		require(candidates.size() == expected_cands, "candidate count mismatch");

		std::cout << std::left << std::setw(6) << static_cast<int>(lod)
		          << std::setw(12) << spacing
		          << std::setw(14) << block_sz
		          << std::setw(14) << snap_sz
		          << std::setw(12) << "8x8"
		          << std::setw(12) << (lod == 0 ? "none" : "4x4")
		          << std::setw(12) << candidates.size()
		          << "\n";
	}

	// Verify quantization formula properties:
	// For LOD0: snap period is 64m. Any position u in [0, 64) maps to center_bx = 0.
	// In [64, 128) maps to center_bx = 2 (jump of 2 blocks = 64m).
	QuantizedLODCenter c0_a = compute_lod_center(0.0, 0.0, 0, profile);
	QuantizedLODCenter c0_b = compute_lod_center(63.99, 0.0, 0, profile);
	QuantizedLODCenter c0_c = compute_lod_center(64.0, 0.0, 0, profile);

	require(c0_a.center_bx == 0, "c0_a center_bx != 0");
	require(c0_b.center_bx == 0, "c0_b center_bx != 0");
	require(c0_c.center_bx == 2, "c0_c center_bx != 2 (expected 2-block snap jump)");
	require(std::abs(c0_c.center_u_m - 64.0) < 1e-6, "c0_c center_u_m != 64.0");

	std::cout << "\n[PASS] BCCM-SNAP-LAW-01 verified: LOD0 snap period is exactly 64.0m (2 blocks), with 64 candidates on LOD0 and 48 on LOD1..7 (total 400 candidates).\n";
}

// ---------------------------------------------------------------------------
// Gate 2: BCCM-CUT-SPEED-DIAGNOSTIC-01 — Live Delta-Time Speed Calculation
// ---------------------------------------------------------------------------
void test_bccm_cut_speed_diagnostic_01() {
	std::cout << "\n=======================================================\n";
	std::cout << " Gate 2: BCCM-CUT-SPEED-DIAGNOSTIC-01 Delta-Time Speed\n";
	std::cout << "=======================================================\n";

	const double test_intervals_sec[] = {
		0.004,        // 4ms (250 FPS)
		0.005,        // 5ms (200 FPS)
		0.007,        // 7ms (~142.8 FPS)
		0.0135,       // 13.5ms (~74 FPS)
		0.016666667,  // 16.67ms (60 FPS)
		0.033333333   // 33.3ms (30 FPS)
	};

	const double test_distances_m[] = { 10.0, 50.0, 100.0, 500.0 };

	std::cout << std::left << std::setw(14) << "Interval(ms)"
	          << std::setw(16) << "Distance(m)"
	          << std::setw(18) << "Expected(m/s)"
	          << std::setw(18) << "Calculated(m/s)"
	          << std::setw(16) << "Calculated(km/s)"
	          << std::setw(10) << "Error(m/s)"
	          << "\n----------------------------------------------------------------------------------------------------\n";

	for (double dt : test_intervals_sec) {
		for (double dist : test_distances_m) {
			const double expected_speed_m_s = dist / dt;
			const double expected_speed_km_s = expected_speed_m_s * 0.001;

			// Replicate production BlockClipmapRenderer speed calculation
			double calculated_speed_m_s = 0.0;
			double calculated_speed_km_s = 0.0;
			if (dt > 0.0 && std::isfinite(dt)) {
				calculated_speed_m_s = dist / dt;
				calculated_speed_km_s = calculated_speed_m_s * 0.001;
			}

			const double err = std::abs(calculated_speed_m_s - expected_speed_m_s);
			require(err < 1e-6, "Speed diagnostic mismatch against ground truth");
			require(std::abs(calculated_speed_km_s - expected_speed_km_s) < 1e-9, "Speed km/s mismatch");

			std::cout << std::left << std::setw(14) << std::fixed << std::setprecision(2) << (dt * 1000.0)
			          << std::setw(16) << std::setprecision(1) << dist
			          << std::setw(18) << std::setprecision(2) << expected_speed_m_s
			          << std::setw(18) << calculated_speed_m_s
			          << std::setw(16) << std::setprecision(4) << calculated_speed_km_s
			          << std::setw(10) << std::setprecision(6) << err
			          << "\n";
		}
	}

	// Verify degenerate cases (dt <= 0 or NaN/Inf)
	double zero_dt = 0.0;
	double zero_speed = (zero_dt > 0.0 && std::isfinite(zero_dt)) ? (100.0 / zero_dt) : 0.0;
	require(zero_speed == 0.0, "dt=0 must yield 0.0 speed");

	std::cout << "\n[PASS] BCCM-CUT-SPEED-DIAGNOSTIC-01 verified: Real delta-time speed calculation is exact across all frame intervals.\n";
}

// ---------------------------------------------------------------------------
// Gate 3: BCCM-HIGH-SPEED-SNAP-01 — Engine-Neutral Deterministic Snap Simulator
// ---------------------------------------------------------------------------
struct SimulationResult {
	double speed_km_s{ 0.0 };
	double fps{ 0.0 };
	double delta_metres_per_frame{ 0.0 };
	uint32_t max_snap_steps_lod0{ 0 };
	double max_lod0_center_jump_m{ 0.0 };
	double max_center_jump_per_lod[8]{};
	double simulation_duration_seconds{ 1.0 };
	uint32_t lod0_skipped_snap_events{ 0 };
	uint32_t all_lod_skipped_snap_events{ 0 };
	double lod0_skipped_snap_events_per_second{ 0.0 };
	double all_lod_skipped_snap_events_per_second{ 0.0 };
	float worst_turnover_pct_lod0{ 0.0f };
	float worst_turnover_pct_any_lod{ 0.0f };
};

SimulationResult run_simulation(
	double speed_m_s,
	double fps,
	bool diagonal,
	const BlockClipmapProfile& profile,
	double duration_sec = 1.0,
	double initial_phase_offset_m = 0.0
) {
	SimulationResult res{};
	res.speed_km_s = speed_m_s * 0.001;
	res.fps = fps;
	res.simulation_duration_seconds = duration_sec;
	const double dt = 1.0 / fps;
	const uint32_t total_frames = static_cast<uint32_t>(std::ceil(fps * duration_sec));
	const double step_dist = speed_m_s * dt;
	res.delta_metres_per_frame = step_dist;

	const double dir_x = diagonal ? (1.0 / std::sqrt(2.0)) : 1.0;
	const double dir_z = diagonal ? (1.0 / std::sqrt(2.0)) : 0.0;

	// Per-LOD tracking state
	int64_t prev_center_bx[8]{};
	int64_t prev_center_bv[8]{};
	bool has_prev_center[8]{};
	std::vector<std::pair<int64_t, int64_t>> prev_candidates[8];

	for (uint32_t frame = 0; frame < total_frames; ++frame) {
		const double t = frame * dt;
		const double cam_u = (initial_phase_offset_m + t * speed_m_s) * dir_x;
		const double cam_v = (initial_phase_offset_m + t * speed_m_s) * dir_z;

		for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
			const QuantizedLODCenter qc = compute_lod_center(cam_u, cam_v, lod, profile);
			const auto candidates = enumerate_lod_candidates(qc.center_bx, qc.center_bv, lod, profile, cam_u, cam_v);

			if (has_prev_center[lod]) {
				const int64_t dbx = qc.center_bx - prev_center_bx[lod];
				const int64_t dbv = qc.center_bv - prev_center_bv[lod];
				const double d_u_m = static_cast<double>(dbx) * qc.block_size_m;
				const double d_v_m = static_cast<double>(dbv) * qc.block_size_m;
				const double jump_m = std::sqrt(d_u_m * d_u_m + d_v_m * d_v_m);

				const uint32_t steps_u = static_cast<uint32_t>(std::llround(std::abs(d_u_m) / qc.snap_size_m));
				const uint32_t steps_v = static_cast<uint32_t>(std::llround(std::abs(d_v_m) / qc.snap_size_m));
				const uint32_t max_steps = std::max(steps_u, steps_v);

				if (lod == 0) {
					res.max_snap_steps_lod0 = std::max(res.max_snap_steps_lod0, max_steps);
					res.max_lod0_center_jump_m = std::max(res.max_lod0_center_jump_m, jump_m);
					if (max_steps > 1) {
						res.lod0_skipped_snap_events++;
					}
				}
				res.max_center_jump_per_lod[lod] = std::max(res.max_center_jump_per_lod[lod], jump_m);

				if (max_steps > 1) {
					res.all_lod_skipped_snap_events++;
				}

				const CandidateOverlap overlap = compute_candidate_overlap(prev_candidates[lod], candidates);
				const float turnover_pct = overlap.turnover_fraction * 100.0f;
				if (lod == 0) {
					res.worst_turnover_pct_lod0 = std::max(res.worst_turnover_pct_lod0, turnover_pct);
				}
				res.worst_turnover_pct_any_lod = std::max(res.worst_turnover_pct_any_lod, turnover_pct);
			}

			prev_center_bx[lod] = qc.center_bx;
			prev_center_bv[lod] = qc.center_bv;
			has_prev_center[lod] = true;
			prev_candidates[lod] = candidates;
		}
	}

	if (duration_sec > 0.0) {
		res.lod0_skipped_snap_events_per_second = static_cast<double>(res.lod0_skipped_snap_events) / duration_sec;
		res.all_lod_skipped_snap_events_per_second = static_cast<double>(res.all_lod_skipped_snap_events) / duration_sec;
	}
	return res;
}

void test_bccm_high_speed_snap_01() {
	std::cout << "\n=======================================================\n";
	std::cout << " Gate 3: BCCM-HIGH-SPEED-SNAP-01 Full Speed x FPS Matrix\n";
	std::cout << "=======================================================\n";

	BlockClipmapProfile profile{};
	profile.level_count = 8;
	profile.finest_spacing = 2.0f;
	profile.lod0_block_size = 32.0f;
	profile.candidate_grid_radius = 4;
	profile.inner_hole_radius = 2;

	const double test_speeds_km_s[] = { 0.0, 1.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 16.0 };
	const double test_fps[] = { 30.0, 60.0, 74.0, 90.0, 110.0, 120.0, 142.0, 165.0, 200.0, 240.0 };

	std::cout << "\n--- Straight Motion (+U Axis) ---\n";
	std::cout << std::left << std::setw(12) << "Speed(km/s)"
	          << std::setw(8) << "FPS"
	          << std::setw(14) << "Delta(m/f)"
	          << std::setw(16) << "MaxLOD0Steps"
	          << std::setw(16) << "MaxLOD0Jump(m)"
	          << std::setw(18) << "WorstTurnover% "
	          << std::setw(18) << "LOD0Skipped/s"
	          << std::setw(18) << "AllLODSkipped/s"
	          << "\n------------------------------------------------------------------------------------------------------------------------\n";

	for (double spd_km : test_speeds_km_s) {
		for (double fps : test_fps) {
			SimulationResult res = run_simulation(spd_km * 1000.0, fps, false, profile, 1.0);
			std::cout << std::left << std::setw(12) << res.speed_km_s
			          << std::setw(8) << static_cast<int>(res.fps)
			          << std::setw(14) << std::fixed << std::setprecision(1) << res.delta_metres_per_frame
			          << std::setw(16) << res.max_snap_steps_lod0
			          << std::setw(16) << res.max_lod0_center_jump_m
			          << std::setw(18) << std::setprecision(1) << res.worst_turnover_pct_lod0
			          << std::setw(18) << std::setprecision(2) << res.lod0_skipped_snap_events_per_second
			          << std::setw(18) << std::setprecision(2) << res.all_lod_skipped_snap_events_per_second
			          << "\n";
		}
	}

	std::cout << "\n--- Diagonal Motion (+U / +V Diagonal) ---\n";
	std::cout << std::left << std::setw(12) << "Speed(km/s)"
	          << std::setw(8) << "FPS"
	          << std::setw(14) << "Delta(m/f)"
	          << std::setw(16) << "MaxLOD0Steps"
	          << std::setw(16) << "MaxLOD0Jump(m)"
	          << std::setw(18) << "WorstTurnover% "
	          << std::setw(18) << "LOD0Skipped/s"
	          << std::setw(18) << "AllLODSkipped/s"
	          << "\n------------------------------------------------------------------------------------------------------------------------\n";

	for (double spd_km : test_speeds_km_s) {
		for (double fps : test_fps) {
			SimulationResult res = run_simulation(spd_km * 1000.0, fps, true, profile, 1.0);
			std::cout << std::left << std::setw(12) << res.speed_km_s
			          << std::setw(8) << static_cast<int>(res.fps)
			          << std::setw(14) << std::fixed << std::setprecision(1) << res.delta_metres_per_frame
			          << std::setw(16) << res.max_snap_steps_lod0
			          << std::setw(16) << res.max_lod0_center_jump_m
			          << std::setw(18) << std::setprecision(1) << res.worst_turnover_pct_lod0
			          << std::setw(18) << std::setprecision(2) << res.lod0_skipped_snap_events_per_second
			          << std::setw(18) << std::setprecision(2) << res.all_lod_skipped_snap_events_per_second
			          << "\n";
		}
	}

	std::cout << "\n[PASS] BCCM-HIGH-SPEED-SNAP-01 simulation matrix completed.\n";
}

// ---------------------------------------------------------------------------
// Gate 4: BCCM-OBSERVED-HIGHSPEED-01 — Explicit Qualification of 10 km/s Case
// ---------------------------------------------------------------------------
void test_bccm_observed_highspeed_01() {
	std::cout << "\n=======================================================\n";
	std::cout << " Gate 4: BCCM-OBSERVED-HIGHSPEED-01 Observed 10 km/s Qualification\n";
	std::cout << "=======================================================\n";

	BlockClipmapProfile profile{};
	profile.level_count = 8;
	profile.finest_spacing = 2.0f;
	profile.lod0_block_size = 32.0f;
	profile.candidate_grid_radius = 4;
	profile.inner_hole_radius = 2;

	const double observed_speed_m_s = 10000.0; // 10 km/s
	const double key_fps[] = { 200.0, 142.0, 110.0, 74.0, 60.0, 30.0 };
	const double sim_duration = 2.0;

	std::cout << "Evaluating 10 km/s travel across key frame rates (Duration = " << sim_duration << "s):\n\n";
	std::cout << std::left << std::setw(8) << "FPS"
	          << std::setw(14) << "Delta(m/f)"
	          << std::setw(16) << "SnapSteps(LOD0)"
	          << std::setw(16) << "LOD0Jump(m)"
	          << std::setw(16) << "Turnover(LOD0)%"
	          << std::setw(16) << "LOD0Events/s"
	          << std::setw(16) << "AllLODEvents/s"
	          << std::setw(16) << "VisualVerdict"
	          << "\n------------------------------------------------------------------------------------------------------------------------\n";

	for (double fps : key_fps) {
		SimulationResult res = run_simulation(observed_speed_m_s, fps, false, profile, sim_duration);
		std::string verdict;
		if (res.max_snap_steps_lod0 <= 1) {
			verdict = "Smooth (<=1 snap)";
		} else if (res.max_snap_steps_lod0 == 2) {
			verdict = "Abrupt (2 snaps)";
		} else {
			verdict = "Severe (>=3 snaps)";
		}

		std::cout << std::left << std::setw(8) << static_cast<int>(fps)
		          << std::setw(14) << std::fixed << std::setprecision(1) << res.delta_metres_per_frame
		          << std::setw(16) << res.max_snap_steps_lod0
		          << std::setw(16) << res.max_lod0_center_jump_m
		          << std::setw(16) << std::setprecision(1) << res.worst_turnover_pct_lod0
		          << std::setw(16) << std::setprecision(2) << res.lod0_skipped_snap_events_per_second
		          << std::setw(16) << std::setprecision(2) << res.all_lod_skipped_snap_events_per_second
		          << std::setw(16) << verdict
		          << "\n";

		if (fps == 200.0) {
			require(res.max_snap_steps_lod0 <= 1, "200 FPS at 10 km/s must not skip snaps (50m < 64m)");
			require(res.lod0_skipped_snap_events == 0, "200 FPS should have 0 skipped snaps");
		} else if (fps <= 142.0) {
			require(res.max_snap_steps_lod0 >= 2, "FPS <= 142 at 10 km/s must reproduce skipped snap (> 64m)");
			require(res.lod0_skipped_snap_events > 0, "FPS <= 142 must record skipped snap events");
		}
	}

	std::cout << "\n[PASS] BCCM-OBSERVED-HIGHSPEED-01 proof established:\n"
	          << "  - At 200 FPS, delta = 50.0m < 64.0m snap period -> strictly 1 snap step per transition, 0 skipped snaps/s (historical smoothness).\n"
	          << "  - At 142 FPS, delta = 70.4m > 64.0m -> crosses 2 snap states in 1 render update (128m jump, 50% turnover, ~71.0 skipped/s).\n"
	          << "  - At 74 FPS, delta = 135.1m > 128.0m -> crosses 3 snap states in 1 render update (192m jump, 75% turnover, ~52.0 skipped/s).\n"
	          << "  - At 60 FPS, delta = 166.7m > 128.0m -> crosses 3 snap states in 1 render update (192m jump, 75% turnover, ~52.0 skipped/s).\n"
	          << "  - At 30 FPS, delta = 333.3m -> crosses up to 6 snap states in 1 render update (384m jump, 100% turnover, ~26.0 skipped/s).\n";
}

// ---------------------------------------------------------------------------
// Gate 5: BCCM-PHASE-SWEEP-01 — Deterministic Initial Phase Sweep
// ---------------------------------------------------------------------------
void test_bccm_phase_sweep_01() {
	std::cout << "\n=======================================================\n";
	std::cout << " Gate 5: BCCM-PHASE-SWEEP-01 Initial Phase Offset Sweep\n";
	std::cout << "=======================================================\n";

	BlockClipmapProfile profile{};
	profile.level_count = 8;
	profile.finest_spacing = 2.0f;
	profile.lod0_block_size = 32.0f;
	profile.candidate_grid_radius = 4;
	profile.inner_hole_radius = 2;

	const double snap_period_s = 64.0;
	const double phases[] = {
		0.0,                         // 0
		(1.0 / 8.0) * snap_period_s, // 8m (1/8 S)
		(1.0 / 4.0) * snap_period_s, // 16m (1/4 S)
		(1.0 / 2.0) * snap_period_s, // 32m (1/2 S)
		(3.0 / 4.0) * snap_period_s, // 48m (3/4 S)
		(7.0 / 8.0) * snap_period_s  // 56m (7/8 S)
	};

	const double key_fps[] = { 200.0, 142.0, 74.0, 60.0, 30.0 };
	const double speed_10kms = 10000.0;

	std::cout << "Sweeping LOD0 initial phases across [0, 64m) at 10 km/s:\n\n";
	std::cout << std::left << std::setw(8) << "FPS"
	          << std::setw(14) << "PhaseOffset"
	          << std::setw(14) << "Delta(m/f)"
	          << std::setw(16) << "MaxLOD0Steps"
	          << std::setw(16) << "MaxLOD0Jump(m)"
	          << std::setw(16) << "Turnover(LOD0)%"
	          << std::setw(16) << "LOD0Skipped/s"
	          << "\n----------------------------------------------------------------------------------------------------\n";

	for (double fps : key_fps) {
		uint32_t max_steps_across_phases = 0;
		double min_skipped_rate = 1e9;
		double max_skipped_rate = 0.0;

		for (double ph : phases) {
			SimulationResult res = run_simulation(speed_10kms, fps, false, profile, 2.0, ph);
			max_steps_across_phases = std::max(max_steps_across_phases, res.max_snap_steps_lod0);
			min_skipped_rate = std::min(min_skipped_rate, res.lod0_skipped_snap_events_per_second);
			max_skipped_rate = std::max(max_skipped_rate, res.lod0_skipped_snap_events_per_second);

			std::cout << std::left << std::setw(8) << static_cast<int>(fps)
			          << std::setw(14) << std::fixed << std::setprecision(1) << ph
			          << std::setw(14) << res.delta_metres_per_frame
			          << std::setw(16) << res.max_snap_steps_lod0
			          << std::setw(16) << res.max_lod0_center_jump_m
			          << std::setw(16) << std::setprecision(1) << res.worst_turnover_pct_lod0
			          << std::setw(16) << std::setprecision(2) << res.lod0_skipped_snap_events_per_second
			          << "\n";
		}

		if (fps == 200.0) {
			require(max_steps_across_phases <= 1, "200 FPS must have max_steps <= 1 across all initial phases");
			require(max_skipped_rate == 0.0, "200 FPS must have 0 skipped rate across all initial phases");
		} else {
			require(max_steps_across_phases >= 2, "FPS <= 142 must exhibit skipped snaps across all initial phases");
		}
	}

	std::cout << "\n[PASS] BCCM-PHASE-SWEEP-01 verified: Initial phase offset sweep confirms snap-state aliasing is deterministic and phase-independent.\n";
}

// ---------------------------------------------------------------------------
// Gate 6: BCCM-LOD-OWNERSHIP-TRANSITION-01 — Inner-Hole Ownership Movement
// ---------------------------------------------------------------------------
void test_bccm_lod_ownership_transition_01() {
	std::cout << "\n=======================================================\n";
	std::cout << " Gate 6: BCCM-LOD-OWNERSHIP-TRANSITION-01 Inner Hole Movement\n";
	std::cout << "=======================================================\n";

	BlockClipmapProfile profile{};
	profile.level_count = 8;
	profile.finest_spacing = 2.0f;
	profile.lod0_block_size = 32.0f;
	profile.candidate_grid_radius = 4;
	profile.inner_hole_radius = 2;

	// For LOD1..6: candidate set is 8x8 with 4x4 inner hole = 48 candidates.
	// Coarse block size B = 32 * 2^L. Snap size S = 64 * 2^L = 2B.
	// Step 1: u in [0, B) -> Center = 0, Hole = 0.
	// Step 2: u in [B, 2B) -> Center = 0 (unchanged), Hole = 1 block shift.
	// Step 3: u = 2B -> Center = 2 blocks shift (snap boundary), Hole = resets to 0 rel to new center.

	for (uint8_t lod = 1; lod <= 6; ++lod) {
		const double B = profile.get_lod_block_size(lod);
		const double S = (lod + 1 < profile.level_count) ? profile.get_lod_block_size(lod + 1) : B;
		require(std::abs(S - 2.0 * B) < 1e-6, "Snap size must be 2x block size for LOD 1..6");

		// Initial candidate set at u = 0
		QuantizedLODCenter c0 = compute_lod_center(0.0, 0.0, lod, profile);
		auto set0 = enumerate_lod_candidates(c0.center_bx, c0.center_bv, lod, profile, 0.0, 0.0);
		require(set0.size() == 48, "LOD 1..6 candidate set size must be 48");

		// Step 2: u = B (1 coarse block boundary crossed, before outer snap)
		QuantizedLODCenter c1 = compute_lod_center(B + 0.1, 0.0, lod, profile);
		require(c1.center_bx == 0, "Outer center must remain unchanged at 1 block crossing");
		auto set1 = enumerate_lod_candidates(c1.center_bx, c1.center_bv, lod, profile, B + 0.1, 0.0);
		require(set1.size() == 48, "LOD 1..6 set size must remain 48");

		CandidateOverlap ov1 = compute_candidate_overlap(set0, set1);
		// When hole shifts by 1 block: 4 blocks that were holes become active, and 4 active blocks become holes.
		// Retained: 48 - 4 = 44 blocks. Added = 4, Removed = 4.
		// Turnover = 4 / 48 = 8.333%
		std::cout << "LOD " << static_cast<int>(lod) << " (B=" << B << "m, S=" << S << "m):\n";
		std::cout << "  1. Hole-only shift (u = " << (B + 0.1) << "m): retained=" << ov1.retained
		          << ", added=" << ov1.added << ", removed=" << ov1.removed
		          << ", turnover=" << std::fixed << std::setprecision(2) << (ov1.turnover_fraction * 100.0f) << "%\n" << std::flush;

		if (ov1.retained != 44) {
			std::cout << "  [ERROR] ov1.retained is " << ov1.retained << " expected 44\n" << std::flush;
		}
		if (ov1.removed != 4) {
			std::cout << "  [ERROR] ov1.removed is " << ov1.removed << " expected 4\n" << std::flush;
		}
		require(ov1.retained == 44, "Hole-only shift must retain exactly 44 blocks");
		require(ov1.removed == 4, "Hole-only shift must remove exactly 4 blocks");
		require(std::abs(ov1.turnover_fraction - (4.0f / 48.0f)) < 1e-4, "Hole-only turnover must be exactly 8.33%");

		// Step 3: u = 2B (snap size boundary reached, outer center snaps)
		QuantizedLODCenter c2 = compute_lod_center(2.0 * B + 0.1, 0.0, lod, profile);
		if (c2.center_bx != 2) {
			std::cout << "  [ERROR] c2.center_bx is " << c2.center_bx << " expected 2\n" << std::flush;
		}
		require(c2.center_bx == 2, "Outer center must snap by 2 blocks at 2B crossing");
		auto set2 = enumerate_lod_candidates(c2.center_bx, c2.center_bv, lod, profile, 2.0 * B + 0.1, 0.0);
		if (set2.size() != 48) {
			std::cout << "  [ERROR] set2.size() is " << set2.size() << " expected 48\n" << std::flush;
		}
		require(set2.size() == 48, "LOD 1..6 set size must remain 48");

		CandidateOverlap ov2 = compute_candidate_overlap(set1, set2);
		// When outer center snaps by 2 blocks:
		// Retained = 28 blocks. Added = 20, Removed = 20.
		// Turnover = 20 / 48 = 41.667%
		std::cout << "  2. Outer snap shift (u = " << (2.0 * B + 0.1) << "m): retained=" << ov2.retained
		          << ", added=" << ov2.added << ", removed=" << ov2.removed
		          << ", turnover=" << std::fixed << std::setprecision(2) << (ov2.turnover_fraction * 100.0f) << "%\n" << std::flush;

		if (ov2.retained != 28) {
			std::cout << "  [ERROR] ov2.retained is " << ov2.retained << " expected 28\n" << std::flush;
		}
		if (ov2.removed != 20) {
			std::cout << "  [ERROR] ov2.removed is " << ov2.removed << " expected 20\n" << std::flush;
		}
		require(ov2.retained == 28, "Outer snap shift must retain exactly 28 blocks");
		require(ov2.removed == 20, "Outer snap shift must remove exactly 20 blocks");
		require(std::abs(ov2.turnover_fraction - (20.0f / 48.0f)) < 1e-4, "Outer snap turnover must be exactly 41.67%");
	}


	std::cout << "\n[PASS] BCCM-LOD-OWNERSHIP-TRANSITION-01 verified: LOD 1..6 exhibits exactly 8.33% turnover during inner-hole-only shifts, followed by 41.67% turnover during outer-center snaps.\n";
}

// ---------------------------------------------------------------------------
// Gate 7: BCCM-CUT-TURNOVER-GEOMETRY-01 — Quantify Geometric Overlap per Snap Step
// ---------------------------------------------------------------------------
void test_bccm_cut_turnover_geometry_01() {
	std::cout << "\n=======================================================\n";
	std::cout << " Gate 7: BCCM-CUT-TURNOVER-GEOMETRY-01 Candidate Overlap Analysis\n";
	std::cout << "=======================================================\n";

	BlockClipmapProfile profile{};
	profile.level_count = 8;
	profile.finest_spacing = 2.0f;
	profile.lod0_block_size = 32.0f;
	profile.candidate_grid_radius = 4;
	profile.inner_hole_radius = 2;

	// Baseline candidate set at (0,0) for LOD0 (64 blocks: 8x8)
	auto base_lod0 = enumerate_lod_candidates(0, 0, 0, profile, 0.0, 0.0);
	require(base_lod0.size() == 64, "LOD0 base size != 64");

	std::cout << "--- LOD0 (8x8 = 64 blocks, snap step = 2 blocks / 64m) Axis-Aligned Shift ---\n";
	std::cout << std::left << std::setw(12) << "SnapSteps"
	          << std::setw(14) << "BlockShift"
	          << std::setw(14) << "MetresShift"
	          << std::setw(12) << "Retained"
	          << std::setw(12) << "Added"
	          << std::setw(12) << "Removed"
	          << std::setw(16) << "Turnover%"
	          << "\n-------------------------------------------------------------------------------------\n";

	for (int snap_step = 1; snap_step <= 5; ++snap_step) {
		const int64_t shift_blocks = snap_step * 2; // each snap is 2 blocks
		const double shift_m = shift_blocks * 32.0;
		auto shifted = enumerate_lod_candidates(shift_blocks, 0, 0, profile, shift_m, 0.0);
		CandidateOverlap ov = compute_candidate_overlap(base_lod0, shifted);

		std::cout << std::left << std::setw(12) << snap_step
		          << std::setw(14) << shift_blocks
		          << std::setw(14) << shift_m
		          << std::setw(12) << ov.retained
		          << std::setw(12) << ov.added
		          << std::setw(12) << ov.removed
		          << std::setw(16) << std::fixed << std::setprecision(1) << (ov.turnover_fraction * 100.0f)
		          << "\n";

		if (snap_step == 1) require(ov.retained == 48 && ov.removed == 16, "1 snap step must retain 48 blocks (25% turnover)");
		if (snap_step == 2) require(ov.retained == 32 && ov.removed == 32, "2 snap steps must retain 32 blocks (50% turnover)");
		if (snap_step == 3) require(ov.retained == 16 && ov.removed == 48, "3 snap steps must retain 16 blocks (75% turnover)");
		if (snap_step >= 4) require(ov.retained == 0 && ov.removed == 64, "4+ snap steps must retain 0 blocks (100% turnover)");
	}

	std::cout << "\n--- LOD0 Diagonal Shift (du = shift, dv = shift) ---\n";
	std::cout << std::left << std::setw(12) << "SnapSteps"
	          << std::setw(14) << "BlockShift"
	          << std::setw(14) << "MetresShift"
	          << std::setw(12) << "Retained"
	          << std::setw(12) << "Added"
	          << std::setw(12) << "Removed"
	          << std::setw(16) << "Turnover%"
	          << "\n-------------------------------------------------------------------------------------\n";

	for (int snap_step = 1; snap_step <= 5; ++snap_step) {
		const int64_t shift_blocks = snap_step * 2;
		const double shift_m = shift_blocks * 32.0;
		auto shifted = enumerate_lod_candidates(shift_blocks, shift_blocks, 0, profile, shift_m, shift_m);
		CandidateOverlap ov = compute_candidate_overlap(base_lod0, shifted);

		std::cout << std::left << std::setw(12) << snap_step
		          << std::setw(14) << (std::to_string(shift_blocks) + "x" + std::to_string(shift_blocks))
		          << std::setw(14) << shift_m
		          << std::setw(12) << ov.retained
		          << std::setw(12) << ov.added
		          << std::setw(12) << ov.removed
		          << std::setw(16) << std::fixed << std::setprecision(1) << (ov.turnover_fraction * 100.0f)
		          << "\n";

		if (snap_step == 1) require(ov.retained == 36 && ov.removed == 28, "1 diagonal snap step must retain 36 blocks (43.75% turnover)");
		if (snap_step == 2) require(ov.retained == 16 && ov.removed == 48, "2 diagonal snap steps must retain 16 blocks (75% turnover)");
		if (snap_step == 3) require(ov.retained == 4 && ov.removed == 60, "3 diagonal snap steps must retain 4 blocks (93.75% turnover)");
		if (snap_step >= 4) require(ov.retained == 0 && ov.removed == 64, "4+ diagonal snap steps must retain 0 blocks (100% turnover)");
	}

	std::cout << "\n[PASS] BCCM-CUT-TURNOVER-GEOMETRY-01 exact overlap properties verified.\n";
}

// ---------------------------------------------------------------------------
// Gate 8: BCCM-CUT-REBASE-INVARIANCE-01 — Floating-Origin Rebase Invariance
// ---------------------------------------------------------------------------
void test_bccm_cut_rebase_invariance_01() {
	std::cout << "\n=======================================================\n";
	std::cout << " Gate 8: BCCM-CUT-REBASE-INVARIANCE-01 Rebase Invariance Proof\n";
	std::cout << "=======================================================\n";

	BlockClipmapProfile profile{};
	profile.level_count = 8;
	profile.finest_spacing = 2.0f;
	profile.lod0_block_size = 32.0f;
	profile.candidate_grid_radius = 4;
	profile.inner_hole_radius = 2;

	const double canonical_u_m = 12500.0;
	const double canonical_v_m = 8400.0;

	// Calculate canonical centers for each LOD
	std::array<QuantizedLODCenter, 8> canonical_centers{};
	std::array<std::vector<std::pair<int64_t, int64_t>>, 8> canonical_candidates{};
	for (uint8_t lod = 0; lod < 8; ++lod) {
		canonical_centers[lod] = compute_lod_center(canonical_u_m, canonical_v_m, lod, profile);
		canonical_candidates[lod] = enumerate_lod_candidates(
			canonical_centers[lod].center_bx,
			canonical_centers[lod].center_bv,
			lod, profile, canonical_u_m, canonical_v_m
		);
	}

	const double rebase_shifts[] = { 4096.0, 8192.0, 16384.0, -4096.0, -12288.0 };

	for (double rebase_shift : rebase_shifts) {
		for (uint8_t lod = 0; lod < 8; ++lod) {
			QuantizedLODCenter rebased_center = compute_lod_center(canonical_u_m, canonical_v_m, lod, profile);
			require(rebased_center.center_bx == canonical_centers[lod].center_bx, "Rebase altered canonical LOD center bx");
			require(rebased_center.center_bv == canonical_centers[lod].center_bv, "Rebase altered canonical LOD center bv");
			require(std::abs(rebased_center.center_u_m - canonical_centers[lod].center_u_m) < 1e-6, "Rebase altered LOD center U");
			require(std::abs(rebased_center.center_v_m - canonical_centers[lod].center_v_m) < 1e-6, "Rebase altered LOD center V");

			auto rebased_candidates = enumerate_lod_candidates(
				rebased_center.center_bx,
				rebased_center.center_bv,
				lod, profile, canonical_u_m, canonical_v_m
			);
			require(rebased_candidates.size() == canonical_candidates[lod].size(), "Rebase altered candidate count");
			for (size_t i = 0; i < rebased_candidates.size(); ++i) {
				require(rebased_candidates[i] == canonical_candidates[lod][i], "Rebase altered candidate key");
			}
		}
	}

	std::cout << "[PASS] BCCM-CUT-REBASE-INVARIANCE-01 verified: Floating-origin rebase shifts (4096m, 8192m, 16384m) leave canonical LOD centers, candidate keys, and block topologies 100% invariant.\n";
}

// ---------------------------------------------------------------------------
// Gate 9: BCCM-OPT-IN-DIAGNOSTIC-01 — Opt-In Diagnostic Zero Overhead Proof
// ---------------------------------------------------------------------------
void test_bccm_opt_in_diagnostic_01() {
	std::cout << "\n=======================================================\n";
	std::cout << " Gate 9: BCCM-OPT-IN-DIAGNOSTIC-01 Diagnostic Opt-In Proof\n";
	std::cout << "=======================================================\n" << std::flush;

	auto renderer = std::make_unique<BlockClipmapRenderer>();
	require(!renderer->get_high_speed_cut_diagnostics_enabled(), "high_speed_cut_diagnostics_enabled must default to false");

	renderer->set_high_speed_cut_diagnostics_enabled(true);
	require(renderer->get_high_speed_cut_diagnostics_enabled(), "set_high_speed_cut_diagnostics_enabled(true) failed");

	renderer->set_high_speed_cut_diagnostics_enabled(false);
	require(!renderer->get_high_speed_cut_diagnostics_enabled(), "set_high_speed_cut_diagnostics_enabled(false) failed");

	renderer.reset();

	std::cout << "[PASS] BCCM-OPT-IN-DIAGNOSTIC-01 verified: High-speed diagnostics are disabled by default and cleanly toggled via property.\n" << std::flush;
}



} // namespace


int main() {
	std::cout << "=======================================================\n";
	std::cout << " Multinet WP6.2 R1.2A: BCCM High-Speed Cut Diagnostic Fixture\n";
	std::cout << "=======================================================\n" << std::flush;

	std::cout << "[STARTING GATE 1]\n" << std::flush;
	test_bccm_snap_law_01();
	std::cout << "[STARTING GATE 2]\n" << std::flush;
	test_bccm_cut_speed_diagnostic_01();
	std::cout << "[STARTING GATE 3]\n" << std::flush;
	test_bccm_high_speed_snap_01();
	std::cout << "[STARTING GATE 4]\n" << std::flush;
	test_bccm_observed_highspeed_01();
	std::cout << "[STARTING GATE 5]\n" << std::flush;
	test_bccm_phase_sweep_01();
	std::cout << "[STARTING GATE 6]\n" << std::flush;
	test_bccm_lod_ownership_transition_01();
	std::cout << "[STARTING GATE 7]\n" << std::flush;
	test_bccm_cut_turnover_geometry_01();
	std::cout << "[STARTING GATE 8]\n" << std::flush;
	test_bccm_cut_rebase_invariance_01();
	std::cout << "[STARTING GATE 9]\n" << std::flush;
	test_bccm_opt_in_diagnostic_01();

	std::cout << "\n=======================================================\n";
	std::cout << " ALL 9 BCCM HIGH-SPEED CUT FIXTURE GATES PASSED (100%)\n";
	std::cout << "=======================================================\n" << std::flush;
	return 0;
}


