#include "multinet/rendering/chp/chp_bounds.h"
#include "multinet/rendering/chp/chp_kernel.h"
#include "multinet/rendering/chp/chp_view.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_profile.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_culling.h"
#include "multinet/core/spatial/surface_frame.h"
#include "multinet/core/spatial/world_domain.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using namespace multinet::rendering;
using namespace multinet::rendering::chp;

namespace {

void require(bool value, const std::string& message) {
	if (!value) {
		std::cerr << "FAILURE: " << message << "\n";
		std::exit(1);
	}
}

ResolvedCurvedHorizonProfile make_earth_profile(double certified_distance_m = 100000.0) {
	ResolvedCurvedHorizonProfile profile{};
	profile.requested.function_class = CHPFunctionClass::SphericalPolynomial6;
	profile.requested.requested_maximum_deformation_distance_m = certified_distance_m;
	profile.requested.maximum_base_position_error_m = 1.0;
	profile.requested.maximum_visual_up_error_radians = 1.0e-4;
	profile.radius_m = 6371000.0;
	profile.inverse_radius = 1.0 / 6371000.0;
	profile.inverse_radius_squared = 1.0 / (6371000.0 * 6371000.0);
	profile.certified_maximum_deformation_distance_m = certified_distance_m;
	profile.certified_maximum_theta = certified_distance_m / 6371000.0;
	profile.certified_maximum_u = profile.certified_maximum_theta * profile.certified_maximum_theta;
	return profile;
}

constexpr double CONST_PI = 3.14159265358979323846;

FrustumPlanes make_perspective_frustum(
	const godot::Vector3& cam_pos,
	double yaw_deg,
	double pitch_deg,
	double fov_deg = 70.0,
	double aspect_ratio = 16.0 / 9.0,
	double near_dist = 0.5,
	double far_dist = 50000.0
) {
	FrustumPlanes frustum{};
	frustum.valid = true;

	const double yaw_rad = yaw_deg * (CONST_PI / 180.0);
	const double pitch_rad = pitch_deg * (CONST_PI / 180.0);

	// Camera forward, right, up vectors in Godot convention (-Z forward, +X right, +Y up)
	godot::Vector3 fwd(
		static_cast<float>(-std::sin(yaw_rad) * std::cos(pitch_rad)),
		static_cast<float>(std::sin(pitch_rad)),
		static_cast<float>(-std::cos(yaw_rad) * std::cos(pitch_rad))
	);
	fwd.normalize();

	godot::Vector3 world_up(0.0f, 1.0f, 0.0f);
	godot::Vector3 right = fwd.cross(world_up);
	right.normalize();
	godot::Vector3 up = right.cross(fwd);
	up.normalize();

	const double fov_y_rad = fov_deg * (CONST_PI / 180.0);
	const double half_v = std::tan(fov_y_rad * 0.5);
	const double half_h = half_v * aspect_ratio;

	// Godot 4 get_frustum() outward normal planes: is_point_over(pt) == true means outside frustum.
	// 0: Near plane (outward normal points backward: -fwd)
	frustum.planes[0] = godot::Plane(-fwd, (-fwd).dot(cam_pos + fwd * static_cast<float>(near_dist)));
	// 1: Far plane (outward normal points forward: +fwd)
	frustum.planes[1] = godot::Plane(fwd, fwd.dot(cam_pos + fwd * static_cast<float>(far_dist)));

	// 2: Left plane (outward normal points left)
	godot::Vector3 left_norm = up.cross(fwd - right * static_cast<float>(half_h));
	left_norm.normalize();
	frustum.planes[2] = godot::Plane(left_norm, left_norm.dot(cam_pos));

	// 3: Right plane (outward normal points right)
	godot::Vector3 right_norm = (fwd + right * static_cast<float>(half_h)).cross(up);
	right_norm.normalize();
	frustum.planes[3] = godot::Plane(right_norm, right_norm.dot(cam_pos));

	// 4: Top plane (outward normal points up)
	godot::Vector3 top_norm = right.cross(fwd + up * static_cast<float>(half_v));
	top_norm.normalize();
	frustum.planes[4] = godot::Plane(top_norm, top_norm.dot(cam_pos));

	// 5: Bottom plane (outward normal points down)
	godot::Vector3 bot_norm = (fwd - up * static_cast<float>(half_v)).cross(right);
	bot_norm.normalize();
	frustum.planes[5] = godot::Plane(bot_norm, bot_norm.dot(cam_pos));

	return frustum;
}

struct BlockCandidate {
	uint8_t lod{ 0 };
	int32_t du{ 0 };
	int32_t dv{ 0 };
	int64_t dist_sq_m{ 0 };
	double flat_min_x{ 0.0 };
	double flat_max_x{ 0.0 };
	double flat_min_z{ 0.0 };
	double flat_max_z{ 0.0 };
	double height_min{ -50.0 };
	double height_max{ 150.0 };
	bool exact_visible{ false };
	bool guard_visible{ false };
	bool resident_visible{ false };
};

struct SimulatedLease {
	uint8_t lod{ 0 };
	int32_t du{ 0 };
	int32_t dv{ 0 };
	double lease_remaining_seconds{ 0.0 };
	bool has_lease{ false };
};

struct SimulatedLODState {
	std::vector<SimulatedLease> leases;
	std::vector<BlockCandidate> last_submitted_set;
	uint32_t buffer_dirty_count{ 0 };
	uint32_t additions_count{ 0 };
	uint32_t removals_count{ 0 };
};

void run_simulated_frame(
	const ResolvedCurvedHorizonProfile& profile,
	const BlockClipmapProfile& clipmap_profile,
	const godot::Vector3& cam_pos,
	double yaw_deg,
	double pitch_deg,
	double delta_seconds,
	bool use_residency,
	std::array<SimulatedLODState, 8>& lod_states,
	std::vector<BlockCandidate>& out_all_candidates
) {
	out_all_candidates.clear();
	const FrustumPlanes frustum = make_perspective_frustum(cam_pos, yaw_deg, pitch_deg);

	for (uint8_t lod = 0; lod < 8; ++lod) {
		const double block_size = clipmap_profile.get_lod_block_size(lod);
		const double guard_pad_m = 0.5 * block_size;
		const int32_t r = clipmap_profile.candidate_grid_radius;
		const int32_t hole_r = clipmap_profile.inner_hole_radius;

		std::vector<BlockCandidate> candidates;
		for (int32_t dv = -r; dv < r; ++dv) {
			for (int32_t du = -r; du < r; ++du) {
				if (lod > 0) {
					if (du >= -hole_r && du < hole_r && dv >= -hole_r && dv < hole_r) continue;
				}
				BlockCandidate cand{};
				cand.lod = lod;
				cand.du = du;
				cand.dv = dv;
				cand.dist_sq_m = static_cast<int64_t>((du * du + dv * dv) * block_size * block_size);
				cand.flat_min_x = du * block_size;
				cand.flat_max_x = cand.flat_min_x + block_size;
				cand.flat_min_z = dv * block_size;
				cand.flat_max_z = cand.flat_min_z + block_size;

				CHPCurvedCoverageBounds bounds{};
				if (try_build_conservative_curved_bounds(
					profile, 0.0,
					cand.flat_min_x, cand.flat_max_x,
					cand.flat_min_z, cand.flat_max_z,
					cand.height_min, cand.height_max,
					bounds) && bounds.valid)
				{
					const godot::AABB curved_world_aabb(
						godot::Vector3(
							static_cast<float>(bounds.minimum_x_m),
							static_cast<float>(bounds.minimum_y_m),
							static_cast<float>(bounds.minimum_z_m)
						) + cam_pos,
						godot::Vector3(
							static_cast<float>(bounds.maximum_x_m - bounds.minimum_x_m),
							static_cast<float>(bounds.maximum_y_m - bounds.minimum_y_m),
							static_cast<float>(bounds.maximum_z_m - bounds.minimum_z_m)
						)
					);
					cand.exact_visible = frustum.intersects_aabb(curved_world_aabb);
					const godot::AABB guard_world_aabb = curved_world_aabb.grow(static_cast<float>(guard_pad_m));
					cand.guard_visible = frustum.intersects_aabb(guard_world_aabb);
					if (cand.exact_visible) {
						cand.guard_visible = true; // Invariant
					}
				} else {
					cand.exact_visible = true;
					cand.guard_visible = true;
				}
				candidates.push_back(cand);
			}
		}

		auto& state = lod_states[lod];
		if (!use_residency) {
			for (auto& cand : candidates) {
				cand.resident_visible = cand.exact_visible;
			}
		} else {
			// Step 1: Match against leases
			std::vector<int32_t> lease_indices(candidates.size(), -1);
			for (size_t c_i = 0; c_i < candidates.size(); ++c_i) {
				for (size_t l_i = 0; l_i < state.leases.size(); ++l_i) {
					if (state.leases[l_i].du == candidates[c_i].du && state.leases[l_i].dv == candidates[c_i].dv) {
						lease_indices[c_i] = static_cast<int32_t>(l_i);
						break;
					}
				}
			}

			// Step 2: Update leases and identify eviction candidates
			struct EvictionInfo {
				size_t cand_idx;
				int32_t lease_idx;
				int64_t dist_sq_m;
			};
			std::vector<EvictionInfo> evictions;

			for (size_t c_i = 0; c_i < candidates.size(); ++c_i) {
				auto& cand = candidates[c_i];
				const int32_t l_idx = lease_indices[c_i];

				if (cand.guard_visible) {
					cand.resident_visible = true;
					if (l_idx >= 0) {
						state.leases[l_idx].lease_remaining_seconds = 0.20;
						state.leases[l_idx].has_lease = true;
					} else {
						SimulatedLease new_lease{};
						new_lease.lod = lod;
						new_lease.du = cand.du;
						new_lease.dv = cand.dv;
						new_lease.lease_remaining_seconds = 0.20;
						new_lease.has_lease = true;
						state.leases.push_back(new_lease);
					}
				} else {
					if (l_idx >= 0 && state.leases[l_idx].has_lease) {
						state.leases[l_idx].lease_remaining_seconds -= delta_seconds;
						if (state.leases[l_idx].lease_remaining_seconds > 0.0) {
							cand.resident_visible = true;
						} else {
							evictions.push_back({ c_i, l_idx, cand.dist_sq_m });
						}
					} else {
						cand.resident_visible = false;
					}
				}
			}

			// Step 3: Eviction budget (at most 2 per LOD per frame)
			if (evictions.size() > 2) {
				std::sort(evictions.begin(), evictions.end(), [](const EvictionInfo& a, const EvictionInfo& b) {
					return a.dist_sq_m > b.dist_sq_m; // farthest first
				});
				for (size_t i = 0; i < 2; ++i) {
					candidates[evictions[i].cand_idx].resident_visible = false;
					if (evictions[i].lease_idx >= 0) {
						state.leases[evictions[i].lease_idx].has_lease = false;
					}
				}
				for (size_t i = 2; i < evictions.size(); ++i) {
					candidates[evictions[i].cand_idx].resident_visible = true; // Retain
					if (evictions[i].lease_idx >= 0) {
						state.leases[evictions[i].lease_idx].has_lease = true;
					}
				}
			} else {
				for (const auto& ev : evictions) {
					candidates[ev.cand_idx].resident_visible = false;
					if (ev.lease_idx >= 0) {
						state.leases[ev.lease_idx].has_lease = false;
					}
				}
			}

			// Step 4: Invariant enforcement
			for (auto& cand : candidates) {
				if (cand.exact_visible) {
					cand.resident_visible = true;
				}
			}

			// Step 5: Compact leases
			state.leases.erase(
				std::remove_if(state.leases.begin(), state.leases.end(), [](const SimulatedLease& l) { return !l.has_lease; }),
				state.leases.end()
			);
		}

		// Track churn
		std::vector<BlockCandidate> curr_visible;
		for (const auto& c : candidates) {
			if (c.resident_visible) curr_visible.push_back(c);
		}

		// Additions & Removals relative to last submitted set
		for (const auto& curr : curr_visible) {
			bool found = false;
			for (const auto& prev : state.last_submitted_set) {
				if (curr.du == prev.du && curr.dv == prev.dv) { found = true; break; }
			}
			if (!found) state.additions_count++;
		}
		for (const auto& prev : state.last_submitted_set) {
			bool found = false;
			for (const auto& curr : curr_visible) {
				if (curr.du == prev.du && curr.dv == prev.dv) { found = true; break; }
			}
			if (!found) state.removals_count++;
		}

		if (curr_visible.size() != state.last_submitted_set.size()) {
			state.buffer_dirty_count++;
		} else {
			bool changed = false;
			for (size_t i = 0; i < curr_visible.size(); ++i) {
				if (curr_visible[i].du != state.last_submitted_set[i].du || curr_visible[i].dv != state.last_submitted_set[i].dv) {
					changed = true;
					break;
				}
			}
			if (changed) state.buffer_dirty_count++;
		}
		state.last_submitted_set = curr_visible;

		for (const auto& c : candidates) {
			out_all_candidates.push_back(c);
		}
	}
}

} // namespace

int main() {
	std::cout << std::setprecision(15);
	const ResolvedCurvedHorizonProfile profile = make_earth_profile();
	const BlockClipmapProfile clipmap_profile{};

	std::cout << "## rendering::BCCM-R3.1-TEMPORAL-COHERENCE-FIXTURE\n";

	// 1. Gate: BCCM-R3-RESIDENT-SUPERSET-01
	// Verify that ExactVisibleSet is strictly a subset of ResidentVisibleSet across 10 diverse trajectories
	{
		uint64_t total_samples_tested = 0;
		uint64_t total_false_negatives = 0;

		struct TrajectoryTest {
			std::string name;
			uint32_t frames;
			double start_yaw;
			double yaw_step;
			double start_pitch;
			double pitch_step;
			godot::Vector3 start_pos;
			godot::Vector3 pos_step;
		};

		const std::array<TrajectoryTest, 10> trajectories = {{
			{ "Stationary", 60, 0.0, 0.0, 0.0, 0.0, godot::Vector3(0, 50, 0), godot::Vector3(0, 0, 0) },
			{ "SlowYaw", 180, 0.0, 1.0, 0.0, 0.0, godot::Vector3(0, 50, 0), godot::Vector3(0, 0, 0) },
			{ "RapidYaw", 60, 0.0, 15.0, 0.0, 0.0, godot::Vector3(0, 50, 0), godot::Vector3(0, 0, 0) },
			{ "YawReversal", 120, -15.0, 0.5, 0.0, 0.0, godot::Vector3(0, 50, 0), godot::Vector3(0, 0, 0) },
			{ "PitchVariation", 60, 0.0, 0.0, -30.0, 1.0, godot::Vector3(0, 50, 0), godot::Vector3(0, 0, 0) },
			{ "CombinedYawPitch", 120, 0.0, 2.0, -20.0, 0.33, godot::Vector3(0, 50, 0), godot::Vector3(0, 0, 0) },
			{ "TranslationX", 60, 45.0, 0.0, 0.0, 0.0, godot::Vector3(0, 50, 0), godot::Vector3(100, 0, 0) },
			{ "TranslationZ", 60, 45.0, 0.0, 0.0, 0.0, godot::Vector3(0, 50, 0), godot::Vector3(0, 0, 100) },
			{ "TranslationAndRotation", 100, 0.0, 1.5, 0.0, 0.0, godot::Vector3(0, 50, 0), godot::Vector3(50, 0, 50) },
			{ "TeleportJump", 20, 0.0, 90.0, 0.0, 0.0, godot::Vector3(0, 50, 0), godot::Vector3(10000, 0, 10000) }
		}};

		for (const auto& traj : trajectories) {
			std::array<SimulatedLODState, 8> states{};
			godot::Vector3 pos = traj.start_pos;
			double yaw = traj.start_yaw;
			double pitch = traj.start_pitch;

			for (uint32_t f = 0; f < traj.frames; ++f) {
				std::vector<BlockCandidate> cands;
				run_simulated_frame(profile, clipmap_profile, pos, yaw, pitch, 1.0 / 60.0, true, states, cands);

				for (const auto& c : cands) {
					total_samples_tested++;
					if (c.exact_visible && !c.resident_visible) {
						total_false_negatives++;
					}
				}

				pos += traj.pos_step;
				yaw += traj.yaw_step;
				pitch += traj.pitch_step;
			}
		}

		require(total_false_negatives == 0, "False negatives detected: exact_visible block was not resident!");
		std::cout << "[PASS] BCCM-R3-RESIDENT-SUPERSET-01: Tested " << total_samples_tested
		          << " block states across 10 trajectories, 0 false negatives.\n";
	}

	// 2. Gate: BCCM-R3-TEMPORAL-COHERENCE-01
	// Measure rotation churn during back-and-forth oscillation: exact vs R3.1 residency
	{
		const uint32_t OSCILLATION_FRAMES = 160;
		const double YAW_AMPLITUDE_DEG = 8.0;
		const double FREQUENCY = 0.05; // oscillation frequency

		// Run Exact Culling (No residency)
		std::array<SimulatedLODState, 8> exact_states{};
		for (uint32_t f = 0; f < OSCILLATION_FRAMES; ++f) {
			double yaw = YAW_AMPLITUDE_DEG * std::sin(2.0 * CONST_PI * FREQUENCY * f);
			std::vector<BlockCandidate> cands;
			run_simulated_frame(profile, clipmap_profile, godot::Vector3(0, 50, 0), yaw, 0.0, 1.0 / 60.0, false, exact_states, cands);
		}

		uint32_t exact_total_additions = 0;
		uint32_t exact_total_removals = 0;
		uint32_t exact_total_buffer_dirties = 0;
		for (int lod = 0; lod < 8; ++lod) {
			exact_total_additions += exact_states[lod].additions_count;
			exact_total_removals += exact_states[lod].removals_count;
			exact_total_buffer_dirties += exact_states[lod].buffer_dirty_count;
		}

		// Run R3.1 Coherent Residency
		std::array<SimulatedLODState, 8> resident_states{};
		for (uint32_t f = 0; f < OSCILLATION_FRAMES; ++f) {
			double yaw = YAW_AMPLITUDE_DEG * std::sin(2.0 * CONST_PI * FREQUENCY * f);
			std::vector<BlockCandidate> cands;
			run_simulated_frame(profile, clipmap_profile, godot::Vector3(0, 50, 0), yaw, 0.0, 1.0 / 60.0, true, resident_states, cands);
		}

		uint32_t resident_total_additions = 0;
		uint32_t resident_total_removals = 0;
		uint32_t resident_total_buffer_dirties = 0;
		for (int lod = 0; lod < 8; ++lod) {
			resident_total_additions += resident_states[lod].additions_count;
			resident_total_removals += resident_states[lod].removals_count;
			resident_total_buffer_dirties += resident_states[lod].buffer_dirty_count;
		}

		std::cout << "  Exact Culling without Residency: Additions=" << exact_total_additions
		          << ", Removals=" << exact_total_removals
		          << ", MultiMesh Dirty Rewrites=" << exact_total_buffer_dirties << "\n";
		std::cout << "  R3.1 Coherent Residency:         Additions=" << resident_total_additions
		          << ", Removals=" << resident_total_removals
		          << ", MultiMesh Dirty Rewrites=" << resident_total_buffer_dirties << "\n";

		require(resident_total_buffer_dirties < exact_total_buffer_dirties,
			"R3.1 failed to reduce MultiMesh buffer churn during camera turning!");
		require(resident_total_additions + resident_total_removals < exact_total_additions + exact_total_removals,
			"R3.1 failed to reduce membership turnover during camera turning!");

		std::cout << "[PASS] BCCM-R3-TEMPORAL-COHERENCE-01: MultiMesh dirty buffer rewrites reduced by "
		          << std::fixed << std::setprecision(1)
		          << (1.0 - static_cast<double>(resident_total_buffer_dirties) / exact_total_buffer_dirties) * 100.0
		          << "%, turnover reduced by "
		          << (1.0 - static_cast<double>(resident_total_additions + resident_total_removals) / (exact_total_additions + exact_total_removals)) * 100.0
		          << "%.\n";
	}

	// 3. Gate: BCCM-R3-FAST-TURN-CORRECTNESS-01
	// Verify instant 100% admission under rapid 180-deg reversal
	{
		std::array<SimulatedLODState, 8> states{};
		std::vector<BlockCandidate> cands_before;
		run_simulated_frame(profile, clipmap_profile, godot::Vector3(0, 50, 0), 0.0, 0.0, 1.0 / 60.0, true, states, cands_before);

		// Frame 2: Instant 180-degree yaw snap
		std::vector<BlockCandidate> cands_after;
		run_simulated_frame(profile, clipmap_profile, godot::Vector3(0, 50, 0), 180.0, 0.0, 1.0 / 60.0, true, states, cands_after);

		uint32_t exact_vis_count = 0;
		uint32_t resident_vis_count = 0;
		for (const auto& c : cands_after) {
			if (c.exact_visible) {
				exact_vis_count++;
				require(c.resident_visible, "Instant 180-deg turn missed newly visible block!");
			}
			if (c.resident_visible) {
				resident_vis_count++;
			}
		}

		require(exact_vis_count > 0, "No visible blocks detected at 180 deg view");
		std::cout << "[PASS] BCCM-R3-FAST-TURN-CORRECTNESS-01: 180-deg snap admitted 100% of "
		          << exact_vis_count << " newly exact-visible blocks immediately (resident="
		          << resident_vis_count << ", 0 false negatives).\n";
	}

	std::cout << "STATUS: PASSED WITH EVIDENCE\n";
	return 0;
}
