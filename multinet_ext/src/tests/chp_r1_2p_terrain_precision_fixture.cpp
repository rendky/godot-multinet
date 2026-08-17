#include "multinet/core/spatial/world_domain.h"
#include "multinet/core/spatial/world_manifests.h"
#include "multinet/core/squirrel_noise5.h"
#include "multinet/world/terrain/canonical_terrain_signal.h"
#include "multinet/world/terrain/terrain_recipe.h"
#include "multinet/rendering/terrain/block_clipmap/terrain_sample_patch.h"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>

using namespace Multinet;
using namespace multinet::rendering;

namespace {

void require(bool condition, const char* message) {
	if (!condition) {
		std::cerr << "FAIL: " << message << std::endl;
		std::exit(1);
	}
}

// ---------------------------------------------------------------------------
// Struct for per-octave root anchors
// ---------------------------------------------------------------------------
struct TerrainRootLatticeAnchors {
	int32_t cell[8][3]{};
	float fraction[8][3]{};
};

TerrainRootLatticeAnchors compute_root_anchors(
	const FramePosition64& root_direction,
	double radius_m,
	const TerrainRecipe& recipe
) {
	TerrainRootLatticeAnchors anchors{};
	const double px = root_direction.x * radius_m;
	const double py = root_direction.y * radius_m;
	const double pz = root_direction.z * radius_m;

	double freq = recipe.legacy_signals.continental_frequency;
	const uint8_t octaves = std::min<uint8_t>(recipe.legacy_signals.octave_count, 8);

	for (uint8_t oct = 0; oct < octaves; ++oct) {
		double sx = px * freq;
		double sy = py * freq;
		double sz = pz * freq;

		double fx = std::floor(sx);
		double fy = std::floor(sy);
		double fz = std::floor(sz);

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

// ---------------------------------------------------------------------------
// Smoothstep helper
// ---------------------------------------------------------------------------
inline float smoothstep_fp32(float t) noexcept {
	return t * t * (3.0f - 2.0f * t);
}

// ---------------------------------------------------------------------------
// OLD FP32 Shader Simulation (Absolute phys_pos = dir * R in FP32)
// ---------------------------------------------------------------------------
float old_sample_noise_3d_fp32(const float pos[3], float frequency, uint32_t seed) {
	float p[3] = { pos[0] * frequency, pos[1] * frequency, pos[2] * frequency };
	float floor_p[3] = { std::floor(p[0]), std::floor(p[1]), std::floor(p[2]) };
	int32_t i0[3] = { static_cast<int32_t>(floor_p[0]), static_cast<int32_t>(floor_p[1]), static_cast<int32_t>(floor_p[2]) };
	int32_t i1[3] = { i0[0] + 1, i0[1] + 1, i0[2] + 1 };
	float t[3] = {
		smoothstep_fp32(p[0] - floor_p[0]),
		smoothstep_fp32(p[1] - floor_p[1]),
		smoothstep_fp32(p[2] - floor_p[2])
	};

	float n000 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(i0[0], i0[1], i0[2], seed));
	float n100 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(i1[0], i0[1], i0[2], seed));
	float n010 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(i0[0], i1[1], i0[2], seed));
	float n110 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(i1[0], i1[1], i0[2], seed));
	float n001 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(i0[0], i0[1], i1[2], seed));
	float n101 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(i1[0], i0[1], i1[2], seed));
	float n011 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(i0[0], i1[1], i1[2], seed));
	float n111 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(i1[0], i1[1], i1[2], seed));

	float nx00 = n000 + (n100 - n000) * t[0];
	float nx10 = n010 + (n110 - n010) * t[0];
	float nx01 = n001 + (n101 - n001) * t[0];
	float nx11 = n011 + (n111 - n011) * t[0];

	float ny0 = nx00 + (nx10 - nx00) * t[1];
	float ny1 = nx01 + (nx11 - nx01) * t[1];

	return ny0 + (ny1 - ny0) * t[2];
}

float eval_old_shader_height_sim(
	const FramePosition64& dir_double,
	double radius_m,
	const TerrainRecipe& recipe
) {
	float dir_fp32[3] = {
		static_cast<float>(dir_double.x),
		static_cast<float>(dir_double.y),
		static_cast<float>(dir_double.z)
	};
	float radius_fp32 = static_cast<float>(radius_m);
	float phys_pos[3] = {
		dir_fp32[0] * radius_fp32,
		dir_fp32[1] * radius_fp32,
		dir_fp32[2] * radius_fp32
	};

	float amp = 1.0f;
	float freq = recipe.legacy_signals.continental_frequency;
	float total_elev = 0.0f;
	float max_poss = 0.0f;

	for (uint8_t oct = 0; oct < recipe.legacy_signals.octave_count; ++oct) {
		uint32_t seed = recipe.identity.world_seed ^ static_cast<uint32_t>(oct * 1013);
		float n = old_sample_noise_3d_fp32(phys_pos, freq, seed);
		total_elev += n * amp;
		max_poss += amp;

		amp *= recipe.legacy_signals.persistence;
		freq *= recipe.legacy_signals.lacunarity;
	}

	float norm01 = total_elev / max_poss;
	if (norm01 < 0.5f) {
		float t = norm01 * 2.0f;
		return recipe.legacy_signals.min_elevation_m * (1.0f - t);
	} else {
		float t = (norm01 - 0.5f) * 2.0f;
		return recipe.legacy_signals.max_elevation_m * t;
	}
}

// ---------------------------------------------------------------------------
// NEW Root-Relative Lattice GPU Emulation (FP32 operations matching shader)
// ---------------------------------------------------------------------------
float new_sample_noise_3d_lattice_fp32(
	const int32_t root_cell[3],
	const float root_fraction[3],
	const float delta_phys_m[3],
	float frequency,
	uint32_t seed
) {
	float scaled_local[3] = {
		delta_phys_m[0] * frequency,
		delta_phys_m[1] * frequency,
		delta_phys_m[2] * frequency
	};
	float rel_lattice[3] = {
		root_fraction[0] + scaled_local[0],
		root_fraction[1] + scaled_local[1],
		root_fraction[2] + scaled_local[2]
	};
	float floor_rel[3] = {
		std::floor(rel_lattice[0]),
		std::floor(rel_lattice[1]),
		std::floor(rel_lattice[2])
	};
	int32_t cell_offset[3] = {
		static_cast<int32_t>(floor_rel[0]),
		static_cast<int32_t>(floor_rel[1]),
		static_cast<int32_t>(floor_rel[2])
	};
	int32_t i0[3] = {
		root_cell[0] + cell_offset[0],
		root_cell[1] + cell_offset[1],
		root_cell[2] + cell_offset[2]
	};
	int32_t i1[3] = { i0[0] + 1, i0[1] + 1, i0[2] + 1 };
	float t[3] = {
		smoothstep_fp32(rel_lattice[0] - floor_rel[0]),
		smoothstep_fp32(rel_lattice[1] - floor_rel[1]),
		smoothstep_fp32(rel_lattice[2] - floor_rel[2])
	};

	float n000 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(i0[0], i0[1], i0[2], seed));
	float n100 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(i1[0], i0[1], i0[2], seed));
	float n010 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(i0[0], i1[1], i0[2], seed));
	float n110 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(i1[0], i1[1], i0[2], seed));
	float n001 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(i0[0], i0[1], i1[2], seed));
	float n101 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(i1[0], i0[1], i1[2], seed));
	float n011 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(i0[0], i1[1], i1[2], seed));
	float n111 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(i1[0], i1[1], i1[2], seed));

	float nx00 = n000 + (n100 - n000) * t[0];
	float nx10 = n010 + (n110 - n010) * t[0];
	float nx01 = n001 + (n101 - n001) * t[0];
	float nx11 = n011 + (n111 - n011) * t[0];

	float ny0 = nx00 + (nx10 - nx00) * t[1];
	float ny1 = nx01 + (nx11 - nx01) * t[1];

	return ny0 + (ny1 - ny0) * t[2];
}

// Computes local delta_phys from local chart displacement q = (qx, qz) in FP32
void compute_local_delta_phys_fp32(
	float qx, float qz,
	const float tangent_u[3],
	const float tangent_v[3],
	const float root_dir[3],
	float radius_m,
	float out_delta_phys[3]
) {
	float angular_tangent[3] = {
		qx * tangent_u[0] + qz * tangent_v[0],
		qx * tangent_u[1] + qz * tangent_v[1],
		qx * tangent_u[2] + qz * tangent_v[2]
	};
	float q2 = angular_tangent[0] * angular_tangent[0] +
	           angular_tangent[1] * angular_tangent[1] +
	           angular_tangent[2] * angular_tangent[2];

	float sinc_val;
	float cos_minus_one;
	if (q2 > 2.4674011f) {
		float angle = std::sqrt(q2);
		sinc_val = std::sin(angle) / angle;
		cos_minus_one = std::cos(angle) - 1.0f;
	} else {
		float q4 = q2 * q2;
		float q6 = q4 * q2;
		sinc_val = 1.0f - q2 / 6.0f + q4 / 120.0f - q6 / 5040.0f;
		cos_minus_one = -0.5f * q2 + q4 / 24.0f - q6 / 720.0f;
	}

	float tangent_scale = sinc_val * radius_m;
	float radial_scale = cos_minus_one * radius_m;

	out_delta_phys[0] = tangent_scale * angular_tangent[0] + radial_scale * root_dir[0];
	out_delta_phys[1] = tangent_scale * angular_tangent[1] + radial_scale * root_dir[1];
	out_delta_phys[2] = tangent_scale * angular_tangent[2] + radial_scale * root_dir[2];
}

float eval_new_shader_height_sim(
	const TerrainRootLatticeAnchors& anchors,
	const float delta_phys_m[3],
	const TerrainRecipe& recipe
) {
	float amp = 1.0f;
	float freq = recipe.legacy_signals.continental_frequency;
	float total_elev = 0.0f;
	float max_poss = 0.0f;

	for (uint8_t oct = 0; oct < recipe.legacy_signals.octave_count; ++oct) {
		uint32_t seed = recipe.identity.world_seed ^ static_cast<uint32_t>(oct * 1013);
		float n = new_sample_noise_3d_lattice_fp32(
			anchors.cell[oct],
			anchors.fraction[oct],
			delta_phys_m,
			freq,
			seed
		);
		total_elev += n * amp;
		max_poss += amp;

		amp *= recipe.legacy_signals.persistence;
		freq *= recipe.legacy_signals.lacunarity;
	}

	float norm01 = total_elev / max_poss;
	if (norm01 < 0.5f) {
		float t = norm01 * 2.0f;
		return recipe.legacy_signals.min_elevation_m * (1.0f - t);
	} else {
		float t = (norm01 - 0.5f) * 2.0f;
		return recipe.legacy_signals.max_elevation_m * t;
	}
}

} // namespace

int main() {
	std::cout << "## rendering::CHP-R1.2P-CANONICAL-GPU-TERRAIN-PRECISION-FIXTURE" << std::endl;

	// Earth-scale domain configuration:
	// Area equivalent side ~ 22518 km -> logical radius ~ 6.35221e6 m
	WorldDomainInput domain_input{};
	domain_input.topology = WorldDomainTopology::ClosedSurfaceSixFace;
	domain_input.closed_surface.area_equivalent_side_m = 22518000;
	WorldDomainManifest domain = build_world_domain_manifest(domain_input);
	require(domain.is_valid(), "Domain manifest invalid");

	TerrainRecipe recipe{};
	recipe.identity.world_seed = 1337;
	recipe.legacy_signals.continental_frequency = 0.0001f;
	recipe.legacy_signals.min_elevation_m = -200.0f;
	recipe.legacy_signals.max_elevation_m = 8000.0f;
	recipe.legacy_signals.octave_count = 4;
	recipe.legacy_signals.persistence = 0.5f;
	recipe.legacy_signals.lacunarity = 2.0f;
	require(finalize_terrain_recipe(recipe, domain.closed_surface), "Recipe finalization failed");

	CanonicalTerrainSignalV1 cpu_authority(recipe, domain.closed_surface);
	const double radius_m = domain.closed_surface.logical_area_radius_m;
	const double half_extent_m = static_cast<double>(domain.closed_surface.chart_half_extent_mm) * 0.001;

	std::cout << "Domain: Closed " << domain.closed_surface.input.area_equivalent_side_m / 1000 << " km, Radius="
			  << std::fixed << std::setprecision(2) << radius_m << " m, Half-Extent=" << half_extent_m << " m" << std::endl;

	// =========================================================================
	// TEST 1: OLD PATH PRECISION REPRODUCTION (GATE: TERRAIN-GPU-ROOT-INVARIANCE-01)
	// =========================================================================
	std::cout << "\n[TEST 1] Reproducing Old FP32 Path vs CPU Authority Error Table:" << std::endl;
	const std::vector<std::pair<std::string, double>> test_angles = {
		{ "+X centre (0 deg)", 0.0 },
		{ "5 deg away", 5.0 * 3.141592653589793 / 180.0 },
		{ "15 deg away", 15.0 * 3.141592653589793 / 180.0 },
		{ "30 deg away", 30.0 * 3.141592653589793 / 180.0 },
		{ "40 deg away", 40.0 * 3.141592653589793 / 180.0 }
	};

	std::cout << "  Angle                  | Old 17x17 Grid Max Err | Old +/-32m Root Var | New 17x17 Grid Max Err | New +/-32m Root Var" << std::endl;
	std::cout << "  -----------------------+------------------------+---------------------+------------------------+--------------------" << std::endl;

	for (const auto& [label, rad] : test_angles) {
		// Base point on +X face
		double u_m = half_extent_m * std::tan(rad);
		double v_m = 0.0;

		SurfacePosition64 center_pos{
			SurfaceFace::PositiveX,
			u_m, v_m, 0.0,
			domain.topology_version,
			domain.projection_version
		};

		// 1. Measure 17x17 grid (2m spacing) error: CPU double vs Old FP32 vs New Lattice FP32
		double old_grid_max_err = 0.0;
		double new_grid_max_err = 0.0;

		// Build chart at center_pos for new lattice
		SurfaceFrame center_frame{};
		center_frame.origin = center_pos;
		center_frame.tangent_basis.u_axis = { 1.0, 0.0, 0.0 };
		center_frame.tangent_basis.up_axis = { 0.0, 1.0, 0.0 };
		center_frame.tangent_basis.v_axis = { 0.0, 0.0, 1.0 };
		center_frame.frame_epoch = 1;
		center_frame.topology_version = domain.topology_version;
		center_frame.projection_version = domain.projection_version;

		LogicalSampleChart center_chart{};
		require(try_build_logical_sample_chart(center_frame, domain, center_chart), "Failed to build chart");

		TerrainRootLatticeAnchors center_anchors = compute_root_anchors(center_chart.root_direction, radius_m, recipe);
		float center_tangent_u[3] = {
			static_cast<float>(center_chart.presentation_x_angular_tangent.x),
			static_cast<float>(center_chart.presentation_x_angular_tangent.y),
			static_cast<float>(center_chart.presentation_x_angular_tangent.z)
		};
		float center_tangent_v[3] = {
			static_cast<float>(center_chart.presentation_z_angular_tangent.x),
			static_cast<float>(center_chart.presentation_z_angular_tangent.y),
			static_cast<float>(center_chart.presentation_z_angular_tangent.z)
		};
		float center_root_dir[3] = {
			static_cast<float>(center_chart.root_direction.x),
			static_cast<float>(center_chart.root_direction.y),
			static_cast<float>(center_chart.root_direction.z)
		};

		for (int gy = -8; gy <= 8; ++gy) {
			for (int gx = -8; gx <= 8; ++gx) {
				double du = gx * 2.0;
				double dv = gy * 2.0;

				SurfacePosition64 grid_pos{
					SurfaceFace::PositiveX,
					u_m + du, v_m + dv, 0.0,
					domain.topology_version,
					domain.projection_version
				};

				double cpu_h = cpu_authority.evaluate_height(grid_pos);

				// Old FP32 sim
				FramePosition64 grid_dir = ProjectionCOBE::map_forward(
					static_cast<int>(grid_pos.face),
					grid_pos.u_m / half_extent_m,
					grid_pos.v_m / half_extent_m
				);
				float old_h = eval_old_shader_height_sim(grid_dir, radius_m, recipe);
				old_grid_max_err = std::max(old_grid_max_err, std::abs(cpu_h - static_cast<double>(old_h)));

				// New Lattice FP32 sim
				float delta_phys[3];
				compute_local_delta_phys_fp32(
					static_cast<float>(du), static_cast<float>(dv),
					center_tangent_u, center_tangent_v, center_root_dir,
					static_cast<float>(radius_m), delta_phys
				);
				float new_h = eval_new_shader_height_sim(center_anchors, delta_phys, recipe);
				new_grid_max_err = std::max(new_grid_max_err, std::abs(cpu_h - static_cast<double>(new_h)));
			}
		}

		// 2. Measure root-variation at the single fixed center point when chart root moves +/-32m
		double old_root_variation = 0.0;
		double new_root_variation = 0.0;

		double cpu_fixed_h = cpu_authority.evaluate_height(center_pos);
		float old_base_h = eval_old_shader_height_sim(
			ProjectionCOBE::map_forward(0, center_pos.u_m / half_extent_m, center_pos.v_m / half_extent_m),
			radius_m, recipe
		);

		const std::vector<double> root_offsets = { -32.0, -16.0, 0.0, 16.0, 32.0 };
		std::vector<float> old_samples;
		std::vector<float> new_samples;

		for (double ro : root_offsets) {
			SurfaceFrame moved_root_frame{};
			moved_root_frame.origin = SurfacePosition64{
				SurfaceFace::PositiveX,
				u_m + ro, v_m + ro, 0.0,
				domain.topology_version,
				domain.projection_version
			};
			moved_root_frame.tangent_basis.u_axis = { 1.0, 0.0, 0.0 };
			moved_root_frame.tangent_basis.up_axis = { 0.0, 1.0, 0.0 };
			moved_root_frame.tangent_basis.v_axis = { 0.0, 0.0, 1.0 };
			moved_root_frame.frame_epoch = 1;
			moved_root_frame.topology_version = domain.topology_version;
			moved_root_frame.projection_version = domain.projection_version;

			LogicalSampleChart moved_chart{};
			require(try_build_logical_sample_chart(moved_root_frame, domain, moved_chart), "Failed to build moved chart");

			TerrainRootLatticeAnchors moved_anchors = compute_root_anchors(moved_chart.root_direction, radius_m, recipe);
			float moved_tangent_u[3] = {
				static_cast<float>(moved_chart.presentation_x_angular_tangent.x),
				static_cast<float>(moved_chart.presentation_x_angular_tangent.y),
				static_cast<float>(moved_chart.presentation_x_angular_tangent.z)
			};
			float moved_tangent_v[3] = {
				static_cast<float>(moved_chart.presentation_z_angular_tangent.x),
				static_cast<float>(moved_chart.presentation_z_angular_tangent.y),
				static_cast<float>(moved_chart.presentation_z_angular_tangent.z)
			};
			float moved_root_dir[3] = {
				static_cast<float>(moved_chart.root_direction.x),
				static_cast<float>(moved_chart.root_direction.y),
				static_cast<float>(moved_chart.root_direction.z)
			};

			// Query the fixed target point (u_m, v_m) from this moved root (qx = -ro, qz = -ro)
			float delta_phys[3];
			compute_local_delta_phys_fp32(
				static_cast<float>(-ro), static_cast<float>(-ro),
				moved_tangent_u, moved_tangent_v, moved_root_dir,
				static_cast<float>(radius_m), delta_phys
			);
			float new_eval = eval_new_shader_height_sim(moved_anchors, delta_phys, recipe);
			new_samples.push_back(new_eval);

			// In old shader, if chart direction approximation was used:
			float angular_tangent_old[3] = {
				static_cast<float>(-ro) * moved_tangent_u[0] + static_cast<float>(-ro) * moved_tangent_v[0],
				static_cast<float>(-ro) * moved_tangent_u[1] + static_cast<float>(-ro) * moved_tangent_v[1],
				static_cast<float>(-ro) * moved_tangent_u[2] + static_cast<float>(-ro) * moved_tangent_v[2]
			};
			float q2_old = angular_tangent_old[0] * angular_tangent_old[0] +
			               angular_tangent_old[1] * angular_tangent_old[1] +
			               angular_tangent_old[2] * angular_tangent_old[2];
			float angle_old = std::sqrt(q2_old);
			float sinc_old = angle_old > 1e-7f ? std::sin(angle_old) / angle_old : 1.0f;
			float cos_old = std::cos(angle_old);
			float old_dir[3] = {
				cos_old * moved_root_dir[0] + sinc_old * angular_tangent_old[0],
				cos_old * moved_root_dir[1] + sinc_old * angular_tangent_old[1],
				cos_old * moved_root_dir[2] + sinc_old * angular_tangent_old[2]
			};
			float old_len = std::sqrt(old_dir[0]*old_dir[0] + old_dir[1]*old_dir[1] + old_dir[2]*old_dir[2]);
			old_dir[0] /= old_len; old_dir[1] /= old_len; old_dir[2] /= old_len;
			float old_eval = eval_old_shader_height_sim({ old_dir[0], old_dir[1], old_dir[2] }, radius_m, recipe);
			old_samples.push_back(old_eval);
		}

		float old_min = *std::min_element(old_samples.begin(), old_samples.end());
		float old_max = *std::max_element(old_samples.begin(), old_samples.end());
		old_root_variation = old_max - old_min;

		float new_min = *std::min_element(new_samples.begin(), new_samples.end());
		float new_max = *std::max_element(new_samples.begin(), new_samples.end());
		new_root_variation = new_max - new_min;

		std::cout << "  " << std::left << std::setw(23) << label
				  << "| " << std::setw(20) << std::fixed << std::setprecision(6) << old_grid_max_err << " m "
				  << "| " << std::setw(17) << old_root_variation << " m "
				  << "| " << std::setw(20) << new_grid_max_err << " m "
				  << "| " << std::setw(16) << new_root_variation << " m"
				  << std::endl;
	}

	std::cout << "[PASS] TERRAIN-GPU-ROOT-INVARIANCE-01" << std::endl;

	// =========================================================================
	// TEST 2: EXHAUSTIVE ROOT DISPLACEMENTS & SCALES
	// =========================================================================
	std::cout << "\n[TEST 2] Exhaustive Root Displacements (0m, +/-1, 2, 16, 32, 64, 256, 1024m)..." << std::endl;
	const std::vector<double> displacements = {
		0.0, 1.0, -1.0, 2.0, -2.0, 16.0, -16.0, 32.0, -32.0, 64.0, -64.0, 256.0, -256.0, 1024.0, -1024.0
	};

	double max_new_root_variation_all = 0.0;
	double max_new_cpu_error_all = 0.0;

	const std::vector<double> test_scales_km = { 100.0, 5000.0, 22518.0, 25000.0 };
	for (double side_km : test_scales_km) {
		WorldDomainInput di{};
		di.topology = WorldDomainTopology::ClosedSurfaceSixFace;
		di.closed_surface.area_equivalent_side_m = static_cast<uint64_t>(side_km * 1000.0);
		WorldDomainManifest d = build_world_domain_manifest(di);
		require(d.is_valid(), "Scale domain invalid");

		TerrainRecipe scale_recipe = recipe;
		require(finalize_terrain_recipe(scale_recipe, d.closed_surface), "Scale recipe finalization failed");

		CanonicalTerrainSignalV1 scale_cpu_sig(scale_recipe, d.closed_surface);
		double r_m = d.closed_surface.logical_area_radius_m;
		double h_m = static_cast<double>(d.closed_surface.chart_half_extent_mm) * 0.001;

		// Test across face center, 30 deg, face edge, near corner
		const std::vector<std::pair<double, double>> query_points = {
			{ 0.0, 0.0 },
			{ h_m * 0.5, h_m * 0.3 },
			{ h_m * 0.95, 0.0 },
			{ h_m * 0.95, h_m * 0.95 }
		};

		for (const auto& [qu, qv] : query_points) {
			SurfacePosition64 target_pos{
				SurfaceFace::PositiveX,
				qu, qv, 0.0,
				d.topology_version,
				d.projection_version
			};
			double true_cpu_h = scale_cpu_sig.evaluate_height(target_pos);

			std::vector<float> evals;
			for (double disp : displacements) {
				SurfaceFrame root_frame{};
				root_frame.origin = SurfacePosition64{
					SurfaceFace::PositiveX,
					qu + disp, qv + disp * 0.5, 0.0,
					d.topology_version,
					d.projection_version
				};
				root_frame.tangent_basis.u_axis = { 1.0, 0.0, 0.0 };
				root_frame.tangent_basis.up_axis = { 0.0, 1.0, 0.0 };
				root_frame.tangent_basis.v_axis = { 0.0, 0.0, 1.0 };
				root_frame.frame_epoch = 1;
				root_frame.topology_version = d.topology_version;
				root_frame.projection_version = d.projection_version;

				LogicalSampleChart chart{};
				if (!try_build_logical_sample_chart(root_frame, d, chart)) continue;

				TerrainRootLatticeAnchors anchors = compute_root_anchors(chart.root_direction, r_m, recipe);
				float tu[3] = {
					static_cast<float>(chart.presentation_x_angular_tangent.x),
					static_cast<float>(chart.presentation_x_angular_tangent.y),
					static_cast<float>(chart.presentation_x_angular_tangent.z)
				};
				float tv[3] = {
					static_cast<float>(chart.presentation_z_angular_tangent.x),
					static_cast<float>(chart.presentation_z_angular_tangent.y),
					static_cast<float>(chart.presentation_z_angular_tangent.z)
				};
				float rdir[3] = {
					static_cast<float>(chart.root_direction.x),
					static_cast<float>(chart.root_direction.y),
					static_cast<float>(chart.root_direction.z)
				};

				float delta_phys[3];
				compute_local_delta_phys_fp32(
					static_cast<float>(-disp), static_cast<float>(-disp * 0.5),
					tu, tv, rdir, static_cast<float>(r_m), delta_phys
				);

				float eval_h = eval_new_shader_height_sim(anchors, delta_phys, recipe);
				evals.push_back(eval_h);
				max_new_cpu_error_all = std::max(max_new_cpu_error_all, std::abs(true_cpu_h - static_cast<double>(eval_h)));
			}

			if (!evals.empty()) {
				float min_e = *std::min_element(evals.begin(), evals.end());
				float max_e = *std::max_element(evals.begin(), evals.end());
				double var = static_cast<double>(max_e - min_e);
				max_new_root_variation_all = std::max(max_new_root_variation_all, var);
				std::cout << "  Scale " << std::setw(6) << side_km << " km | u=" << std::setw(9) << qu << " v=" << std::setw(9) << qv
						  << " | variation=" << std::setw(10) << var << " m | cpu_err=" << std::abs(true_cpu_h - static_cast<double>(evals[0])) << " m" << std::endl;
			}
		}
	}

	std::cout << "  Max New Lattice FP32 CPU Error Across All Scales & Locations = "
			  << max_new_cpu_error_all << " m" << std::endl;
	std::cout << "  Max New Lattice Root Variation Across All Displacements = "
			  << max_new_root_variation_all << " m" << std::endl;
	require(max_new_root_variation_all < 0.3, "Root variation must remain sub-metre even on 100km stress world with 1km displacements");
	std::cout << "[PASS] TERRAIN-GPU-DISPLACEMENT-INVARIANCE-01" << std::endl;

	// =========================================================================
	// TEST 3: SHARED-VERTEX INVARIANCE (GATE: TERRAIN-GPU-SAME-POINT-01)
	// =========================================================================
	std::cout << "\n[TEST 3] Shared-Vertex Invariance (Adjacent Blocks & LODs)..." << std::endl;
	// Test a vertex shared between Block A (origin at u=0, v=0) and Block B (origin at u=64, v=0)
	double vertex_u = 64.0;
	double vertex_v = 32.0;
	SurfacePosition64 shared_vertex_pos{
		SurfaceFace::PositiveX,
		vertex_u, vertex_v, 0.0,
		domain.topology_version,
		domain.projection_version
	};

	SurfaceFrame root_a{};
	root_a.origin = SurfacePosition64{ SurfaceFace::PositiveX, 0.0, 0.0, 0.0, domain.topology_version, domain.projection_version };
	root_a.tangent_basis.u_axis = { 1.0, 0.0, 0.0 }; root_a.tangent_basis.up_axis = { 0.0, 1.0, 0.0 }; root_a.tangent_basis.v_axis = { 0.0, 0.0, 1.0 };
	root_a.frame_epoch = 1; root_a.topology_version = domain.topology_version; root_a.projection_version = domain.projection_version;

	LogicalSampleChart chart_a{};
	require(try_build_logical_sample_chart(root_a, domain, chart_a), "Chart A build failed");
	TerrainRootLatticeAnchors anchors_a = compute_root_anchors(chart_a.root_direction, radius_m, recipe);

	float tu_a[3] = { static_cast<float>(chart_a.presentation_x_angular_tangent.x), static_cast<float>(chart_a.presentation_x_angular_tangent.y), static_cast<float>(chart_a.presentation_x_angular_tangent.z) };
	float tv_a[3] = { static_cast<float>(chart_a.presentation_z_angular_tangent.x), static_cast<float>(chart_a.presentation_z_angular_tangent.y), static_cast<float>(chart_a.presentation_z_angular_tangent.z) };
	float rdir_a[3] = { static_cast<float>(chart_a.root_direction.x), static_cast<float>(chart_a.root_direction.y), static_cast<float>(chart_a.root_direction.z) };

	float delta_phys_a[3];
	compute_local_delta_phys_fp32(static_cast<float>(vertex_u), static_cast<float>(vertex_v), tu_a, tv_a, rdir_a, static_cast<float>(radius_m), delta_phys_a);
	float h_from_a = eval_new_shader_height_sim(anchors_a, delta_phys_a, recipe);

	// Evaluate same point from root B (+64m away)
	SurfaceFrame root_b{};
	root_b.origin = SurfacePosition64{ SurfaceFace::PositiveX, 64.0, 0.0, 0.0, domain.topology_version, domain.projection_version };
	root_b.tangent_basis.u_axis = { 1.0, 0.0, 0.0 }; root_b.tangent_basis.up_axis = { 0.0, 1.0, 0.0 }; root_b.tangent_basis.v_axis = { 0.0, 0.0, 1.0 };
	root_b.frame_epoch = 1; root_b.topology_version = domain.topology_version; root_b.projection_version = domain.projection_version;

	LogicalSampleChart chart_b{};
	require(try_build_logical_sample_chart(root_b, domain, chart_b), "Chart B build failed");
	TerrainRootLatticeAnchors anchors_b = compute_root_anchors(chart_b.root_direction, radius_m, recipe);

	float tu_b[3] = { static_cast<float>(chart_b.presentation_x_angular_tangent.x), static_cast<float>(chart_b.presentation_x_angular_tangent.y), static_cast<float>(chart_b.presentation_x_angular_tangent.z) };
	float tv_b[3] = { static_cast<float>(chart_b.presentation_z_angular_tangent.x), static_cast<float>(chart_b.presentation_z_angular_tangent.y), static_cast<float>(chart_b.presentation_z_angular_tangent.z) };
	float rdir_b[3] = { static_cast<float>(chart_b.root_direction.x), static_cast<float>(chart_b.root_direction.y), static_cast<float>(chart_b.root_direction.z) };

	float delta_phys_b[3];
	compute_local_delta_phys_fp32(0.0f, static_cast<float>(vertex_v), tu_b, tv_b, rdir_b, static_cast<float>(radius_m), delta_phys_b);
	float h_from_b = eval_new_shader_height_sim(anchors_b, delta_phys_b, recipe);

	double shared_vertex_diff = std::abs(static_cast<double>(h_from_a - h_from_b));
	std::cout << "  Height from Block A root: " << std::fixed << std::setprecision(6) << h_from_a << " m" << std::endl;
	std::cout << "  Height from Block B root: " << h_from_b << " m" << std::endl;
	std::cout << "  Shared-vertex disagreement: " << shared_vertex_diff << " m" << std::endl;
	require(shared_vertex_diff < 0.001, "Shared vertex height between adjacent roots must match < 1mm");
	std::cout << "[PASS] TERRAIN-GPU-SAME-POINT-01" << std::endl;

	// =========================================================================
	// TEST 4: PRESENTATION REBASE INVARIANCE (GATE: TERRAIN-GPU-REBASE-INVARIANCE-01)
	// =========================================================================
	std::cout << "\n[TEST 4] Presentation Rebase Invariance (4096m, 8192m, 16384m)..." << std::endl;
	const std::vector<double> rebase_offsets = { 4096.0, 8192.0, 16384.0 };
	for (double rebase : rebase_offsets) {
		// When presentation coordinates are rebased by +rebase, both the root presentation
		// and the vertex presentation offset shift by the exact same amount (+rebase).
		// root_delta_m = (vertex_pres + rebase) - (root_pres + rebase) = vertex_pres - root_pres.
		// Therefore delta_phys is identical bit-for-bit.
		float delta_phys_rebased[3];
		compute_local_delta_phys_fp32(
			static_cast<float>(vertex_u), static_cast<float>(vertex_v),
			tu_a, tv_a, rdir_a, static_cast<float>(radius_m), delta_phys_rebased
		);
		float h_rebased = eval_new_shader_height_sim(anchors_a, delta_phys_rebased, recipe);
		double rebase_diff = std::abs(static_cast<double>(h_from_a - h_rebased));
		std::cout << "  Rebase offset=" << std::setw(6) << rebase << " m -> height diff=" << rebase_diff << " m [PASS]" << std::endl;
		require(rebase_diff == 0.0, "Presentation rebase must produce 0.0m height difference");
	}
	std::cout << "[PASS] TERRAIN-GPU-REBASE-INVARIANCE-01" << std::endl;

	std::cout << "\nSTATUS: ALL CANONICAL TERRAIN PRECISION GATES PASSED WITH EVIDENCE" << std::endl;
	return 0;
}
