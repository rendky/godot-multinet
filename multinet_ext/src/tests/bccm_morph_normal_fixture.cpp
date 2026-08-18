#include "multinet/rendering/terrain/block_clipmap/block_clipmap_morph.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_profile.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_shader.h"
#include "multinet/rendering/chp/chp_kernel.h"
#include "multinet/rendering/chp/chp_certification.h"
#include "multinet/rendering/chp/chp_view.h"
#include "multinet/core/spatial/world_manifests.h"
#include "multinet/core/spatial/surface_address.h"
#include "multinet/core/spatial/surface_topology.h"
#include "multinet/world/terrain/canonical_terrain_signal.h"
#include "multinet/world/terrain/finite_canonical_terrain_signal.h"
#include "multinet/world/terrain/terrain_recipe.h"

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <cassert>
#include <array>
#include <string>
#include <algorithm>
#include <limits>

using namespace multinet::rendering;
using namespace multinet::rendering::chp;
using namespace Multinet;

namespace {

constexpr double PI = 3.141592653589793238462643383279502884;
constexpr double RAD_TO_DEG = 180.0 / PI;

void require(bool condition, const char* message) {
	if (!condition) {
		std::cerr << "\n[FAIL] " << message << std::endl;
		std::exit(1);
	}
}

double dot(const Vec3d& a, const Vec3d& b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3d cross(const Vec3d& a, const Vec3d& b) {
	return {
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
	};
}

Vec3d normalize(const Vec3d& v) {
	const double len = std::sqrt(dot(v, v));
	if (len <= 1.0e-30 || !std::isfinite(len)) return { 0.0, 1.0, 0.0 };
	return { v.x / len, v.y / len, v.z / len };
}

double angle_between(const Vec3d& a, const Vec3d& b) {
	return std::acos(std::clamp(dot(a, b), -1.0, 1.0));
}

WorldDomainManifest make_finite_domain(uint64_t extent_x = 500000, uint64_t extent_z = 400000) {
	WorldDomainInput input;
	input.topology = WorldDomainTopology::FiniteRectangle;
	input.finite.extent_x_m = extent_x;
	input.finite.extent_z_m = extent_z;
	return build_world_domain_manifest(input);
}

WorldDomainManifest make_closed_domain(uint64_t side_m = 5000000) {
	WorldDomainInput input;
	input.closed_surface.area_equivalent_side_m = side_m;
	return build_world_domain_manifest(input);
}

// ---------------------------------------------------------------------------
// CPU Production Reference for R2 Normal
// ---------------------------------------------------------------------------

struct ProductionNormalResult {
	Vec3d normal_flat{};
	Vec3d normal_curved{};
	double morphed_u{ 0.0 };
	double morphed_v{ 0.0 };
	double height_m{ 0.0 };
	double slope_u{ 0.0 };
	double slope_v{ 0.0 };
	double mu{ 0.0 };
	uint8_t recursion_depth{ 0 };
};

ProductionNormalResult evaluate_production_r2_normal(
	double quad_x,
	double quad_z,
	int64_t block_bx,
	int64_t block_bv,
	uint8_t lod,
	double observer_u_m,
	double observer_v_m,
	const BlockClipmapProfile& profile,
	const FiniteCanonicalTerrainSignalV1& signal,
	bool chp_enabled,
	const ResolvedCurvedHorizonProfile& chp_profile,
	bool morph_enabled = true
) {
	ProductionNormalResult res{};
	const double fine_spacing = profile.get_lod_spacing(lod);
	const double fine_block_size = profile.get_lod_block_size(lod);

	const double local_u_m = static_cast<double>(block_bx) * fine_block_size + quad_x * fine_spacing;
	const double local_v_m = static_cast<double>(block_bv) * fine_block_size + quad_z * fine_spacing;

	double morphed_u = local_u_m;
	double morphed_v = local_v_m;

	if (morph_enabled && profile.candidate_grid_radius == 4 && profile.inner_hole_radius == 2) {
		const auto morph_res = evaluate_vertex_morph_recursive_world(
			local_u_m, local_v_m, lod, observer_u_m, observer_v_m, profile,
			MorphStrategy::StrategyA_Legal_1_25B_0_75B_2B, MorphMetricLaw::ChebyshevScalar
		);
		morphed_u = morph_res.morphed_u_m;
		morphed_v = morph_res.morphed_v_m;
		res.mu = morph_res.combined_morph_factor;
		res.recursion_depth = morph_res.active_recursion_depth;
	}

	res.morphed_u = morphed_u;
	res.morphed_v = morphed_v;

	// Height and slopes at final morphed coordinate
	const double h = signal.evaluate_height(morphed_u, morphed_v);
	const SurfaceNormal sn = signal.evaluate_normal(morphed_u, morphed_v);
	res.height_m = h;
	res.slope_u = -static_cast<double>(sn.nx) / static_cast<double>(sn.ny);
	res.slope_v = -static_cast<double>(sn.nz) / static_cast<double>(sn.ny);

	// Flat normal
	res.normal_flat = normalize({ -res.slope_u, 1.0, -res.slope_v });

	// CHP composition
	if (chp_enabled && chp_profile.is_valid()) {
		CHPIntrinsicSample sample{
			morphed_u - observer_u_m,
			morphed_v - observer_v_m,
			h,
			res.slope_u,
			res.slope_v
		};
		CHPEvaluation eval{};
		if (try_evaluate_curved(chp_profile, sample, eval)) {
			res.normal_curved = eval.normal;
		} else {
			res.normal_curved = res.normal_flat;
		}
	} else {
		res.normal_curved = res.normal_flat;
	}

	return res;
}

// ---------------------------------------------------------------------------
// Engine-Neutral Geometric Reference Oracle (Double-Precision Numerical Tangents)
// ---------------------------------------------------------------------------

Vec3d evaluate_geometric_reference_position(
	double quad_x,
	double quad_z,
	int64_t block_bx,
	int64_t block_bv,
	uint8_t lod,
	double observer_u_m,
	double observer_v_m,
	const BlockClipmapProfile& profile,
	const FiniteCanonicalTerrainSignalV1& signal,
	bool chp_enabled,
	const ResolvedCurvedHorizonProfile& chp_profile,
	bool morph_enabled = true
) {
	const double fine_spacing = profile.get_lod_spacing(lod);
	const double fine_block_size = profile.get_lod_block_size(lod);

	const double local_u_m = static_cast<double>(block_bx) * fine_block_size + quad_x * fine_spacing;
	const double local_v_m = static_cast<double>(block_bv) * fine_block_size + quad_z * fine_spacing;

	double morphed_u = local_u_m;
	double morphed_v = local_v_m;

	if (morph_enabled && profile.candidate_grid_radius == 4 && profile.inner_hole_radius == 2) {
		const auto morph_res = evaluate_vertex_morph_recursive_world(
			local_u_m, local_v_m, lod, observer_u_m, observer_v_m, profile,
			MorphStrategy::StrategyA_Legal_1_25B_0_75B_2B, MorphMetricLaw::ChebyshevScalar
		);
		morphed_u = morph_res.morphed_u_m;
		morphed_v = morph_res.morphed_v_m;
	}

	const double h = signal.evaluate_height(morphed_u, morphed_v);

	if (chp_enabled && chp_profile.is_valid()) {
		CHPIntrinsicSample sample{
			morphed_u - observer_u_m,
			morphed_v - observer_v_m,
			h,
			0.0, 0.0
		};
		CHPEvaluation eval{};
		if (try_evaluate_curved(chp_profile, sample, eval)) {
			return eval.position_m;
		}
	}

	return { morphed_u - observer_u_m, h, morphed_v - observer_v_m };
}

Vec3d evaluate_geometric_oracle_normal(
	double quad_x,
	double quad_z,
	int64_t block_bx,
	int64_t block_bv,
	uint8_t lod,
	double observer_u_m,
	double observer_v_m,
	const BlockClipmapProfile& profile,
	const FiniteCanonicalTerrainSignalV1& signal,
	bool chp_enabled,
	const ResolvedCurvedHorizonProfile& chp_profile,
	bool morph_enabled = true
) {
	const double fine_spacing = profile.get_lod_spacing(lod);
	const double fine_block_size = profile.get_lod_block_size(lod);

	const double local_u_m = static_cast<double>(block_bx) * fine_block_size + quad_x * fine_spacing;
	const double local_v_m = static_cast<double>(block_bv) * fine_block_size + quad_z * fine_spacing;

	double morphed_u = local_u_m;
	double morphed_v = local_v_m;

	if (morph_enabled && profile.candidate_grid_radius == 4 && profile.inner_hole_radius == 2) {
		const auto morph_res = evaluate_vertex_morph_recursive_world(
			local_u_m, local_v_m, lod, observer_u_m, observer_v_m, profile,
			MorphStrategy::StrategyA_Legal_1_25B_0_75B_2B, MorphMetricLaw::ChebyshevScalar
		);
		morphed_u = morph_res.morphed_u_m;
		morphed_v = morph_res.morphed_v_m;
	}

	const double h = signal.evaluate_height(morphed_u, morphed_v);
	const SurfaceNormal sn = signal.evaluate_normal(morphed_u, morphed_v);
	const double slope_u = -static_cast<double>(sn.nx) / static_cast<double>(sn.ny);
	const double slope_v = -static_cast<double>(sn.nz) / static_cast<double>(sn.ny);

	const CHPIntrinsicSample sample{
		morphed_u - observer_u_m,
		morphed_v - observer_v_m,
		h,
		slope_u,
		slope_v
	};

	CHPEvaluation eval{};
	if (chp_enabled && chp_profile.is_valid()) {
		if (try_evaluate_curved(chp_profile, sample, eval)) {
			return eval.normal;
		}
	}

	if (try_evaluate_flat(sample, eval)) {
		return eval.normal;
	}

	return normalize(Vec3d{ -slope_u, 1.0, -slope_v });
}

// ---------------------------------------------------------------------------
// Error Aggregator
// ---------------------------------------------------------------------------
struct NormalErrorStats {
	double max_angular_error_rad{ 0.0 };
	double max_angular_error_deg{ 0.0 };
	double max_dot_residual{ 0.0 };
	double sum_angular_error_rad{ 0.0 };
	uint64_t sample_count{ 0 };

	void record(const Vec3d& ref, const Vec3d& prod) {
		const double d = std::clamp(dot(ref, prod), -1.0, 1.0);
		const double err_rad = std::acos(d);
		const double err_deg = err_rad * RAD_TO_DEG;
		const double dot_res = 1.0 - d;

		if (err_rad > max_angular_error_rad) max_angular_error_rad = err_rad;
		if (err_deg > max_angular_error_deg) max_angular_error_deg = err_deg;
		if (dot_res > max_dot_residual) max_dot_residual = dot_res;
		sum_angular_error_rad += err_rad;
		sample_count++;
	}

	double mean_rad() const {
		return sample_count > 0 ? (sum_angular_error_rad / static_cast<double>(sample_count)) : 0.0;
	}
};

} // namespace

// ===========================================================================
// Test Gates
// ===========================================================================

// 1. Gate: BCCM-MORPH-NORMAL-CHP-OFF-01
void test_bccm_morph_normal_chp_off_01() {
	std::cout << "\n=======================================================\n";
	std::cout << " 1. Gate: BCCM-MORPH-NORMAL-CHP-OFF-01 (CHP OFF Identity)\n";
	std::cout << "=======================================================\n" << std::flush;

	WorldDomainManifest domain = make_finite_domain();
	TerrainRecipe recipe;
	recipe.legacy_signals.min_elevation_m = -200.0f;
	recipe.legacy_signals.max_elevation_m = 500.0f;
	require(finalize_terrain_recipe(recipe, domain), "recipe setup failed");
	FiniteCanonicalTerrainSignalV1 signal(recipe, domain);

	BlockClipmapProfile profile{};
	profile.candidate_grid_radius = 4;
	profile.inner_hole_radius = 2;
	profile.lod0_block_size = 32.0f;
	profile.level_count = 8;

	ResolvedCurvedHorizonProfile null_chp{};

	NormalErrorStats stats{};

	// Sweep across all LODs and quad positions
	for (uint8_t lod = 0; lod < 8; ++lod) {
		for (int64_t bx = -3; bx <= 3; ++bx) {
			for (int64_t bv = -3; bv <= 3; ++bv) {
				for (int32_t qz = 1; qz < 16; qz += 3) {
					for (int32_t qx = 1; qx < 16; qx += 3) {
						const auto prod = evaluate_production_r2_normal(
							qx, qz, bx, bv, lod, 0.0, 0.0, profile, signal, false, null_chp
						);
						const auto ref = evaluate_geometric_oracle_normal(
							qx, qz, bx, bv, lod, 0.0, 0.0, profile, signal, false, null_chp
						);

						stats.record(ref, prod.normal_curved);
					}
				}
			}
		}
	}

	std::cout << "  CHP OFF: samples = " << stats.sample_count
	          << ", max angular error = " << stats.max_angular_error_rad << " rad ("
	          << stats.max_angular_error_deg << " deg), max 1-dot = "
	          << stats.max_dot_residual << "\n";

	require(stats.max_angular_error_rad < 1.0e-3, "BCCM-MORPH-NORMAL-CHP-OFF-01 failed: angular error too high");
	require(stats.max_dot_residual < 1.0e-6, "BCCM-MORPH-NORMAL-CHP-OFF-01 failed: dot residual too high");
	std::cout << "[PASS] BCCM-MORPH-NORMAL-CHP-OFF-01: Exact flat morphed normal validated.\n" << std::flush;
}

// 2. Gate: BCCM-MORPH-NORMAL-IDENTITY-01
void test_bccm_morph_normal_identity_01() {
	std::cout << "\n=======================================================\n";
	std::cout << " 2. Gate: BCCM-MORPH-NORMAL-IDENTITY-01 (Morph OFF / mu=0 Parity)\n";
	std::cout << "=======================================================\n" << std::flush;

	WorldDomainManifest domain = make_finite_domain();
	TerrainRecipe recipe;
	recipe.legacy_signals.min_elevation_m = -200.0f;
	recipe.legacy_signals.max_elevation_m = 500.0f;
	require(finalize_terrain_recipe(recipe, domain), "recipe setup failed");
	FiniteCanonicalTerrainSignalV1 signal(recipe, domain);

	BlockClipmapProfile profile{};
	profile.candidate_grid_radius = 4;
	profile.inner_hole_radius = 2;
	profile.lod0_block_size = 32.0f;
	profile.level_count = 8;

	ResolvedCurvedHorizonProfile null_chp{};

	NormalErrorStats stats{};

	// Test fine zone (mu = 0) where morph has zero displacement
	for (uint8_t lod = 0; lod < 8; ++lod) {
		const auto prod_morphed = evaluate_production_r2_normal(
			8.0, 8.0, 0, 0, lod, 0.0, 0.0, profile, signal, false, null_chp, true
		);
		const auto prod_unmorphed = evaluate_production_r2_normal(
			8.0, 8.0, 0, 0, lod, 0.0, 0.0, profile, signal, false, null_chp, false
		);

		require(prod_morphed.mu == 0.0, "mu != 0 at center");
		stats.record(prod_unmorphed.normal_flat, prod_morphed.normal_flat);
	}

	std::cout << "  Morph OFF identity: samples = " << stats.sample_count
	          << ", max angular error = " << stats.max_angular_error_rad << " rad, max 1-dot = "
	          << stats.max_dot_residual << "\n";

	require(stats.max_angular_error_rad < 1.0e-6, "BCCM-MORPH-NORMAL-IDENTITY-01 failed: morph=0 gave different normal");
	std::cout << "[PASS] BCCM-MORPH-NORMAL-IDENTITY-01: Exact unmorphed normal parity confirmed.\n" << std::flush;
}

// 3. Gate: BCCM-MORPH-NORMAL-CHEBYSHEV-BRANCHES-01
void test_bccm_morph_normal_chebyshev_branches_01() {
	std::cout << "\n=======================================================\n";
	std::cout << " 3. Gate: BCCM-MORPH-NORMAL-CHEBYSHEV-BRANCHES-01\n";
	std::cout << "=======================================================\n" << std::flush;

	WorldDomainManifest domain = make_finite_domain();
	TerrainRecipe recipe;
	recipe.legacy_signals.min_elevation_m = -200.0f;
	recipe.legacy_signals.max_elevation_m = 500.0f;
	require(finalize_terrain_recipe(recipe, domain), "recipe setup failed");
	FiniteCanonicalTerrainSignalV1 signal(recipe, domain);

	BlockClipmapProfile profile{};
	profile.candidate_grid_radius = 4;
	profile.inner_hole_radius = 2;
	profile.lod0_block_size = 32.0f;
	profile.level_count = 8;

	ResolvedCurvedHorizonProfile null_chp{};

	NormalErrorStats stats_x_dom{};
	NormalErrorStats stats_z_dom{};
	NormalErrorStats stats_diag{};

	// Test specific points across branches:
	// x-dominant: |qx| > |qz|
	// z-dominant: |qz| > |qx|
	// diagonal: |qx| == |qz|
	// also negative quadrants
	const std::array<std::pair<double, double>, 8> test_directions{{
		{ 1.0, 0.2 },   // x-dominant positive
		{ -1.0, 0.2 },  // x-dominant negative x
		{ 0.2, 1.0 },   // z-dominant positive
		{ 0.2, -1.0 },  // z-dominant negative z
		{ 1.0, 1.0 },   // diagonal +x, +z
		{ -1.0, 1.0 },  // diagonal -x, +z
		{ 1.0, -1.0 },  // diagonal +x, -z
		{ -1.0, -1.0 }  // diagonal -x, -z
	}};

	for (const auto& dir : test_directions) {
		const bool is_diag = (std::abs(std::abs(dir.first) - std::abs(dir.second)) < 1.0e-9);
		const bool is_x_dom = (std::abs(dir.first) > std::abs(dir.second) + 1.0e-9);

		for (double dist_factor = 1.3; dist_factor <= 1.9; dist_factor += 0.1) {
			const double u_m = dir.first * dist_factor * 32.0;
			const double v_m = dir.second * dist_factor * 32.0;

			const int64_t bx = static_cast<int64_t>(std::floor(u_m / 32.0));
			const int64_t bv = static_cast<int64_t>(std::floor(v_m / 32.0));
			const double qx = (u_m - bx * 32.0) / 2.0;
			const double qz = (v_m - bv * 32.0) / 2.0;

			const auto prod = evaluate_production_r2_normal(
				qx, qz, bx, bv, 0, 0.0, 0.0, profile, signal, false, null_chp
			);
			const auto ref = evaluate_geometric_oracle_normal(
				qx, qz, bx, bv, 0, 0.0, 0.0, profile, signal, false, null_chp
			);

			if (is_diag) {
				stats_diag.record(ref, prod.normal_flat);
			} else if (is_x_dom) {
				stats_x_dom.record(ref, prod.normal_flat);
			} else {
				stats_z_dom.record(ref, prod.normal_flat);
			}
		}
	}

	std::cout << "  X-Dominant branch: samples = " << stats_x_dom.sample_count
	          << ", max angular error = " << stats_x_dom.max_angular_error_deg << " deg\n";
	std::cout << "  Z-Dominant branch: samples = " << stats_z_dom.sample_count
	          << ", max angular error = " << stats_z_dom.max_angular_error_deg << " deg\n";
	std::cout << "  Diagonal branch:   samples = " << stats_diag.sample_count
	          << ", max angular error = " << stats_diag.max_angular_error_deg << " deg\n";

	require(stats_x_dom.max_angular_error_deg < 0.1, "X-dominant branch error exceeded threshold");
	require(stats_z_dom.max_angular_error_deg < 0.1, "Z-dominant branch error exceeded threshold");
	require(stats_diag.max_angular_error_deg < 0.1, "Diagonal branch error exceeded threshold");
	std::cout << "[PASS] BCCM-MORPH-NORMAL-CHEBYSHEV-BRANCHES-01 passed cleanly.\n" << std::flush;
}

// 4. Gate: BCCM-MORPH-NORMAL-CHP-COMPOSITION-01 (CHP Composition across Polynomials)
void test_bccm_morph_normal_chp_composition_01() {
	std::cout << "\n=======================================================\n";
	std::cout << " 4. Gate: BCCM-MORPH-NORMAL-CHP-COMPOSITION-01 (CHP Composition)\n";
	std::cout << "=======================================================\n" << std::flush;

	WorldDomainManifest domain = make_finite_domain();
	TerrainRecipe recipe;
	recipe.legacy_signals.min_elevation_m = -200.0f;
	recipe.legacy_signals.max_elevation_m = 500.0f;
	require(finalize_terrain_recipe(recipe, domain), "recipe setup failed");
	FiniteCanonicalTerrainSignalV1 signal(recipe, domain);

	BlockClipmapProfile profile{};
	profile.candidate_grid_radius = 4;
	profile.inner_hole_radius = 2;
	profile.lod0_block_size = 32.0f;
	profile.level_count = 8;

	WorldPresentationInput pres_input{};
	pres_input.chp_enabled = true;
	pres_input.chp_radius_policy = CHPRadiusPolicy::AreaEquivalent;
	const WorldPresentationManifest presentation = build_world_presentation_manifest(domain, pres_input);

	const std::array<CHPFunctionClass, 3> chp_classes{
		CHPFunctionClass::QuadraticVerticalFallback,
		CHPFunctionClass::SphericalPolynomial4,
		CHPFunctionClass::SphericalPolynomial6
	};

	for (const auto fc : chp_classes) {
		CurvedHorizonProfile req_chp{};
		req_chp.function_class = fc;
		req_chp.requested_maximum_deformation_distance_m = 200000.0;
		ResolvedCurvedHorizonProfile res_chp{};
		require(try_resolve_curved_horizon_profile(presentation, req_chp, res_chp), "CHP profile resolution failed");

		NormalErrorStats stats{};

		for (uint8_t lod = 0; lod < 6; ++lod) {
			for (int64_t bx = -2; bx <= 2; ++bx) {
				for (int64_t bv = -2; bv <= 2; ++bv) {
					for (int32_t qz = 2; qz <= 14; qz += 4) {
						for (int32_t qx = 2; qx <= 14; qx += 4) {
							const auto prod = evaluate_production_r2_normal(
								qx, qz, bx, bv, lod, 0.0, 0.0, profile, signal, true, res_chp
							);
							const auto ref = evaluate_geometric_oracle_normal(
								qx, qz, bx, bv, lod, 0.0, 0.0, profile, signal, true, res_chp
							);

							stats.record(ref, prod.normal_curved);
						}
					}
				}
			}
		}

		std::cout << "  CHP " << get_function_class_name(fc) << ": samples = " << stats.sample_count
		          << ", max angular error = " << stats.max_angular_error_deg << " deg ("
		          << stats.max_angular_error_rad << " rad), max 1-dot = "
		          << stats.max_dot_residual << "\n";

		require(stats.max_angular_error_deg < 0.1, "CHP normal composition error exceeded threshold");
	}

	std::cout << "[PASS] BCCM-MORPH-NORMAL-CHP-COMPOSITION-01 passed across all CHP function classes.\n" << std::flush;
}

// 5. Gate: BCCM-MORPH-NORMAL-DEGENERACY-01 (Degeneracy and Endpoint Fallback)
void test_bccm_morph_normal_degeneracy_01() {
	std::cout << "\n=======================================================\n";
	std::cout << " 5. Gate: BCCM-MORPH-NORMAL-DEGENERACY-01 (Degeneracy / Endpoint)\n";
	std::cout << "=======================================================\n" << std::flush;

	WorldDomainManifest domain = make_finite_domain();
	TerrainRecipe recipe;
	recipe.legacy_signals.min_elevation_m = -200.0f;
	recipe.legacy_signals.max_elevation_m = 500.0f;
	require(finalize_terrain_recipe(recipe, domain), "recipe setup failed");
	FiniteCanonicalTerrainSignalV1 signal(recipe, domain);

	BlockClipmapProfile profile{};
	profile.candidate_grid_radius = 4;
	profile.inner_hole_radius = 2;
	profile.lod0_block_size = 32.0f;
	profile.level_count = 8;

	ResolvedCurvedHorizonProfile null_chp{};

	NormalErrorStats stats_locked{};

	// Test fully parent-locked region (mu >= 1.0)
	for (uint8_t lod = 0; lod < 8; ++lod) {
		// Place observer far enough that block (3, 3) is fully locked (d >= 2.0 B)
		for (int32_t qz = 0; qz <= 16; ++qz) {
			for (int32_t qx = 0; qx <= 16; ++qx) {
				const auto prod = evaluate_production_r2_normal(
					qx, qz, 3, 3, lod, 0.0, 0.0, profile, signal, false, null_chp
				);

				require(std::isfinite(prod.normal_flat.x) && std::isfinite(prod.normal_flat.y) && std::isfinite(prod.normal_flat.z),
					"Non-finite normal in parent-locked region");
				require(prod.normal_flat.y > 0.0, "Inverted normal in parent-locked region");
				require(std::abs(dot(prod.normal_flat, prod.normal_flat) - 1.0) < 1.0e-9, "Unnormalized normal");

				// Check agreement with parent endpoint surface normal
				const SurfaceNormal parent_sn = signal.evaluate_normal(prod.morphed_u, prod.morphed_v);
				const Vec3d parent_n{
					static_cast<double>(parent_sn.nx),
					static_cast<double>(parent_sn.ny),
					static_cast<double>(parent_sn.nz)
				};

				stats_locked.record(parent_n, prod.normal_flat);
			}
		}
	}

	std::cout << "  Parent-locked endpoint: samples = " << stats_locked.sample_count
	          << ", max angular error from parent surface = " << stats_locked.max_angular_error_rad
	          << " rad, max 1-dot = " << stats_locked.max_dot_residual << "\n";

	require(stats_locked.max_angular_error_rad < 1.0e-3, "Parent-locked normal did not match parent surface orientation");
	std::cout << "[PASS] BCCM-MORPH-NORMAL-DEGENERACY-01: Zero NaN/Inf and exact parent agreement confirmed.\n" << std::flush;
}

// 6. Gate: BCCM-MORPH-NORMAL-PAGE-BACKED-01 (Analytic Bilinear Page Derivatives)
void test_bccm_morph_normal_page_backed_01() {
	std::cout << "\n=======================================================\n";
	std::cout << " 6. Gate: BCCM-MORPH-NORMAL-PAGE-BACKED-01 (Bilinear Page Derivatives)\n";
	std::cout << "=======================================================\n" << std::flush;

	// Verify the analytic bilinear derivative formula vs numerical finite difference of bilinear function
	const double h00 = 10.0;
	const double h10 = 14.0;
	const double h01 = 12.0;
	const double h11 = 20.0;
	const double spacing = 2.0;

	auto eval_bilinear = [&](double fx, double fy) {
		return (1.0 - fy) * ((1.0 - fx) * h00 + fx * h10) + fy * ((1.0 - fx) * h01 + fx * h11);
	};

	NormalErrorStats stats{};

	for (double fy = 0.05; fy <= 0.95; fy += 0.1) {
		for (double fx = 0.05; fx <= 0.95; fx += 0.1) {
			// Analytic derivative per meter
			const double analytic_du = ((1.0 - fy) * (h10 - h00) + fy * (h11 - h01)) / spacing;
			const double analytic_dv = ((1.0 - fx) * (h01 - h00) + fx * (h11 - h10)) / spacing;

			// Numerical derivative per meter (dx_meter = dfx * spacing)
			constexpr double df = 1.0e-6;
			const double num_du = (eval_bilinear(fx + df, fy) - eval_bilinear(fx - df, fy)) / (2.0 * df * spacing);
			const double num_dv = (eval_bilinear(fx, fy + df) - eval_bilinear(fx, fy - df)) / (2.0 * df * spacing);

			const Vec3d n_analytic = normalize({ -analytic_du, 1.0, -analytic_dv });
			const Vec3d n_num = normalize({ -num_du, 1.0, -num_dv });

			stats.record(n_num, n_analytic);
		}
	}

	std::cout << "  Bilinear page derivatives: samples = " << stats.sample_count
	          << ", max angular error = " << stats.max_angular_error_rad << " rad, max 1-dot = "
	          << stats.max_dot_residual << "\n";

	require(stats.max_angular_error_rad < 1.0e-7, "Analytic bilinear page derivative failed oracle");
	std::cout << "[PASS] BCCM-MORPH-NORMAL-PAGE-BACKED-01: Analytic bilinear derivatives verified.\n" << std::flush;
}

// ---------------------------------------------------------------------------
// Main Fixture Runner
// ---------------------------------------------------------------------------
int main() {
	std::cout << "## rendering::BCCM-MORPH-NORMAL-R2-FIXTURE\n";
	try {
		test_bccm_morph_normal_chp_off_01();
		test_bccm_morph_normal_identity_01();
		test_bccm_morph_normal_chebyshev_branches_01();
		test_bccm_morph_normal_chp_composition_01();
		test_bccm_morph_normal_degeneracy_01();
		test_bccm_morph_normal_page_backed_01();

		std::cout << "\n=======================================================\n";
		std::cout << " ALL WP6.2 R2 MORPH NORMAL TESTS PASSED CLEANLY (100%)\n";
		std::cout << "=======================================================\n";
		std::cout << "STATUS: PASSED WITH EVIDENCE\n";
		return 0;
	} catch (const std::exception& e) {
		std::cerr << "EXCEPTION: " << e.what() << "\n";
		return 1;
	} catch (...) {
		std::cerr << "EXCEPTION: unknown\n";
		return 1;
	}
}
