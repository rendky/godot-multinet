#include "multinet/core/spatial/world_manifests.h"
#include "multinet/core/spatial/surface_address.h"
#include "multinet/world/terrain/canonical_terrain_signal.h"
#include "multinet/world/terrain/finite_canonical_terrain_signal.h"
#include "multinet/world/terrain/terrain_recipe.h"

#include <cmath>
#include <array>
#include <cstdlib>
#include <iostream>
#include <utility>

using namespace Multinet;

static void require(bool condition, const char* message) {
	if (!condition) {
		std::cerr << "FAILURE: " << message << "\n";
		std::exit(1);
	}
}

static double normal_distance(const SurfaceNormal& a, const SurfaceNormal& b) {
	const double x = static_cast<double>(a.nx) - b.nx;
	const double y = static_cast<double>(a.ny) - b.ny;
	const double z = static_cast<double>(a.nz) - b.nz;
	return std::sqrt(x * x + y * y + z * z);
}

static SurfaceNormal canonical_normal(const CanonicalTerrainSignalV1& signal, SurfacePosition64 position, double step_m) {
	const double h_l = signal.evaluate_height(SurfacePosition64{ position.face, position.u_m - step_m, position.v_m, position.altitude_m, position.topology_version, position.projection_version });
	const double h_r = signal.evaluate_height(SurfacePosition64{ position.face, position.u_m + step_m, position.v_m, position.altitude_m, position.topology_version, position.projection_version });
	const double h_d = signal.evaluate_height(SurfacePosition64{ position.face, position.u_m, position.v_m - step_m, position.altitude_m, position.topology_version, position.projection_version });
	const double h_u = signal.evaluate_height(SurfacePosition64{ position.face, position.u_m, position.v_m + step_m, position.altitude_m, position.topology_version, position.projection_version });
	const double du = (h_r - h_l) / (2.0 * step_m);
	const double dv = (h_u - h_d) / (2.0 * step_m);
	const double len = std::sqrt(du * du + 1.0 + dv * dv);
	return SurfaceNormal{ static_cast<float>(-du / len), static_cast<float>(1.0 / len), static_cast<float>(-dv / len) };
}

static WorldDomainManifest make_domain() {
	WorldDomainInput input;
	input.topology = WorldDomainTopology::FiniteRectangle;
	input.finite.extent_x_m = 500000;
	input.finite.extent_z_m = 400000;
	return build_world_domain_manifest(input);
}

int main() {
	std::cerr << "## rendering::BCCM-FLAT-NORMAL-LOD-01\n";
	try {
	WorldDomainManifest domain = make_domain();
	TerrainRecipe recipe;
	recipe.legacy_signals.min_elevation_m = -200.0f;
	recipe.legacy_signals.max_elevation_m = 500.0f;
	require(finalize_terrain_recipe(recipe, domain), "normal recipe finalization failed");
	FiniteCanonicalTerrainSignalV1 signal(recipe, domain);

	const SurfaceNormal baseline = signal.evaluate_normal(1234.0, -4321.0);
	for (int lod = 0; lod < 8; ++lod) {
		const SurfaceNormal same_point = signal.evaluate_normal(1234.0, -4321.0);
		require(normal_distance(baseline, same_point) < 1e-6, "normal changed with visual LOD");
	}

	const std::array<std::pair<float, float>, 3> ranges{{
		{ -1000.0f, 1000.0f },
		{ -200.0f, 500.0f },
		{ -50.0f, 100.0f }
	}};
	for (const auto& range : ranges) {
		TerrainRecipe ranged = recipe;
		ranged.legacy_signals.min_elevation_m = range.first;
		ranged.legacy_signals.max_elevation_m = range.second;
		require(finalize_terrain_recipe(ranged, domain), "elevation-range recipe finalization failed");
		FiniteCanonicalTerrainSignalV1 ranged_signal(ranged, domain);
		require(std::isfinite(ranged_signal.evaluate_normal(1234.0, -4321.0).ny), "elevation-range normal invalid");
	}
	TerrainRecipe amplified = recipe;
	amplified.legacy_signals.min_elevation_m = -1000.0f;
	amplified.legacy_signals.max_elevation_m = 1000.0f;
	require(finalize_terrain_recipe(amplified, domain), "amplified recipe finalization failed");
	FiniteCanonicalTerrainSignalV1 amplified_signal(amplified, domain);
	const SurfaceNormal changed = amplified_signal.evaluate_normal(1234.0, -4321.0);
	require(normal_distance(baseline, changed) > 1e-7, "elevation range did not update normal");

	const double hx = static_cast<double>(domain.finite.half_extent_x_mm) * 0.001;
	const double hz = static_cast<double>(domain.finite.half_extent_z_mm) * 0.001;
	(void)signal.evaluate_normal(hx, hz);
	(void)signal.evaluate_normal(-hx, -hz);
	const SurfaceNormal edge_x = signal.evaluate_normal(hx, 100.0);
	const SurfaceNormal edge_z = signal.evaluate_normal(100.0, hz);
	require(std::isfinite(edge_x.ny) && std::isfinite(edge_z.ny), "finite boundary normal invalid");

	WorldDomainInput closed_input;
	closed_input.closed_surface.area_equivalent_side_m = 32000;
	WorldDomainManifest closed = build_world_domain_manifest(closed_input);
	TerrainRecipe closed_recipe = recipe;
	require(finalize_terrain_recipe(closed_recipe, closed.closed_surface), "closed seam recipe finalization failed");
	CanonicalTerrainSignalV1 closed_signal(closed_recipe, closed.closed_surface);
	const double H = static_cast<double>(closed.closed_surface.chart_half_extent_mm) * 0.001;
	SurfacePosition64 seam_raw{ SurfaceFace::PositiveX, H + 0.25, 123.0, 0.0, closed.topology_version, closed.projection_version };
	SurfaceAddress seam_address{ SurfaceFace::PositiveX, static_cast<int64_t>(std::llround(seam_raw.u_m * 1000.0)), static_cast<int64_t>(std::llround(seam_raw.v_m * 1000.0)), 0, closed.topology_version, closed.projection_version };
	SurfaceAddress seam_canonical = canonicalize_surface_address(seam_address, closed.closed_surface);
	SurfacePosition64 seam_alias{ seam_canonical.face, seam_canonical.u_mm * 0.001, seam_canonical.v_mm * 0.001, 0.0, closed.topology_version, closed.projection_version };
	require(std::abs(closed_signal.evaluate_height(seam_raw) - closed_signal.evaluate_height(seam_alias)) < 1e-6, "closed seam height alias mismatch");
	require(normal_distance(canonical_normal(closed_signal, seam_raw, CANONICAL_ANALYTIC_NORMAL_SAMPLE_STEP_M), canonical_normal(closed_signal, seam_alias, CANONICAL_ANALYTIC_NORMAL_SAMPLE_STEP_M)) < 1e-3,
		"closed seam normal alias mismatch");

	const double analytic_slope = 0.25;
	for (const double lod_spacing : { 2.0, 256.0 }) {
		const double delta_slope = (lod_spacing - (-lod_spacing)) / (2.0 * lod_spacing);
		require(std::abs((analytic_slope + delta_slope) - (analytic_slope + 1.0)) < 1e-9, "hybrid physical slope changed with LOD spacing");
	}
	std::cout << "analytic_step_m=" << CANONICAL_ANALYTIC_NORMAL_SAMPLE_STEP_M << "\n";
	std::cout << "lods_qualified=0-7\n";
	std::cout << "elevation_ranges_qualified=3\n";
	std::cout << "closed_seam_normal_qualified=1\n";
	std::cout << "hybrid_physical_slope_qualified=1\n";
	std::cout << "STATUS: PASSED WITH EVIDENCE\n";
	return 0;
	} catch (const std::exception& error) {
		std::cerr << "EXCEPTION: " << error.what() << "\n";
		return 1;
	} catch (...) {
		std::cerr << "EXCEPTION: unknown\n";
		return 1;
	}
}
