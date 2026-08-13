#ifndef MULTINET_FINITE_CANONICAL_TERRAIN_SIGNAL_H
#define MULTINET_FINITE_CANONICAL_TERRAIN_SIGNAL_H

#include "multinet/core/spatial/world_manifests.h"
#include "multinet/core/squirrel_noise5.h"
#include "multinet/world/terrain/terrain_recipe.h"
#include "multinet/world/terrain/heightfield_generator.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace Multinet {

// Canonical planar signal for finite worlds. The chart slot remains SurfaceFace
// zero for storage compatibility, but the manifest decides that it is planar.
class FiniteCanonicalTerrainSignalV1 {
private:
	TerrainRecipe recipe{};
	WorldDomainManifest domain{};

	[[nodiscard]] static float smoothstep(float t) noexcept {
		return t * t * (3.0f - 2.0f * t);
	}

	[[nodiscard]] float sample_noise_3d(double x, double y, double z, double frequency, uint32_t salt) const noexcept {
		double px = x * frequency;
		double py = y * frequency;
		double pz = z * frequency;
		double fx = std::floor(px);
		double fy = std::floor(py);
		double fz = std::floor(pz);
		int32_t x0 = static_cast<int32_t>(fx), y0 = static_cast<int32_t>(fy), z0 = static_cast<int32_t>(fz);
		int32_t x1 = x0 + 1, y1 = y0 + 1, z1 = z0 + 1;
		float tx = smoothstep(static_cast<float>(px - fx));
		float ty = smoothstep(static_cast<float>(py - fy));
		float tz = smoothstep(static_cast<float>(pz - fz));
		uint32_t seed = recipe.identity.world_seed ^ salt;
		float n000 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(x0, y0, z0, seed));
		float n100 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(x1, y0, z0, seed));
		float n010 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(x0, y1, z0, seed));
		float n110 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(x1, y1, z0, seed));
		float n001 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(x0, y0, z1, seed));
		float n101 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(x1, y0, z1, seed));
		float n011 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(x0, y1, z1, seed));
		float n111 = squirrel_u01_24_v1(squirrel_noise5_i3_v1(x1, y1, z1, seed));
		float nx00 = n000 + (n100 - n000) * tx;
		float nx10 = n010 + (n110 - n010) * tx;
		float nx01 = n001 + (n101 - n001) * tx;
		float nx11 = n011 + (n111 - n011) * tx;
		float ny0 = nx00 + (nx10 - nx00) * ty;
		float ny1 = nx01 + (nx11 - nx01) * ty;
		return ny0 + (ny1 - ny0) * tz;
	}

	[[nodiscard]] double evaluate_unbounded(double x, double z) const noexcept {
		double amplitude = 1.0;
		double frequency = recipe.legacy_signals.continental_frequency;
		double total = 0.0;
		double max_possible = 0.0;
		for (uint8_t octave = 0; octave < recipe.legacy_signals.octave_count; ++octave) {
			total += static_cast<double>(sample_noise_3d(x, 0.0, z, frequency, static_cast<uint32_t>(octave * 1013))) * amplitude;
			max_possible += amplitude;
			amplitude *= static_cast<double>(recipe.legacy_signals.persistence);
			frequency *= static_cast<double>(recipe.legacy_signals.lacunarity);
		}
		double norm01 = max_possible > 0.0 ? total / max_possible : 0.5;
		double min_e = static_cast<double>(recipe.legacy_signals.min_elevation_m);
		double max_e = static_cast<double>(recipe.legacy_signals.max_elevation_m);
		if (norm01 < 0.5) return min_e * (1.0 - norm01 * 2.0);
		return max_e * ((norm01 - 0.5) * 2.0);
	}

public:
	FiniteCanonicalTerrainSignalV1() = default;
	FiniteCanonicalTerrainSignalV1(const TerrainRecipe& p_recipe, const WorldDomainManifest& p_domain)
		: recipe(p_recipe), domain(p_domain) {
		if (!domain.is_valid() || !domain.is_finite() || !validate_terrain_recipe(recipe, domain)) {
			throw std::invalid_argument("Invalid finite terrain recipe/domain");
		}
	}

	[[nodiscard]] double evaluate_height(double x_m, double z_m) const noexcept {
		if (!domain.is_finite()) return 0.0;
		const double hx = static_cast<double>(domain.finite.half_extent_x_mm) * 0.001;
		const double hz = static_cast<double>(domain.finite.half_extent_z_mm) * 0.001;
		return evaluate_unbounded(std::clamp(x_m, -hx, hx), std::clamp(z_m, -hz, hz));
	}

	[[nodiscard]] SurfaceNormal evaluate_normal(double x_m, double z_m, double step_m = CANONICAL_ANALYTIC_NORMAL_SAMPLE_STEP_M) const noexcept {
		if (!domain.is_finite() || !(step_m > 0.0) || !std::isfinite(step_m)) return {};
		const double hx = static_cast<double>(domain.finite.half_extent_x_mm) * 0.001;
		const double hz = static_cast<double>(domain.finite.half_extent_z_mm) * 0.001;
		const double x = std::clamp(x_m, -hx, hx), z = std::clamp(z_m, -hz, hz);
		const bool at_left = x <= -hx + step_m;
		const bool at_right = x >= hx - step_m;
		const bool at_bottom = z <= -hz + step_m;
		const bool at_top = z >= hz - step_m;
		const double h_center = evaluate_height(x, z);
		const double hL = evaluate_height(x - step_m, z), hR = evaluate_height(x + step_m, z);
		const double hD = evaluate_height(x, z - step_m), hU = evaluate_height(x, z + step_m);
		const double dx = at_left ? (hR - h_center) / step_m
			: (at_right ? (h_center - hL) / step_m : (hR - hL) / (2.0 * step_m));
		const double dz = at_bottom ? (hU - h_center) / step_m
			: (at_top ? (h_center - hD) / step_m : (hU - hD) / (2.0 * step_m));
		const double len = std::sqrt(dx * dx + 1.0 + dz * dz);
		return SurfaceNormal{ static_cast<float>(-dx / len), static_cast<float>(1.0 / len), static_cast<float>(-dz / len) };
	}

	[[nodiscard]] const TerrainRecipe& get_recipe() const noexcept { return recipe; }
	[[nodiscard]] const WorldDomainManifest& get_domain() const noexcept { return domain; }
};

} // namespace Multinet

#endif // MULTINET_FINITE_CANONICAL_TERRAIN_SIGNAL_H
