#ifndef MULTINET_CANONICAL_TERRAIN_SIGNAL_H
#define MULTINET_CANONICAL_TERRAIN_SIGNAL_H

#include "multinet/core/spatial/surface_address.h"
#include "multinet/core/spatial/surface_projection.h"
#include "multinet/core/spatial/world_manifests.h"
#include "multinet/core/squirrel_noise5.h"
#include "multinet/world/terrain/terrain_recipe.h"

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace Multinet {

class CanonicalTerrainSignalV1 {
private:
	TerrainRecipe recipe{};
	WorldScaleManifest scale{};

	[[nodiscard]] static float smoothstep(float p_t) noexcept {
		return p_t * p_t * (3.0f - 2.0f * p_t);
	}

	[[nodiscard]] float sample_noise_3d(const Vec3d& dir, double frequency, uint32_t p_salt) const noexcept {
		double px = dir.x * frequency;
		double py = dir.y * frequency;
		double pz = dir.z * frequency;

		double floor_x = std::floor(px);
		double floor_y = std::floor(py);
		double floor_z = std::floor(pz);

		int32_t x0 = static_cast<int32_t>(floor_x);
		int32_t y0 = static_cast<int32_t>(floor_y);
		int32_t z0 = static_cast<int32_t>(floor_z);
		int32_t x1 = x0 + 1;
		int32_t y1 = y0 + 1;
		int32_t z1 = z0 + 1;

		float tx = smoothstep(static_cast<float>(px - floor_x));
		float ty = smoothstep(static_cast<float>(py - floor_y));
		float tz = smoothstep(static_cast<float>(pz - floor_z));

		uint32_t seed = recipe.identity.world_seed ^ p_salt;

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

public:
	CanonicalTerrainSignalV1() = default;

	CanonicalTerrainSignalV1(const TerrainRecipe &p_recipe, const WorldScaleManifest &p_scale)
		: recipe(p_recipe), scale(p_scale) {
		if (!validate_terrain_recipe(recipe, scale)) {
			throw std::invalid_argument("Invalid or mismatched terrain recipe");
		}
	}

	[[nodiscard]] double evaluate_height(const SurfacePosition64 &p_pos) const noexcept {
		SurfaceAddress addr;
		addr.face = p_pos.face;
		addr.u_mm = static_cast<int64_t>(std::round(p_pos.u_m * 1000.0));
		addr.v_mm = static_cast<int64_t>(std::round(p_pos.v_m * 1000.0));

		SurfaceAddress canon = canonicalize_surface_address(addr, scale);
		if (!canon.is_valid()) {
			return 0.0;
		}

		double u_normalized = static_cast<double>(canon.u_mm) / static_cast<double>(scale.chart_half_extent_mm);
		double v_normalized = static_cast<double>(canon.v_mm) / static_cast<double>(scale.chart_half_extent_mm);

		FramePosition64 direction = ProjectionCOBE::map_forward(static_cast<int>(canon.face), u_normalized, v_normalized);

		Vec3d physical_pos;
		physical_pos.x = direction.x * scale.logical_area_radius_m;
		physical_pos.y = direction.y * scale.logical_area_radius_m;
		physical_pos.z = direction.z * scale.logical_area_radius_m;

		double amplitude = 1.0;
		double frequency = recipe.legacy_signals.continental_frequency;
		double total_elevation = 0.0;
		double max_possible = 0.0;

		for (uint8_t octave = 0; octave < recipe.legacy_signals.octave_count; ++octave) {
			float n = sample_noise_3d(physical_pos, frequency, static_cast<uint32_t>(octave * 1013));
			total_elevation += static_cast<double>(n) * amplitude;
			max_possible += amplitude;

			amplitude *= static_cast<double>(recipe.legacy_signals.persistence);
			frequency *= static_cast<double>(recipe.legacy_signals.lacunarity);
		}

		double norm01 = total_elevation / max_possible;
		double min_e = static_cast<double>(recipe.legacy_signals.min_elevation_m);
		double max_e = static_cast<double>(recipe.legacy_signals.max_elevation_m);

		if (norm01 < 0.5) {
			double t = norm01 * 2.0;
			return min_e * (1.0 - t);
		} else {
			double t = (norm01 - 0.5) * 2.0;
			return max_e * t;
		}
	}

	[[nodiscard]] const TerrainRecipe &get_recipe() const noexcept { return recipe; }
};

} // namespace Multinet

#endif // MULTINET_CANONICAL_TERRAIN_SIGNAL_H
