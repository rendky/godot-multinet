#ifndef MULTINET_HEIGHTFIELD_GENERATOR_H
#define MULTINET_HEIGHTFIELD_GENERATOR_H

#include "modules/multinet/core/coordinates.h"
#include "modules/multinet/core/squirrel_noise5.h"
#include "modules/multinet/terrain/terrain_recipe.h"

#include <cmath>
#include <cstdint>

namespace Multinet {

// ============================================================================
// Gate: TERRAIN-FIELD-01 (Deterministic Heightfield Generator via SquirrelNoise5)
// Gate: TERRAIN-SEAM-01 (Region Seam Continuity)
// ============================================================================

struct SurfaceNormal {
	float nx{ 0.0f };
	float ny{ 1.0f };
	float nz{ 0.0f };
};

class HeightfieldGenerator {
private:
	TerrainRecipe recipe{};

	[[nodiscard]] static float smoothstep(float p_t) noexcept {
		return p_t * p_t * (3.0f - 2.0f * p_t);
	}

	[[nodiscard]] float sample_noise_2d(double p_x, double p_z, uint32_t p_salt) const noexcept {
		double floor_x = std::floor(p_x);
		double floor_z = std::floor(p_z);

		int32_t x0 = static_cast<int32_t>(floor_x);
		int32_t z0 = static_cast<int32_t>(floor_z);
		int32_t x1 = x0 + 1;
		int32_t z1 = z0 + 1;

		float tx = smoothstep(static_cast<float>(p_x - floor_x));
		float tz = smoothstep(static_cast<float>(p_z - floor_z));

		uint32_t seed = recipe.seed ^ p_salt;
		float n00 = squirrel_noise5_2d_zero_to_one(x0, z0, seed);
		float n10 = squirrel_noise5_2d_zero_to_one(x1, z0, seed);
		float n01 = squirrel_noise5_2d_zero_to_one(x0, z1, seed);
		float n11 = squirrel_noise5_2d_zero_to_one(x1, z1, seed);

		float nx0 = n00 + (n10 - n00) * tx;
		float nx1 = n01 + (n11 - n01) * tx;
		return nx0 + (nx1 - nx0) * tz;
	}

public:
	HeightfieldGenerator() = default;

	explicit HeightfieldGenerator(const TerrainRecipe &p_recipe) : recipe(p_recipe) {}

	[[nodiscard]] double evaluate_height(double p_world_x, double p_world_z) const noexcept {
		double amplitude = 1.0;
		double frequency = recipe.continental_frequency;
		double total_elevation = 0.0;
		double max_possible = 0.0;

		for (uint8_t octave = 0; octave < recipe.octave_count; ++octave) {
			double sample_x = p_world_x * frequency;
			double sample_z = p_world_z * frequency;

			float n = sample_noise_2d(sample_x, sample_z, static_cast<uint32_t>(octave * 1013));
			total_elevation += static_cast<double>(n) * amplitude;
			max_possible += amplitude;

			amplitude *= static_cast<double>(recipe.persistence);
			frequency *= static_cast<double>(recipe.lacunarity);
		}

		double normalized = total_elevation / max_possible;
		return normalized * static_cast<double>(recipe.max_elevation_m);
	}

	[[nodiscard]] double evaluate_height(const WorldPosition64 &p_pos) const noexcept {
		return evaluate_height(p_pos.x, p_pos.z);
	}

	[[nodiscard]] double evaluate_height(const RegionPosition &p_region_pos) const noexcept {
		WorldPosition64 w = p_region_pos.to_world();
		return evaluate_height(w.x, w.z);
	}

	[[nodiscard]] SurfaceNormal evaluate_normal(double p_world_x, double p_world_z, double p_sample_step_m = 0.5) const noexcept {
		double hL = evaluate_height(p_world_x - p_sample_step_m, p_world_z);
		double hR = evaluate_height(p_world_x + p_sample_step_m, p_world_z);
		double hD = evaluate_height(p_world_x, p_world_z - p_sample_step_m);
		double hU = evaluate_height(p_world_x, p_world_z + p_sample_step_m);

		double dx = (hR - hL) / (2.0 * p_sample_step_m);
		double dz = (hU - hD) / (2.0 * p_sample_step_m);

		double len = std::sqrt(dx * dx + 1.0 + dz * dz);
		return SurfaceNormal{
			static_cast<float>(-dx / len),
			static_cast<float>(1.0 / len),
			static_cast<float>(-dz / len)
		};
	}

	[[nodiscard]] const TerrainRecipe &get_recipe() const noexcept { return recipe; }
};

} // namespace Multinet

#endif // MULTINET_HEIGHTFIELD_GENERATOR_H
