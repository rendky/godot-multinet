#ifndef MULTINET_TERRAIN_RECIPE_H
#define MULTINET_TERRAIN_RECIPE_H

#include "multinet/core/schema/binary_schema.h"

#include <cstdint>
#include <bit>

#include "multinet/world/terrain/terrain_recipe_identity.h"
#include "multinet/core/spatial/world_manifests.h"

namespace Multinet {

// Gate: TERRAIN-RECIPE-01 (Terrain Recipe Identity & Serialization)

struct LegacyTerrainSignalBand {
	float continental_frequency{ 0.0001f };
	float regional_frequency{ 0.001f };
	float detail_frequency{ 0.01f };
	float min_elevation_m{ -200.0f };
	float max_elevation_m{ 500.0f };
	uint8_t octave_count{ 4 };
	float persistence{ 0.5f };
	float lacunarity{ 2.0f };
};

struct TerrainRecipe {
	static constexpr uint32_t EXPECTED_MAGIC = 0x4D4E5452; // 'MNTR'

	uint32_t magic{ EXPECTED_MAGIC };
	TerrainRecipeIdentity identity{};
	LegacyTerrainSignalBand legacy_signals{};
};

class TerrainRecipeSerializer {
public:
	static bool write_recipe(BinaryWriter &p_writer, const TerrainRecipe &p_recipe) noexcept {
		if (!p_writer.write_u32_le(TerrainRecipe::EXPECTED_MAGIC)) return false;
		if (!TerrainRecipeIdentitySerializer::write_identity(p_writer, p_recipe.identity)) return false;
		if (!p_writer.write_f32_le(p_recipe.legacy_signals.continental_frequency)) return false;
		if (!p_writer.write_f32_le(p_recipe.legacy_signals.regional_frequency)) return false;
		if (!p_writer.write_f32_le(p_recipe.legacy_signals.detail_frequency)) return false;
		if (!p_writer.write_f32_le(p_recipe.legacy_signals.min_elevation_m)) return false;
		if (!p_writer.write_f32_le(p_recipe.legacy_signals.max_elevation_m)) return false;
		if (!p_writer.write_u8(p_recipe.legacy_signals.octave_count)) return false;
		if (!p_writer.write_f32_le(p_recipe.legacy_signals.persistence)) return false;
		if (!p_writer.write_f32_le(p_recipe.legacy_signals.lacunarity)) return false;
		return true;
	}

	static bool read_recipe(BinaryReader &p_reader, TerrainRecipe &r_recipe) noexcept {
		if (!p_reader.read_u32_le(r_recipe.magic) || r_recipe.magic != TerrainRecipe::EXPECTED_MAGIC) return false;
		if (!TerrainRecipeIdentitySerializer::read_identity(p_reader, r_recipe.identity)) return false;
		if (!p_reader.read_f32_le(r_recipe.legacy_signals.continental_frequency)) return false;
		if (!p_reader.read_f32_le(r_recipe.legacy_signals.regional_frequency)) return false;
		if (!p_reader.read_f32_le(r_recipe.legacy_signals.detail_frequency)) return false;
		if (!p_reader.read_f32_le(r_recipe.legacy_signals.min_elevation_m)) return false;
		if (!p_reader.read_f32_le(r_recipe.legacy_signals.max_elevation_m)) return false;
		if (!p_reader.read_u8(r_recipe.legacy_signals.octave_count)) return false;
		if (!p_reader.read_f32_le(r_recipe.legacy_signals.persistence)) return false;
		if (!p_reader.read_f32_le(r_recipe.legacy_signals.lacunarity)) return false;
		return true;
	}
};

namespace {
	inline void recipe_hash_combine_u64_le(uint64_t &hash, uint64_t val) noexcept {
		for (int i = 0; i < 8; ++i) {
			hash ^= (val & 0xFF);
			hash *= 1099511628211ULL;
			val >>= 8;
		}
	}
	inline void recipe_hash_combine_u32_le(uint64_t &hash, uint32_t val) noexcept {
		for (int i = 0; i < 4; ++i) {
			hash ^= (val & 0xFF);
			hash *= 1099511628211ULL;
			val >>= 8;
		}
	}
	inline void recipe_hash_combine_u16_le(uint64_t &hash, uint16_t val) noexcept {
		for (int i = 0; i < 2; ++i) {
			hash ^= (val & 0xFF);
			hash *= 1099511628211ULL;
			val >>= 8;
		}
	}
	inline void recipe_hash_combine_u8(uint64_t &hash, uint8_t val) noexcept {
		hash ^= val;
		hash *= 1099511628211ULL;
	}
	inline void recipe_hash_combine_f32_le(uint64_t &hash, float val) noexcept {
		uint32_t bit_pattern = std::bit_cast<uint32_t>(val);
		recipe_hash_combine_u32_le(hash, bit_pattern);
	}
}

constexpr uint32_t TERRAIN_RECIPE_HASH_VERSION = 1;

[[nodiscard]] inline uint64_t compute_terrain_recipe_hash(const TerrainRecipe& recipe) noexcept {
	uint64_t h = 14695981039346656037ULL; // FNV-1a 64-bit offset basis
	
	recipe_hash_combine_u32_le(h, TERRAIN_RECIPE_HASH_VERSION);
	recipe_hash_combine_u32_le(h, recipe.identity.recipe_version);
	recipe_hash_combine_u16_le(h, recipe.identity.schema_version); // Wait, we need u16 le...
	recipe_hash_combine_u32_le(h, recipe.identity.topology_version);
	recipe_hash_combine_u32_le(h, recipe.identity.projection_version);
	recipe_hash_combine_u32_le(h, recipe.identity.deterministic_algorithm_id);
	recipe_hash_combine_u32_le(h, recipe.identity.world_seed);
	recipe_hash_combine_u64_le(h, recipe.identity.manifest_hash);

	recipe_hash_combine_f32_le(h, recipe.legacy_signals.continental_frequency);
	recipe_hash_combine_f32_le(h, recipe.legacy_signals.regional_frequency);
	recipe_hash_combine_f32_le(h, recipe.legacy_signals.detail_frequency);
	recipe_hash_combine_f32_le(h, recipe.legacy_signals.min_elevation_m);
	recipe_hash_combine_f32_le(h, recipe.legacy_signals.max_elevation_m);
	recipe_hash_combine_u8(h, recipe.legacy_signals.octave_count);
	recipe_hash_combine_f32_le(h, recipe.legacy_signals.persistence);
	recipe_hash_combine_f32_le(h, recipe.legacy_signals.lacunarity);

	if (h == 0) h = 1;
	return h;
}

[[nodiscard]] inline bool finalize_terrain_recipe(TerrainRecipe& recipe, const WorldScaleManifest& scale) noexcept {
	if (!scale.is_valid()) return false;
	
	recipe.identity.topology_version = scale.topology_version;
	recipe.identity.projection_version = scale.projection_version;
	recipe.identity.manifest_hash = scale.manifest_hash;
	recipe.identity.deterministic_algorithm_id = 0x53513502u; // SquirrelNoise5U3V1
	
	recipe.identity.recipe_hash = compute_terrain_recipe_hash(recipe);
	return recipe.identity.recipe_hash != 0;
}

[[nodiscard]] inline bool validate_terrain_recipe(const TerrainRecipe& recipe, const WorldScaleManifest& scale) noexcept {
	if (!scale.is_valid()) return false;
	if (recipe.identity.topology_version != scale.topology_version) return false;
	if (recipe.identity.projection_version != scale.projection_version) return false;
	if (recipe.identity.manifest_hash != scale.manifest_hash) return false;
	if (recipe.identity.deterministic_algorithm_id != 0x53513502u) return false;
	if (recipe.identity.recipe_hash == 0) return false;
	
	uint64_t expected_hash = compute_terrain_recipe_hash(recipe);
	return recipe.identity.recipe_hash == expected_hash;
}

} // namespace Multinet

#endif // MULTINET_TERRAIN_RECIPE_H
