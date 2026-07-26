#ifndef MULTINET_TERRAIN_RECIPE_H
#define MULTINET_TERRAIN_RECIPE_H

#include "modules/multinet/schema/binary_schema.h"

#include <cstdint>

namespace Multinet {

// Gate: TERRAIN-RECIPE-01 (Terrain Recipe Identity & Serialization)

struct TerrainRecipe {
	static constexpr uint32_t EXPECTED_MAGIC = 0x4D4E5452; // 'MNTR'

	uint32_t magic{ EXPECTED_MAGIC };
	uint16_t recipe_version{ 1 };
	uint32_t seed{ 0xDEADBEEF };
	float continental_frequency{ 0.0001f };
	float regional_frequency{ 0.001f };
	float detail_frequency{ 0.01f };
	float max_elevation_m{ 500.0f };
	uint8_t octave_count{ 4 };
	float persistence{ 0.5f };
	float lacunarity{ 2.0f };
};

class TerrainRecipeSerializer {
public:
	static bool write_recipe(BinaryWriter &p_writer, const TerrainRecipe &p_recipe) noexcept {
		if (!p_writer.write_u32_le(TerrainRecipe::EXPECTED_MAGIC)) return false;
		if (!p_writer.write_u16_le(p_recipe.recipe_version)) return false;
		if (!p_writer.write_u32_le(p_recipe.seed)) return false;
		if (!p_writer.write_f32_le(p_recipe.continental_frequency)) return false;
		if (!p_writer.write_f32_le(p_recipe.regional_frequency)) return false;
		if (!p_writer.write_f32_le(p_recipe.detail_frequency)) return false;
		if (!p_writer.write_f32_le(p_recipe.max_elevation_m)) return false;
		if (!p_writer.write_u8(p_recipe.octave_count)) return false;
		if (!p_writer.write_f32_le(p_recipe.persistence)) return false;
		if (!p_writer.write_f32_le(p_recipe.lacunarity)) return false;
		return true;
	}

	static bool read_recipe(BinaryReader &p_reader, TerrainRecipe &r_recipe) noexcept {
		if (!p_reader.read_u32_le(r_recipe.magic) || r_recipe.magic != TerrainRecipe::EXPECTED_MAGIC) return false;
		if (!p_reader.read_u16_le(r_recipe.recipe_version)) return false;
		if (!p_reader.read_u32_le(r_recipe.seed)) return false;
		if (!p_reader.read_f32_le(r_recipe.continental_frequency)) return false;
		if (!p_reader.read_f32_le(r_recipe.regional_frequency)) return false;
		if (!p_reader.read_f32_le(r_recipe.detail_frequency)) return false;
		if (!p_reader.read_f32_le(r_recipe.max_elevation_m)) return false;
		if (!p_reader.read_u8(r_recipe.octave_count)) return false;
		if (!p_reader.read_f32_le(r_recipe.persistence)) return false;
		if (!p_reader.read_f32_le(r_recipe.lacunarity)) return false;
		return true;
	}
};

} // namespace Multinet

#endif // MULTINET_TERRAIN_RECIPE_H
