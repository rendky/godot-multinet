#ifndef MULTINET_TERRAIN_RECIPE_IDENTITY_H
#define MULTINET_TERRAIN_RECIPE_IDENTITY_H

#include "multinet/core/schema/binary_schema.h"
#include <cstdint>

namespace Multinet {

struct TerrainRecipeIdentity {
	uint16_t recipe_version{ 1 };
	uint16_t schema_version{ 1 };
	uint32_t topology_version{ 1 };
	uint32_t projection_version{ 1 };
	uint32_t deterministic_algorithm_id{ 0x53513502u }; // SquirrelNoise5U3V1
	uint32_t world_seed{ 1337 };
	uint64_t recipe_hash{ 0 };
	uint64_t manifest_hash{ 0 };

	[[nodiscard]] bool operator==(const TerrainRecipeIdentity& other) const noexcept {
		return recipe_version == other.recipe_version &&
		       schema_version == other.schema_version &&
		       topology_version == other.topology_version &&
		       projection_version == other.projection_version &&
		       deterministic_algorithm_id == other.deterministic_algorithm_id &&
		       world_seed == other.world_seed &&
		       recipe_hash == other.recipe_hash &&
		       manifest_hash == other.manifest_hash;
	}

	[[nodiscard]] bool operator!=(const TerrainRecipeIdentity& other) const noexcept {
		return !(*this == other);
	}
};

class TerrainRecipeIdentitySerializer {
public:
	static bool write_identity(BinaryWriter &p_writer, const TerrainRecipeIdentity &p_identity) noexcept {
		if (!p_writer.write_u16_le(p_identity.recipe_version)) return false;
		if (!p_writer.write_u16_le(p_identity.schema_version)) return false;
		if (!p_writer.write_u32_le(p_identity.topology_version)) return false;
		if (!p_writer.write_u32_le(p_identity.projection_version)) return false;
		if (!p_writer.write_u32_le(p_identity.deterministic_algorithm_id)) return false;
		if (!p_writer.write_u32_le(p_identity.world_seed)) return false;
		if (!p_writer.write_u64_le(p_identity.recipe_hash)) return false;
		if (!p_writer.write_u64_le(p_identity.manifest_hash)) return false;
		return true;
	}

	static bool read_identity(BinaryReader &p_reader, TerrainRecipeIdentity &r_identity) noexcept {
		if (!p_reader.read_u16_le(r_identity.recipe_version)) return false;
		if (!p_reader.read_u16_le(r_identity.schema_version)) return false;
		if (!p_reader.read_u32_le(r_identity.topology_version)) return false;
		if (!p_reader.read_u32_le(r_identity.projection_version)) return false;
		if (!p_reader.read_u32_le(r_identity.deterministic_algorithm_id)) return false;
		if (!p_reader.read_u32_le(r_identity.world_seed)) return false;
		if (!p_reader.read_u64_le(r_identity.recipe_hash)) return false;
		if (!p_reader.read_u64_le(r_identity.manifest_hash)) return false;
		return true;
	}
};

} // namespace Multinet

#endif // MULTINET_TERRAIN_RECIPE_IDENTITY_H
