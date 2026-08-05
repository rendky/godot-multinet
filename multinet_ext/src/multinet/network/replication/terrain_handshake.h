#ifndef MULTINET_TERRAIN_HANDSHAKE_H
#define MULTINET_TERRAIN_HANDSHAKE_H

#include <cstdint>
#include "multinet/world/terrain/terrain_recipe.h" // For LegacyTerrainSignalBand
#include "multinet/world/terrain/terrain_recipe_identity.h"

namespace Multinet {

struct TerrainRecipeHandshakePacket {
	static constexpr uint32_t EXPECTED_MAGIC = 0x4D4E5452; // 'MNTR'

	uint32_t magic{ EXPECTED_MAGIC };
	TerrainRecipeIdentity identity{};
	LegacyTerrainSignalBand legacy_signals{};

	[[nodiscard]] constexpr bool is_valid(const TerrainRecipeIdentity& expected_identity) const noexcept {
		return magic == EXPECTED_MAGIC && identity == expected_identity;
	}
};

} // namespace Multinet

#endif // MULTINET_TERRAIN_HANDSHAKE_H
