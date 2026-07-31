#ifndef MULTINET_TERRAIN_HANDSHAKE_H
#define MULTINET_TERRAIN_HANDSHAKE_H

#include <cstdint>

namespace Multinet {

struct TerrainRecipeHandshakePacket {
	static constexpr uint32_t EXPECTED_MAGIC = 0x4D4E5452; // 'MNTR'

	uint32_t magic{ EXPECTED_MAGIC };
	uint32_t recipe_version{ 1 };
	uint32_t world_seed{ 1337 };
	float min_elevation_m{ -100.0f };
	float max_elevation_m{ 1000.0f };
	float continental_frequency{ 0.001f };

	[[nodiscard]] constexpr bool is_valid() const noexcept {
		return magic == EXPECTED_MAGIC && recipe_version > 0;
	}
};

} // namespace Multinet

#endif // MULTINET_TERRAIN_HANDSHAKE_H
