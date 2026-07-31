#ifndef MULTINET_TERRAIN_EVENTS_H
#define MULTINET_TERRAIN_EVENTS_H

#include "multinet/network/events/typed_events.h"
#include "multinet/network/replication/region_subscription.h"

namespace Multinet {

enum class TerrainMutationType : uint8_t {
	CRATER = 0,
	FLATTEN = 1,
	RAISE = 2,
	LOWER = 3
};

struct TerrainMutationEvent {
	RegionID target_region{};
	float local_x{ 0.0f };
	float local_z{ 0.0f };
	float radius_m{ 5.0f };
	float depth_or_height_m{ 2.0f };
	TerrainMutationType mutation_type{ TerrainMutationType::CRATER };
	uint32_t mutation_version{ 1 };
};

} // namespace Multinet

#endif // MULTINET_TERRAIN_EVENTS_H
