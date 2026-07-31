#ifndef MULTINET_REGION_SUBSCRIPTION_H
#define MULTINET_REGION_SUBSCRIPTION_H

#include "multinet/core/coordinates.h"

#include <cstddef>
#include <cstdint>

namespace Multinet {

struct RegionID {
	int64_t x{ 0 };
	int64_t y{ 0 };
	int64_t z{ 0 };

	[[nodiscard]] constexpr bool operator==(const RegionID &p_other) const noexcept {
		return x == p_other.x && y == p_other.y && z == p_other.z;
	}
};

struct PrefetchHint {
	Vec3f velocity_ms{};
	float speed_ms{ 0.0f };
	uint8_t prefetch_radius_cells{ 1 };
};

template <size_t MaxSubscribedRegions = 16>
struct RegionSubscriptionPacket {
	uint32_t player_id{ 0 };
	uint32_t sequence_num{ 0 };
	RegionID center_region{};
	PrefetchHint prefetch_hint{};

	uint8_t subscribed_count{ 0 };
	RegionID subscribed_regions[MaxSubscribedRegions]{};

	bool add_region(const RegionID &p_region) noexcept {
		if (subscribed_count >= MaxSubscribedRegions) {
			return false;
		}
		subscribed_regions[subscribed_count++] = p_region;
		return true;
	}
};

} // namespace Multinet

#endif // MULTINET_REGION_SUBSCRIPTION_H
