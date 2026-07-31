#ifndef MULTINET_HPGS_TYPES_H
#define MULTINET_HPGS_TYPES_H

#include "../../core/schema/feature_key.h"
#include <cstdint>
#include <type_traits>

namespace Multinet {

enum class HPGSChannelType : uint8_t {
	Synthetic = 0,
	Terrain = 1,
	Structure = 2,
	Collision = 3,
	Water = 4
};

struct HPGSRequest {
	FeatureKey key{};
	uint64_t epoch{ 0 };
	HPGSChannelType channel{ HPGSChannelType::Synthetic };
	uint8_t lod{ 0 };
	uint16_t request_flags{ 0 };
};

struct HPGSResult {
	FeatureKey key{};
	uint64_t epoch{ 0 };
	uint16_t generator_version{ 0 };
	bool is_stale{ false };
	bool is_cancelled{ false };
};

static_assert(std::is_trivially_copyable_v<HPGSRequest>, "HPGSRequest must be POD");
static_assert(std::is_trivially_copyable_v<HPGSResult>, "HPGSResult must be POD");

} // namespace Multinet

#endif // MULTINET_HPGS_TYPES_H
