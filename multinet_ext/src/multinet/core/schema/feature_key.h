#ifndef MULTINET_FEATURE_KEY_H
#define MULTINET_FEATURE_KEY_H

#include "../squirrel_noise5.h"
#include <cstdint>
#include <type_traits>

namespace Multinet {

// ============================================================================
// FeatureKey - 128-bit Hierarchical Identity (Blocker B / Task R1)
// Enables deterministic parent/child feature key derivation independent of
// traversal order, global state, or sibling generation.
// ============================================================================

struct FeatureKey {
	uint64_t root_id{ 0 };
	uint64_t path_hash{ 0 };

	[[nodiscard]] static constexpr FeatureKey make_root(uint64_t p_root_seed) noexcept {
		return FeatureKey{ p_root_seed, 0 };
	}

	[[nodiscard]] constexpr FeatureKey derive_child(
			uint16_t p_feature_class,
			uint32_t p_local_child_id,
			uint16_t p_generator_version) const noexcept {
		const uint32_t n0 = static_cast<uint32_t>(path_hash);
		const uint32_t n1 = static_cast<uint32_t>(path_hash >> 32);

		const uint32_t h0 = squirrel_noise5(static_cast<int32_t>(p_feature_class), n0);
		const uint32_t h1 = squirrel_noise5(static_cast<int32_t>(p_local_child_id), h0);
		const uint32_t h2 = squirrel_noise5(static_cast<int32_t>(p_generator_version), h1 ^ n1);

		const uint64_t next_hash = (static_cast<uint64_t>(h2) << 32) | h1;
		return FeatureKey{ root_id, next_hash };
	}

	constexpr bool operator==(const FeatureKey &p_other) const noexcept = default;
};

// Static assertions to guarantee C++23 POD layout & zero-allocation compliance
static_assert(sizeof(FeatureKey) == 16, "FeatureKey must be exactly 128 bits (16 bytes)");
static_assert(std::is_trivially_copyable_v<FeatureKey>, "FeatureKey must be trivially copyable POD");

} // namespace Multinet

#endif // MULTINET_FEATURE_KEY_H
