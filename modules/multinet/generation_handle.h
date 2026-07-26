#ifndef MULTINET_GENERATION_HANDLE_H
#define MULTINET_GENERATION_HANDLE_H

#include <cstdint>

namespace Multinet {

struct GenerationHandle {
	uint32_t index{ 0xFFFFFFFF };
	uint32_t generation{ 0 };

	[[nodiscard]] constexpr bool is_valid() const noexcept {
		return index != 0xFFFFFFFF;
	}

	[[nodiscard]] constexpr bool operator==(const GenerationHandle &p_other) const noexcept {
		return index == p_other.index && generation == p_other.generation;
	}

	[[nodiscard]] constexpr bool operator!=(const GenerationHandle &p_other) const noexcept {
		return !(*this == p_other);
	}

	static constexpr GenerationHandle invalid() noexcept {
		return GenerationHandle{ 0xFFFFFFFF, 0 };
	}
};

} // namespace Multinet

#endif // MULTINET_GENERATION_HANDLE_H
