#ifndef MULTINET_BOUNDED_POOL_H
#define MULTINET_BOUNDED_POOL_H

#include "multinet/core/memory/generation_handle.h"
#include <cstddef>
#include <cstdint>
#include <vector>
#include <utility>

namespace Multinet {

template <typename T, size_t Capacity>
class BoundedPool {
private:
	struct Slot {
		T storage;
		uint32_t generation{ 1 };
		bool is_active{ false };
	};

	Slot slots[Capacity]{};
	uint32_t free_list[Capacity]{};
	size_t free_head{ 0 };
	size_t active_count{ 0 };
	bool overflow_flag{ false };

public:
	BoundedPool() {
		for (size_t i = 0; i < Capacity; ++i) {
			free_list[i] = static_cast<uint32_t>(i);
		}
	}

	template <typename... Args>
	[[nodiscard]] GenerationHandle allocate(Args &&... p_args) {
		if (free_head >= Capacity) {
			overflow_flag = true;
			return GenerationHandle::invalid(); // Bounded pool exhaustion
		}

		uint32_t slot_idx = free_list[free_head++];
		Slot &slot = slots[slot_idx];
		slot.storage = T(std::forward<Args>(p_args)...);
		slot.is_active = true;
		active_count++;

		return GenerationHandle{ slot_idx, slot.generation };
	}

	[[nodiscard]] T *get(GenerationHandle p_handle) noexcept {
		if (!p_handle.is_valid() || p_handle.index >= Capacity) {
			return nullptr;
		}

		Slot &slot = slots[p_handle.index];
		if (!slot.is_active || slot.generation != p_handle.generation) {
			return nullptr; // Stale or freed generation handle rejected
		}

		return &slot.storage;
	}

	[[nodiscard]] const T *get(GenerationHandle p_handle) const noexcept {
		if (!p_handle.is_valid() || p_handle.index >= Capacity) {
			return nullptr;
		}

		const Slot &slot = slots[p_handle.index];
		if (!slot.is_active || slot.generation != p_handle.generation) {
			return nullptr;
		}

		return &slot.storage;
	}

	bool free(GenerationHandle p_handle) noexcept {
		if (!p_handle.is_valid() || p_handle.index >= Capacity) {
			return false;
		}

		Slot &slot = slots[p_handle.index];
		if (!slot.is_active || slot.generation != p_handle.generation) {
			return false; // Re-free or stale handle rejected
		}

		slot.is_active = false;
		slot.generation++; // Increment generation counter
		if (slot.generation == 0) slot.generation = 1; // Prevent 0 generation

		if (free_head > 0) {
			free_list[--free_head] = p_handle.index;
		}
		active_count--;
		return true;
	}

	void clear() noexcept {
		free_head = 0;
		active_count = 0;
		overflow_flag = false;
		for (size_t i = 0; i < Capacity; ++i) {
			slots[i].is_active = false;
			slots[i].generation++;
			if (slots[i].generation == 0) slots[i].generation = 1;
			free_list[i] = static_cast<uint32_t>(i);
		}
	}

	[[nodiscard]] constexpr size_t get_capacity() const noexcept { return Capacity; }
	[[nodiscard]] size_t get_active_count() const noexcept { return active_count; }
	[[nodiscard]] bool has_overflowed() const noexcept { return overflow_flag; }
};

} // namespace Multinet

#endif // MULTINET_BOUNDED_POOL_H
