#ifndef MULTINET_ARENA_ALLOCATOR_H
#define MULTINET_ARENA_ALLOCATOR_H

#include <cstddef>
#include <cstdint>

namespace Multinet {

class ArenaAllocator {
private:
	uint8_t *buffer{ nullptr };
	size_t capacity{ 0 };
	size_t offset{ 0 };

public:
	ArenaAllocator() = default;

	explicit ArenaAllocator(size_t p_capacity) {
		capacity = p_capacity;
		if (capacity > 0) {
			buffer = new uint8_t[capacity];
		}
	}

	~ArenaAllocator() {
		if (buffer) {
			delete[] buffer;
			buffer = nullptr;
		}
	}

	ArenaAllocator(const ArenaAllocator &) = delete;
	ArenaAllocator &operator=(const ArenaAllocator &) = delete;

	ArenaAllocator(ArenaAllocator &&p_other) noexcept
		: buffer(p_other.buffer), capacity(p_other.capacity), offset(p_other.offset) {
		p_other.buffer = nullptr;
		p_other.capacity = 0;
		p_other.offset = 0;
	}

	ArenaAllocator &operator=(ArenaAllocator &&p_other) noexcept {
		if (this != &p_other) {
			if (buffer) delete[] buffer;
			buffer = p_other.buffer;
			capacity = p_other.capacity;
			offset = p_other.offset;
			p_other.buffer = nullptr;
			p_other.capacity = 0;
			p_other.offset = 0;
		}
		return *this;
	}

	void reserve(size_t p_capacity) {
		if (buffer) delete[] buffer;
		capacity = p_capacity;
		offset = 0;
		if (capacity > 0) {
			buffer = new uint8_t[capacity];
		}
	}

	template <typename T>
	[[nodiscard]] T *allocate(size_t p_count = 1, size_t p_alignment = alignof(T)) {
		if (!buffer || capacity == 0) return nullptr;

		uintptr_t current_ptr = reinterpret_cast<uintptr_t>(buffer + offset);
		size_t padding = (p_alignment - (current_ptr % p_alignment)) % p_alignment;
		size_t total_size = (sizeof(T) * p_count) + padding;

		if (offset + total_size > capacity) {
			return nullptr; // Arena exhaustion
		}

		uint8_t *aligned_ptr = buffer + offset + padding;
		offset += total_size;
		return reinterpret_cast<T *>(aligned_ptr);
	}

	void reset() noexcept {
		offset = 0;
	}

	[[nodiscard]] size_t get_capacity() const noexcept { return capacity; }
	[[nodiscard]] size_t get_used() const noexcept { return offset; }
	[[nodiscard]] size_t get_remaining() const noexcept { return capacity - offset; }
};

} // namespace Multinet

#endif // MULTINET_ARENA_ALLOCATOR_H
