#ifndef MULTINET_TYPED_EVENTS_H
#define MULTINET_TYPED_EVENTS_H

#include <cstddef>
#include <cstdint>
#include <utility>

namespace Multinet {

enum class EventCategory : uint8_t {
	CANONICAL = 0,   // Durable state-altering events (idempotent)
	SIMULATION = 1,  // Transient simulation events
	PRESENTATION = 2,// Visual/audio cosmetic events
	DIAGNOSTIC = 3   // Logging/profiling events
};

struct EventHeader {
	uint64_t event_id{ 0 };
	uint64_t sequence_num{ 0 };
	uint32_t creator_id{ 0 };
	EventCategory category{ EventCategory::SIMULATION };
};

template <size_t HistoryCapacity = 128>
class EventDeduplicator {
private:
	uint64_t history[HistoryCapacity]{};
	size_t head{ 0 };
	size_t count{ 0 };

public:
	EventDeduplicator() = default;

	// Checks if event_id has already been processed (idempotency check)
	[[nodiscard]] bool is_duplicate(uint64_t p_event_id) const noexcept {
		for (size_t i = 0; i < count; ++i) {
			if (history[i] == p_event_id) {
				return true; // Duplicate canonical event detected
			}
		}
		return false;
	}

	// Registers processed canonical event ID into ring buffer
	void record(uint64_t p_event_id) noexcept {
		if (is_duplicate(p_event_id)) return;

		history[head] = p_event_id;
		head = (head + 1) % HistoryCapacity;
		if (count < HistoryCapacity) {
			count++;
		}
	}

	void clear() noexcept {
		head = 0;
		count = 0;
		for (size_t i = 0; i < HistoryCapacity; ++i) {
			history[i] = 0;
		}
	}
};

template <typename T, size_t Capacity = 64>
class BoundedEventQueue {
private:
	struct EventSlot {
		EventHeader header;
		T payload;
		bool is_active{ false };
	};

	EventSlot slots[Capacity]{};
	size_t head{ 0 };
	size_t tail{ 0 };
	size_t count{ 0 };
	bool overflow_flag{ false };

public:
	BoundedEventQueue() = default;

	bool push(const EventHeader &p_header, T p_payload) noexcept {
		if (count >= Capacity) {
			overflow_flag = true;
			return false; // Overflow allocation cap
		}

		EventSlot &slot = slots[tail];
		slot.header = p_header;
		slot.payload = std::move(p_payload);
		slot.is_active = true;

		tail = (tail + 1) % Capacity;
		count++;
		return true;
	}

	bool pop(EventHeader &r_header, T &r_payload) noexcept {
		if (count == 0) return false;

		EventSlot &slot = slots[head];
		head = (head + 1) % Capacity;
		count--;

		r_header = slot.header;
		r_payload = std::move(slot.payload);
		slot.is_active = false;
		return true;
	}

	[[nodiscard]] size_t get_count() const noexcept { return count; }
	[[nodiscard]] constexpr size_t get_capacity() const noexcept { return Capacity; }
	[[nodiscard]] bool has_overflowed() const noexcept { return overflow_flag; }
};

} // namespace Multinet

#endif // MULTINET_TYPED_EVENTS_H
