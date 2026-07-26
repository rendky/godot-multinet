#ifndef MULTINET_SNAPSHOT_PUBLISHER_H
#define MULTINET_SNAPSHOT_PUBLISHER_H

#include <atomic>
#include <cstdint>
#include <utility>

namespace Multinet {

template <typename T>
class SnapshotPublisher {
private:
	T buffers[2]{};
	std::atomic<uint32_t> active_index{ 0 };
	std::atomic<uint64_t> version{ 0 };

public:
	SnapshotPublisher() = default;

	explicit SnapshotPublisher(const T &p_initial_state) {
		buffers[0] = p_initial_state;
		buffers[1] = p_initial_state;
	}

	// Worker thread writes to back buffer
	T &get_back_buffer() noexcept {
		uint32_t current_active = active_index.load(std::memory_order_relaxed);
		return buffers[1 - current_active];
	}

	// Worker thread publishes new state atomically
	uint64_t publish() noexcept {
		uint32_t current_active = active_index.load(std::memory_order_relaxed);
		uint32_t next_active = 1 - current_active;

		// Atomic index swap
		active_index.store(next_active, std::memory_order_release);
		uint64_t new_version = version.fetch_add(1, std::memory_order_acq_rel) + 1;

		// Propagate latest published state into new back buffer for next update cycle
		buffers[current_active] = buffers[next_active];

		return new_version;
	}

	// Consumer (Main thread) reads last valid immutable snapshot
	[[nodiscard]] const T &get_read_snapshot() const noexcept {
		uint32_t current_active = active_index.load(std::memory_order_acquire);
		return buffers[current_active];
	}

	[[nodiscard]] uint64_t get_version() const noexcept {
		return version.load(std::memory_order_acquire);
	}
};

} // namespace Multinet

#endif // MULTINET_SNAPSHOT_PUBLISHER_H
