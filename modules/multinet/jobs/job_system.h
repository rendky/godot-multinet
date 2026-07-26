#ifndef MULTINET_JOB_SYSTEM_H
#define MULTINET_JOB_SYSTEM_H

#include "memory/generation_handle.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>

namespace Multinet {

enum class JobPriority : uint8_t {
	HIGH = 0,
	NORMAL = 1,
	LOW = 2,
	COUNT = 3
};

struct JobToken {
	uint32_t id{ 0 };
	std::atomic<bool> cancelled{ false };
	std::atomic<bool> completed{ false };

	void cancel() noexcept {
		cancelled.store(true, std::memory_order_release);
	}

	[[nodiscard]] bool is_cancelled() const noexcept {
		return cancelled.load(std::memory_order_acquire);
	}

	[[nodiscard]] bool is_completed() const noexcept {
		return completed.load(std::memory_order_acquire);
	}
};

template <size_t MaxJobs = 256>
class BoundedJobQueue {
private:
	struct JobSlot {
		std::function<void()> work;
		JobPriority priority{ JobPriority::NORMAL };
		JobToken *token{ nullptr };
		bool is_active{ false };
	};

	JobSlot slots[MaxJobs]{};
	size_t count{ 0 };
	size_t head{ 0 };
	size_t tail{ 0 };
	bool overflow_flag{ false };

public:
	BoundedJobQueue() = default;

	bool enqueue(JobPriority p_priority, std::function<void()> p_work, JobToken *p_token = nullptr) {
		if (count >= MaxJobs) {
			overflow_flag = true;
			return false; // Bounded queue exhaustion fallback
		}

		JobSlot &slot = slots[tail];
		slot.work = std::move(p_work);
		slot.priority = p_priority;
		slot.token = p_token;
		slot.is_active = true;

		tail = (tail + 1) % MaxJobs;
		count++;
		return true;
	}

	bool dequeue_and_execute() {
		if (count == 0) return false;

		JobSlot &slot = slots[head];
		head = (head + 1) % MaxJobs;
		count--;

		if (slot.is_active) {
			if (!slot.token || !slot.token->is_cancelled()) {
				if (slot.work) {
					slot.work();
				}
				if (slot.token) {
					slot.token->completed.store(true, std::memory_order_release);
				}
			}
			slot.is_active = false;
		}

		return true;
	}

	void clear() noexcept {
		count = 0;
		head = 0;
		tail = 0;
		overflow_flag = false;
		for (size_t i = 0; i < MaxJobs; ++i) {
			slots[i].is_active = false;
		}
	}

	[[nodiscard]] size_t get_count() const noexcept { return count; }
	[[nodiscard]] constexpr size_t get_capacity() const noexcept { return MaxJobs; }
	[[nodiscard]] bool has_overflowed() const noexcept { return overflow_flag; }
};

} // namespace Multinet

#endif // MULTINET_JOB_SYSTEM_H
