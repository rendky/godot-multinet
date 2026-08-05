#ifndef MULTINET_BOUNDED_BACKGROUND_JOB_EXECUTOR_H
#define MULTINET_BOUNDED_BACKGROUND_JOB_EXECUTOR_H

#include "job_system.h"

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <thread>

namespace Multinet {

// Owned by a shared runtime service; injected into subsystems that need
// background page generation. For WP5, owned by MultinetBCCMNode3D
// (intended to move to MultinétRuntime in a later work package).
class BoundedBackgroundJobExecutor {
public:
	static constexpr size_t QUEUE_CAPACITY = 256;

	BoundedBackgroundJobExecutor();
	~BoundedBackgroundJobExecutor();

	BoundedBackgroundJobExecutor(const BoundedBackgroundJobExecutor&) = delete;
	BoundedBackgroundJobExecutor& operator=(const BoundedBackgroundJobExecutor&) = delete;

	// Returns false when the queue is full (overflow) or executor has stopped.
	[[nodiscard]] bool submit(
		JobPriority priority,
		std::function<void()> work,
		JobToken* token = nullptr
	);

	// Signals the worker to stop and blocks until it joins.
	void shutdown();

	[[nodiscard]] size_t pending_count() const noexcept;
	[[nodiscard]] bool is_running() const noexcept;
	[[nodiscard]] bool is_idle() const noexcept;
	bool wait_idle_for(std::chrono::milliseconds timeout) const noexcept;

private:
	struct JobEntry {
		std::function<void()> work;
		JobPriority priority{ JobPriority::NORMAL };
		JobToken* token{ nullptr };
		bool active{ false };
	};

	std::array<JobEntry, QUEUE_CAPACITY> queue_{};
	size_t head_{ 0 };
	size_t tail_{ 0 };
	size_t count_{ 0 };
	bool overflow_{ false };

	mutable std::mutex mutex_;
	mutable std::condition_variable cv_;
	mutable std::condition_variable idle_cv_;
	std::thread worker_;
	std::atomic<bool> stopped_{ false };
	size_t executing_count_{ 0 };

	void worker_loop();
};

} // namespace Multinet

#endif // MULTINET_BOUNDED_BACKGROUND_JOB_EXECUTOR_H
