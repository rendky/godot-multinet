#include "bounded_background_job_executor.h"

namespace Multinet {

BoundedBackgroundJobExecutor::BoundedBackgroundJobExecutor(size_t worker_count)
	: worker_count_(worker_count == 0 ? 1 : worker_count) {
	workers_.reserve(worker_count_);
	for (size_t i = 0; i < worker_count_; ++i) {
		workers_.emplace_back([this] { worker_loop(); });
	}
}

BoundedBackgroundJobExecutor::~BoundedBackgroundJobExecutor() {
	shutdown();
}

bool BoundedBackgroundJobExecutor::submit(
	JobPriority priority,
	std::function<void()> work,
	JobToken* token
) {
	std::unique_lock<std::mutex> lock(mutex_);
	if (stopped_.load(std::memory_order_acquire)) return false;

	JobEntry entry;
	entry.work = std::move(work);
	entry.priority = priority;
	entry.token = token;
	entry.active = true;

	bool ok = false;
	if (priority == JobPriority::HIGH) {
		ok = lane_high_.enqueue(std::move(entry));
	} else if (priority == JobPriority::LOW) {
		ok = lane_low_.enqueue(std::move(entry));
	} else {
		ok = lane_normal_.enqueue(std::move(entry));
	}

	if (!ok) {
		overflow_ = true;
		return false;
	}

	lock.unlock();
	cv_.notify_all();
	return true;
}

void BoundedBackgroundJobExecutor::shutdown() {
	{
		std::lock_guard<std::mutex> lock(mutex_);
		if (stopped_.load(std::memory_order_relaxed)) return;
		stopped_.store(true, std::memory_order_release);
	}
	cv_.notify_all();
	for (auto& worker : workers_) {
		if (worker.joinable()) {
			worker.join();
		}
	}
	workers_.clear();
}

size_t BoundedBackgroundJobExecutor::pending_count() const noexcept {
	std::lock_guard<std::mutex> lock(mutex_);
	return lane_high_.count + lane_normal_.count + lane_low_.count;
}

size_t BoundedBackgroundJobExecutor::high_priority_count() const noexcept {
	std::lock_guard<std::mutex> lock(mutex_);
	return lane_high_.count;
}

size_t BoundedBackgroundJobExecutor::normal_priority_count() const noexcept {
	std::lock_guard<std::mutex> lock(mutex_);
	return lane_normal_.count;
}

size_t BoundedBackgroundJobExecutor::low_priority_count() const noexcept {
	std::lock_guard<std::mutex> lock(mutex_);
	return lane_low_.count;
}

size_t BoundedBackgroundJobExecutor::executing_worker_count() const noexcept {
	std::lock_guard<std::mutex> lock(mutex_);
	return executing_count_;
}

bool BoundedBackgroundJobExecutor::is_running() const noexcept {
	return !stopped_.load(std::memory_order_acquire);
}

bool BoundedBackgroundJobExecutor::is_idle() const noexcept {
	std::lock_guard<std::mutex> lock(mutex_);
	size_t pending = lane_high_.count + lane_normal_.count + lane_low_.count;
	return pending == 0 && executing_count_ == 0;
}

bool BoundedBackgroundJobExecutor::wait_idle_for(std::chrono::milliseconds timeout) const noexcept {
	std::unique_lock<std::mutex> lock(mutex_);
	return idle_cv_.wait_for(lock, timeout, [this] {
		size_t pending = lane_high_.count + lane_normal_.count + lane_low_.count;
		return pending == 0 && executing_count_ == 0;
	});
}

void BoundedBackgroundJobExecutor::worker_loop() {
	while (true) {
		JobEntry job;
		{
			std::unique_lock<std::mutex> lock(mutex_);
			cv_.wait(lock, [this] {
				size_t pending = lane_high_.count + lane_normal_.count + lane_low_.count;
				return pending > 0 || stopped_.load(std::memory_order_acquire);
			});

			size_t pending = lane_high_.count + lane_normal_.count + lane_low_.count;
			if (stopped_.load(std::memory_order_acquire) && pending == 0) break;
			if (pending == 0) continue;

			// Weighted round-robin priority dequeue: after 4 consecutive HIGH jobs, serve 1 NORMAL job if available
			bool dequeued = false;
			if (high_consecutive_count_ >= 4 && lane_normal_.count > 0) {
				dequeued = lane_normal_.dequeue(job);
				if (dequeued) {
					high_consecutive_count_ = 0;
				}
			}

			if (!dequeued) {
				dequeued = lane_high_.dequeue(job);
				if (dequeued) {
					++high_consecutive_count_;
				}
			}

			if (!dequeued) {
				dequeued = lane_normal_.dequeue(job);
				if (dequeued) {
					high_consecutive_count_ = 0;
				}
			}

			if (!dequeued) {
				dequeued = lane_low_.dequeue(job);
				if (dequeued) {
					high_consecutive_count_ = 0;
				}
			}

			if (dequeued && job.active) {
				++executing_count_;
			}
		}

		if (job.active) {
			if (!job.token || !job.token->is_cancelled()) {
				if (job.work) {
					job.work();
				}
				if (job.token) {
					job.token->completed.store(true, std::memory_order_release);
				}
			}
			{
				std::lock_guard<std::mutex> lock(mutex_);
				--executing_count_;
			}
			idle_cv_.notify_all();
		}
	}
}

} // namespace Multinet
