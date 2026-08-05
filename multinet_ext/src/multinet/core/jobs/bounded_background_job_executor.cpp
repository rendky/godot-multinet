#include "bounded_background_job_executor.h"

namespace Multinet {

BoundedBackgroundJobExecutor::BoundedBackgroundJobExecutor() {
	worker_ = std::thread([this] { worker_loop(); });
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

	if (count_ >= QUEUE_CAPACITY) {
		overflow_ = true;
		return false;
	}

	JobEntry& entry = queue_[tail_];
	entry.work = std::move(work);
	entry.priority = priority;
	entry.token = token;
	entry.active = true;

	tail_ = (tail_ + 1) % QUEUE_CAPACITY;
	++count_;
	lock.unlock();
	cv_.notify_one();
	return true;
}

void BoundedBackgroundJobExecutor::shutdown() {
	{
		std::lock_guard<std::mutex> lock(mutex_);
		if (stopped_.load(std::memory_order_relaxed)) return;
		stopped_.store(true, std::memory_order_release);
	}
	cv_.notify_all();
	if (worker_.joinable()) {
		worker_.join();
	}
}

size_t BoundedBackgroundJobExecutor::pending_count() const noexcept {
	std::lock_guard<std::mutex> lock(mutex_);
	return count_;
}

bool BoundedBackgroundJobExecutor::is_running() const noexcept {
	return !stopped_.load(std::memory_order_acquire);
}

bool BoundedBackgroundJobExecutor::is_idle() const noexcept {
	std::lock_guard<std::mutex> lock(mutex_);
	return count_ == 0 && executing_count_ == 0;
}

bool BoundedBackgroundJobExecutor::wait_idle_for(std::chrono::milliseconds timeout) const noexcept {
	std::unique_lock<std::mutex> lock(mutex_);
	return idle_cv_.wait_for(lock, timeout, [this] {
		return count_ == 0 && executing_count_ == 0;
	});
}

void BoundedBackgroundJobExecutor::worker_loop() {
	while (true) {
		JobEntry job;
		{
			std::unique_lock<std::mutex> lock(mutex_);
			cv_.wait(lock, [this] {
				return count_ > 0 || stopped_.load(std::memory_order_acquire);
			});

			if (stopped_.load(std::memory_order_acquire) && count_ == 0) break;
			if (count_ == 0) continue;

			job = std::move(queue_[head_]);
			queue_[head_].active = false;
			head_ = (head_ + 1) % QUEUE_CAPACITY;
			--count_;
			if (job.active) {
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
