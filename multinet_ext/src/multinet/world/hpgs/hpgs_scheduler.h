#ifndef MULTINET_HPGS_SCHEDULER_H
#define MULTINET_HPGS_SCHEDULER_H

#include "hpgs_types.h"
#include <array>
#include <cstddef>

namespace Multinet {

// ============================================================================
// HPGSScheduler - H0/H1 Synthetic Scheduler (Task R2)
// Bounded admission, prediction epochs, cancellation, stale-result rejection,
// and previous-valid retention.
// ============================================================================

class HPGSScheduler {
public:
	static constexpr size_t MAX_BOUNDED_REQUESTS = 64;

	constexpr HPGSScheduler() noexcept = default;

	[[nodiscard]] constexpr bool submit_request(const HPGSRequest &p_request) noexcept {
		if (m_request_count >= MAX_BOUNDED_REQUESTS) {
			m_rejected_overflow_count++;
			return false;
		}
		m_requests[m_request_count++] = p_request;
		m_admitted_count++;
		return true;
	}

	constexpr void set_current_epoch(uint64_t p_epoch) noexcept {
		m_current_epoch = p_epoch;
	}

	[[nodiscard]] constexpr uint64_t get_current_epoch() const noexcept {
		return m_current_epoch;
	}

	[[nodiscard]] constexpr HPGSResult process_request(const HPGSRequest &p_request, bool p_is_cancelled = false) noexcept {
		HPGSResult result{};
		result.key = p_request.key;
		result.epoch = p_request.epoch;

		if (p_is_cancelled) {
			result.is_cancelled = true;
			m_cancelled_count++;
			return result;
		}

		if (p_request.epoch < m_current_epoch) {
			result.is_stale = true;
			m_stale_rejected_count++;
			return result;
		}

		result.generator_version = 1;
		m_committed_count++;
		m_last_valid_result = result;
		return result;
	}

	constexpr void reset_queue() noexcept {
		m_request_count = 0;
	}

	// Telemetry query getters
	[[nodiscard]] constexpr uint32_t get_admitted_count() const noexcept { return m_admitted_count; }
	[[nodiscard]] constexpr uint32_t get_cancelled_count() const noexcept { return m_cancelled_count; }
	[[nodiscard]] constexpr uint32_t get_stale_rejected_count() const noexcept { return m_stale_rejected_count; }
	[[nodiscard]] constexpr uint32_t get_committed_count() const noexcept { return m_committed_count; }
	[[nodiscard]] constexpr HPGSResult get_last_valid_result() const noexcept { return m_last_valid_result; }

private:
	std::array<HPGSRequest, MAX_BOUNDED_REQUESTS> m_requests{};
	size_t m_request_count{ 0 };
	uint64_t m_current_epoch{ 0 };

	HPGSResult m_last_valid_result{};

	uint32_t m_admitted_count{ 0 };
	uint32_t m_cancelled_count{ 0 };
	uint32_t m_stale_rejected_count{ 0 };
	uint32_t m_committed_count{ 0 };
	uint32_t m_rejected_overflow_count{ 0 };
};

} // namespace Multinet

#endif // MULTINET_HPGS_SCHEDULER_H
