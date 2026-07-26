#ifndef MULTINET_QUALITY_AUTHORITY_H
#define MULTINET_QUALITY_AUTHORITY_H

#include <cstdint>

namespace Multinet {

enum class QualityTier : uint8_t {
	ULTRA = 0,
	HIGH = 1,
	MEDIUM = 2,
	LOW = 3,
	MINIMAL = 4,
	COUNT = 5
};

class QualityAuthorityManager {
private:
	QualityTier active_tier{ QualityTier::HIGH };
	uint64_t last_transition_ms{ 0 };
	uint64_t hysteresis_cooldown_ms{ 2000 }; // 2-second hysteresis cooldown
	uint32_t transition_count{ 0 };

public:
	QualityAuthorityManager() = default;

	explicit QualityAuthorityManager(QualityTier p_initial_tier, uint64_t p_cooldown_ms = 2000)
		: active_tier(p_initial_tier), hysteresis_cooldown_ms(p_cooldown_ms) {}

	// Requests quality tier change with hysteresis validation
	bool request_tier_change(QualityTier p_requested_tier, uint64_t p_current_time_ms) noexcept {
		if (p_requested_tier == active_tier) {
			return false; // No change needed
		}

		// Enforce hysteresis cooldown after initial placement
		if (last_transition_ms > 0 && (p_current_time_ms - last_transition_ms < hysteresis_cooldown_ms)) {
			return false; // Rejected by hysteresis cooldown
		}

		active_tier = p_requested_tier;
		last_transition_ms = p_current_time_ms;
		transition_count++;
		return true; // Transition accepted
	}

	[[nodiscard]] QualityTier get_active_tier() const noexcept { return active_tier; }
	[[nodiscard]] uint64_t get_last_transition_ms() const noexcept { return last_transition_ms; }
	[[nodiscard]] uint32_t get_transition_count() const noexcept { return transition_count; }
};

} // namespace Multinet

#endif // MULTINET_QUALITY_AUTHORITY_H
