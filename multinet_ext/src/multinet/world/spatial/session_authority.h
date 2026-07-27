#ifndef MULTINET_SESSION_AUTHORITY_H
#define MULTINET_SESSION_AUTHORITY_H

#include <cstdint>

namespace Multinet {

enum class SessionAuthorityMode : uint8_t {
	HOST_AUTHORITY = 0,
	DEDICATED_AUTHORITY = 1,
	PEER_AUTHORITY = 2,
	VALIDATION_PEER = 3
};

struct SessionAuthorityConfig {
	SessionAuthorityMode mode{ SessionAuthorityMode::HOST_AUTHORITY };
	uint64_t session_id{ 0 };
	uint32_t max_peers{ 64 };
	bool allow_late_join{ true };
};

class SessionAuthorityAdapter {
private:
	SessionAuthorityConfig config{};
	bool is_active{ false };

public:
	SessionAuthorityAdapter() = default;

	explicit SessionAuthorityAdapter(const SessionAuthorityConfig &p_config)
		: config(p_config), is_active(true) {}

	void initialize(const SessionAuthorityConfig &p_config) noexcept {
		config = p_config;
		is_active = true;
	}

	[[nodiscard]] SessionAuthorityMode get_mode() const noexcept { return config.mode; }
	[[nodiscard]] uint64_t get_session_id() const noexcept { return config.session_id; }
	[[nodiscard]] bool is_initialized() const noexcept { return is_active; }
};

} // namespace Multinet

#endif // MULTINET_SESSION_AUTHORITY_H
