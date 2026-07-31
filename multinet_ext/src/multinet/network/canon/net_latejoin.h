#ifndef MULTINET_NET_LATEJOIN_H
#define MULTINET_NET_LATEJOIN_H

#include "multinet/network/canon/canon.h"
#include "../spatial/net_interest.h"

#include <cstdint>
#include <array>
#include <span>

namespace Multinet {

using SessionToken = uint64_t;

struct ReconnectSessionRecord {
	PlayerID player_id{ 0 };
	SessionID session_id{ 0 };
	SessionToken session_token{ 0 };
	uint64_t last_ack_sequence{ 0 };
	bool is_connected{ true };
	bool is_active{ false };
};

class LateJoinManager {
public:
	static constexpr size_t MAX_RECONNECT_SESSIONS = 128;

private:
	std::array<ReconnectSessionRecord, MAX_RECONNECT_SESSIONS> m_sessions{};
	LocalCanonMock *canon_ref{ nullptr };

	[[nodiscard]] ReconnectSessionRecord* find_session(PlayerID p_player_id) noexcept {
		for (size_t i = 0; i < MAX_RECONNECT_SESSIONS; ++i) {
			if (m_sessions[i].is_active && m_sessions[i].player_id == p_player_id) {
				return &m_sessions[i];
			}
		}
		return nullptr;
	}

public:
	LateJoinManager() = default;

	explicit LateJoinManager(LocalCanonMock *p_canon) : canon_ref(p_canon) {}

	void set_canon_ref(LocalCanonMock *p_canon) noexcept {
		canon_ref = p_canon;
	}

	bool register_new_session(PlayerID p_player_id, SessionID p_session_id, SessionToken p_token) noexcept {
		if (p_player_id == 0 || p_session_id == 0 || p_token == 0) return false;

		// Overwrite existing or find empty
		ReconnectSessionRecord* slot = find_session(p_player_id);
		if (!slot) {
			for (size_t i = 0; i < MAX_RECONNECT_SESSIONS; ++i) {
				if (!m_sessions[i].is_active) {
					slot = &m_sessions[i];
					break;
				}
			}
		}

		if (!slot) return false; // Full

		slot->player_id = p_player_id;
		slot->session_id = p_session_id;
		slot->session_token = p_token;
		slot->last_ack_sequence = canon_ref ? canon_ref->get_global_sequence() : 0;
		slot->is_connected = true;
		slot->is_active = true;

		return true;
	}

	bool handle_disconnect(PlayerID p_player_id) noexcept {
		ReconnectSessionRecord* slot = find_session(p_player_id);
		if (!slot) return false;

		slot->is_connected = false;
		return true;
	}

	bool attempt_reconnect(PlayerID p_player_id, SessionToken p_token, uint64_t &r_last_seq) noexcept {
		ReconnectSessionRecord* slot = find_session(p_player_id);
		if (!slot) return false;

		if (slot->session_token != p_token) {
			return false;
		}

		slot->is_connected = true;
		r_last_seq = slot->last_ack_sequence;
		return true;
	}

	// Populates a provided array with missed events up to its capacity, returns number of events copied
	template <size_t MaxOutput>
	size_t get_missed_events_for_reconnect(uint64_t p_client_last_seq, std::array<CanonicalEvent, MaxOutput> &r_missed_events) const noexcept {
		if (!canon_ref) return 0;

		size_t count = 0;
		size_t log_count = canon_ref->get_log_count();
		
		for (size_t i = 0; i < log_count; ++i) {
			const CanonicalEvent* evt = canon_ref->get_event(i);
			if (evt && evt->canonical_sequence > p_client_last_seq) {
				if (count < MaxOutput) {
					r_missed_events[count++] = *evt;
				} else {
					break; // Output buffer full
				}
			}
		}
		
		return count;
	}
};

} // namespace Multinet

#endif // MULTINET_NET_LATEJOIN_H
