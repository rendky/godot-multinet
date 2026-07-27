#ifndef MULTINET_NET_LATEJOIN_H
#define MULTINET_NET_LATEJOIN_H

#include "multinet/network/canon/canon.h"
#include "spatial/net_interest.h"

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace Multinet {

using SessionToken = uint64_t;

struct ReconnectSessionRecord {
	PlayerID player_id{ 0 };
	SessionID session_id{ 0 };
	SessionToken session_token{ 0 };
	uint64_t last_ack_sequence{ 0 };
	bool is_connected{ true };
};

class LateJoinManager {
private:
	std::unordered_map<PlayerID, ReconnectSessionRecord> sessions;
	LocalCanonMock *canon_ref{ nullptr };

public:
	LateJoinManager() = default;

	explicit LateJoinManager(LocalCanonMock *p_canon) : canon_ref(p_canon) {}

	void set_canon_ref(LocalCanonMock *p_canon) noexcept {
		canon_ref = p_canon;
	}

	bool register_new_session(PlayerID p_player_id, SessionID p_session_id, SessionToken p_token) noexcept {
		if (p_player_id == 0 || p_session_id == 0 || p_token == 0) return false;

		ReconnectSessionRecord rec;
		rec.player_id = p_player_id;
		rec.session_id = p_session_id;
		rec.session_token = p_token;
		rec.last_ack_sequence = canon_ref ? canon_ref->get_global_sequence() : 0;
		rec.is_connected = true;

		sessions[p_player_id] = rec;
		return true;
	}

	bool handle_disconnect(PlayerID p_player_id) noexcept {
		auto it = sessions.find(p_player_id);
		if (it == sessions.end()) return false;

		it->second.is_connected = false;
		return true;
	}

	bool attempt_reconnect(PlayerID p_player_id, SessionToken p_token, uint64_t &r_last_seq) noexcept {
		auto it = sessions.find(p_player_id);
		if (it == sessions.end()) return false;

		ReconnectSessionRecord &rec = it->second;
		if (rec.session_token != p_token) {
			return false;
		}

		rec.is_connected = true;
		r_last_seq = rec.last_ack_sequence;
		return true;
	}

	size_t get_missed_events_for_reconnect(uint64_t p_client_last_seq, std::vector<CanonicalEvent> &r_missed_events) const noexcept {
		r_missed_events.clear();
		if (!canon_ref) return 0;

		for (const auto &evt : canon_ref->get_event_log()) {
			if (evt.canonical_sequence > p_client_last_seq) {
				r_missed_events.push_back(evt);
			}
		}
		return r_missed_events.size();
	}
};

} // namespace Multinet

#endif // MULTINET_NET_LATEJOIN_H
