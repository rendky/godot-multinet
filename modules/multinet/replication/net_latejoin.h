#ifndef MULTINET_NET_LATEJOIN_H
#define MULTINET_NET_LATEJOIN_H

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

struct CanonicalEventRecord {
	uint64_t event_id{ 0 };
	uint64_t sequence_num{ 0 };
};

class LateJoinManager {
private:
	std::unordered_map<PlayerID, ReconnectSessionRecord> sessions;
	std::vector<CanonicalEventRecord> event_log;
	uint64_t current_server_sequence{ 0 };

public:
	LateJoinManager() = default;

	bool register_new_session(PlayerID p_player_id, SessionID p_session_id, SessionToken p_token) noexcept {
		if (p_player_id == 0 || p_session_id == 0 || p_token == 0) return false;

		ReconnectSessionRecord rec;
		rec.player_id = p_player_id;
		rec.session_id = p_session_id;
		rec.session_token = p_token;
		rec.last_ack_sequence = current_server_sequence;
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
			return false; // Token mismatch rejection!
		}

		rec.is_connected = true;
		r_last_seq = rec.last_ack_sequence;
		return true; // Session successfully recovered
	}

	void record_canonical_event(uint64_t p_event_id) noexcept {
		current_server_sequence++;
		event_log.push_back(CanonicalEventRecord{ p_event_id, current_server_sequence });

		// Cap event log to last 256 events
		if (event_log.size() > 256) {
			event_log.erase(event_log.begin());
		}
	}

	size_t get_missed_events_for_reconnect(uint64_t p_client_last_seq, std::vector<uint64_t> &r_missed_events) const noexcept {
		r_missed_events.clear();
		for (const auto &rec : event_log) {
			if (rec.sequence_num > p_client_last_seq) {
				r_missed_events.push_back(rec.event_id);
			}
		}
		return r_missed_events.size();
	}

	[[nodiscard]] uint64_t get_current_sequence() const noexcept { return current_server_sequence; }
};

} // namespace Multinet

#endif // MULTINET_NET_LATEJOIN_H
