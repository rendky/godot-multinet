#ifndef MULTINET_NET_INTEREST_H
#define MULTINET_NET_INTEREST_H

#include "multinet/core/coordinates.h"

#include <cmath>
#include <cstdint>
#include <array>

namespace Multinet {

using PlayerID = uint64_t;
using SessionID = uint64_t;

struct PlayerSessionState {
	PlayerID player_id{ 0 };
	SessionID session_id{ 0 };
	RegionPosition position{};
	uint32_t interest_radius_cells{ 1 }; // Default 1 cell radius (3x3 grid)
	bool is_active{ false };
};

class SpatialInterestGrid {
public:
	static constexpr size_t MAX_SESSIONS = 128;

private:
	std::array<PlayerSessionState, MAX_SESSIONS> m_sessions{};
	size_t m_active_count{ 0 };

	[[nodiscard]] PlayerSessionState* find_session(PlayerID p_player_id) noexcept {
		for (size_t i = 0; i < MAX_SESSIONS; ++i) {
			if (m_sessions[i].is_active && m_sessions[i].player_id == p_player_id) {
				return &m_sessions[i];
			}
		}
		return nullptr;
	}

	[[nodiscard]] const PlayerSessionState* find_session(PlayerID p_player_id) const noexcept {
		for (size_t i = 0; i < MAX_SESSIONS; ++i) {
			if (m_sessions[i].is_active && m_sessions[i].player_id == p_player_id) {
				return &m_sessions[i];
			}
		}
		return nullptr;
	}

public:
	SpatialInterestGrid() = default;

	bool register_player(PlayerID p_player_id, SessionID p_session_id, RegionPosition p_initial_pos, uint32_t p_radius_cells = 1) noexcept {
		if (p_player_id == 0 || p_session_id == 0) return false;

		// Don't register duplicates
		if (find_session(p_player_id) != nullptr) return false;

		// Find empty slot
		for (size_t i = 0; i < MAX_SESSIONS; ++i) {
			if (!m_sessions[i].is_active) {
				m_sessions[i].player_id = p_player_id;
				m_sessions[i].session_id = p_session_id;
				m_sessions[i].position = p_initial_pos;
				m_sessions[i].interest_radius_cells = p_radius_cells;
				m_sessions[i].is_active = true;
				m_active_count++;
				return true;
			}
		}
		return false; // Grid full
	}

	bool unregister_player(PlayerID p_player_id) noexcept {
		PlayerSessionState* session = find_session(p_player_id);
		if (session) {
			session->is_active = false;
			session->player_id = 0;
			m_active_count--;
			return true;
		}
		return false;
	}

	bool update_player_position(PlayerID p_player_id, RegionPosition p_new_pos) noexcept {
		PlayerSessionState* session = find_session(p_player_id);
		if (session) {
			session->position = p_new_pos;
			return true;
		}
		return false;
	}

	[[nodiscard]] bool is_in_interest_range(PlayerID p_player_id, RegionPosition p_entity_pos) const noexcept {
		const PlayerSessionState* player = find_session(p_player_id);
		if (!player) return false;

		int64_t dx = std::abs(static_cast<int64_t>(p_entity_pos.cell_x) - static_cast<int64_t>(player->position.cell_x));
		int64_t dy = std::abs(static_cast<int64_t>(p_entity_pos.cell_y) - static_cast<int64_t>(player->position.cell_y));
		int64_t dz = std::abs(static_cast<int64_t>(p_entity_pos.cell_z) - static_cast<int64_t>(player->position.cell_z));

		int64_t radius = static_cast<int64_t>(player->interest_radius_cells);
		return (dx <= radius && dy <= radius && dz <= radius);
	}

	[[nodiscard]] size_t get_active_player_count() const noexcept { return m_active_count; }
};

} // namespace Multinet

#endif // MULTINET_NET_INTEREST_H
