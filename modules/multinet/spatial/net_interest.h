#ifndef MULTINET_NET_INTEREST_H
#define MULTINET_NET_INTEREST_H

#include "coordinates.h"

#include <cmath>
#include <cstdint>
#include <unordered_map>

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
private:
	std::unordered_map<PlayerID, PlayerSessionState> active_sessions;

public:
	SpatialInterestGrid() = default;

	bool register_player(PlayerID p_player_id, SessionID p_session_id, RegionPosition p_initial_pos, uint32_t p_radius_cells = 1) noexcept {
		if (p_player_id == 0 || p_session_id == 0) return false;

		PlayerSessionState state;
		state.player_id = p_player_id;
		state.session_id = p_session_id;
		state.position = p_initial_pos;
		state.interest_radius_cells = p_radius_cells;
		state.is_active = true;

		active_sessions[p_player_id] = state;
		return true;
	}

	bool unregister_player(PlayerID p_player_id) noexcept {
		return active_sessions.erase(p_player_id) > 0;
	}

	bool update_player_position(PlayerID p_player_id, RegionPosition p_new_pos) noexcept {
		auto it = active_sessions.find(p_player_id);
		if (it == active_sessions.end() || !it->second.is_active) return false;

		it->second.position = p_new_pos;
		return true;
	}

	[[nodiscard]] bool is_in_interest_range(PlayerID p_player_id, RegionPosition p_entity_pos) const noexcept {
		auto it = active_sessions.find(p_player_id);
		if (it == active_sessions.end() || !it->second.is_active) return false;

		const PlayerSessionState &player = it->second;
		int64_t dx = std::abs(static_cast<int64_t>(p_entity_pos.cell_x) - static_cast<int64_t>(player.position.cell_x));
		int64_t dy = std::abs(static_cast<int64_t>(p_entity_pos.cell_y) - static_cast<int64_t>(player.position.cell_y));
		int64_t dz = std::abs(static_cast<int64_t>(p_entity_pos.cell_z) - static_cast<int64_t>(player.position.cell_z));

		int64_t radius = static_cast<int64_t>(player.interest_radius_cells);
		return (dx <= radius && dy <= radius && dz <= radius);
	}

	[[nodiscard]] size_t get_active_player_count() const noexcept { return active_sessions.size(); }
};

} // namespace Multinet

#endif // MULTINET_NET_INTEREST_H
