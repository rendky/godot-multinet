#ifndef MULTINET_NET_RECONCILIATION_H
#define MULTINET_NET_RECONCILIATION_H

#include "coordinates.h"

#include <cmath>
#include <cstdint>

namespace Multinet {

struct TransformSnapshotPacket {
	uint64_t sequence_num{ 0 };
	uint64_t timestamp_ms{ 0 };
	WorldPosition64 world_pos{};
	QuantizedTransform transform{};
};

class ClientReconciler {
private:
	uint64_t last_server_seq{ 0 };
	WorldPosition64 last_authoritative_pos{};
	bool has_received_initial{ false };

public:
	ClientReconciler() = default;

	// Validates incoming server snapshot sequence & rejects out-of-order/duplicate packets
	bool process_server_snapshot(const TransformSnapshotPacket &p_snapshot) noexcept {
		if (has_received_initial && p_snapshot.sequence_num <= last_server_seq) {
			return false; // Reject duplicate or out-of-order server snapshot packet!
		}

		last_server_seq = p_snapshot.sequence_num;
		last_authoritative_pos = p_snapshot.world_pos;
		has_received_initial = true;
		return true;
	}

	// Calculates spatial error delta between client predicted position and server authoritative position
	[[nodiscard]] double calculate_error_distance(const WorldPosition64 &p_predicted_pos) const noexcept {
		if (!has_received_initial) return 0.0;

		double dx = p_predicted_pos.x - last_authoritative_pos.x;
		double dy = p_predicted_pos.y - last_authoritative_pos.y;
		double dz = p_predicted_pos.z - last_authoritative_pos.z;
		return std::sqrt(dx * dx + dy * dy + dz * dz);
	}

	// Smoothly interpolates client position toward server authoritative position
	[[nodiscard]] WorldPosition64 apply_smooth_correction(const WorldPosition64 &p_client_pos, float p_alpha = 0.2f) const noexcept {
		if (!has_received_initial) return p_client_pos;

		double factor = static_cast<double>(p_alpha);
		return WorldPosition64{
			p_client_pos.x + (last_authoritative_pos.x - p_client_pos.x) * factor,
			p_client_pos.y + (last_authoritative_pos.y - p_client_pos.y) * factor,
			p_client_pos.z + (last_authoritative_pos.z - p_client_pos.z) * factor
		};
	}

	[[nodiscard]] uint64_t get_last_sequence() const noexcept { return last_server_seq; }
	[[nodiscard]] const WorldPosition64 &get_authoritative_position() const noexcept { return last_authoritative_pos; }
};

} // namespace Multinet

#endif // MULTINET_NET_RECONCILIATION_H
