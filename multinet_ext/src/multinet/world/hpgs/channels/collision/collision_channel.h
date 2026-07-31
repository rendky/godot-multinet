#ifndef MULTINET_COLLISION_CHANNEL_H
#define MULTINET_COLLISION_CHANNEL_H

#include "../../../structure/structure_package.h"
#include "../../hpgs_types.h"
#include <array>
#include <cstddef>
#include <cstdint>

namespace Multinet {

enum class CollisionEnvelopeState : uint8_t {
	Distant = 0, // No collision shapes resident
	Preparation = 1, // Asynchronous shape compilation
	Protected = 2, // Committed Jolt collision active
	Retention = 3 // Brief hysteresis before shape retirement
};

struct CollisionClusterRecord {
	FeatureKey cluster_key{};
	uint16_t building_count{ 0 };
	CollisionEnvelopeState state{ CollisionEnvelopeState::Distant };
	uint32_t jolt_shape_count{ 0 };
};

class CollisionChannel {
public:
	static constexpr size_t MAX_ACTIVE_CLUSTERS = 32;

	constexpr CollisionChannel() noexcept = default;

	[[nodiscard]] constexpr bool prepare_cluster(const FeatureKey &p_key, const CompiledBuildingPackage &p_package) noexcept {
		if (m_cluster_count >= MAX_ACTIVE_CLUSTERS) {
			m_aborted_count++;
			return false;
		}

		CollisionClusterRecord record{};
		record.cluster_key = p_key;
		record.building_count = 1;
		record.state = CollisionEnvelopeState::Protected;
		record.jolt_shape_count = static_cast<uint32_t>(p_package.member_count);

		m_clusters[m_cluster_count++] = record;
		m_prepared_count++;
		m_active_count++;
		return true;
	}

	[[nodiscard]] constexpr bool retire_cluster(const FeatureKey &p_key) noexcept {
		for (size_t i = 0; i < m_cluster_count; ++i) {
			if (m_clusters[i].cluster_key == p_key) {
				m_clusters[i] = m_clusters[--m_cluster_count];
				m_retiring_count++;
				if (m_active_count > 0) m_active_count--;
				return true;
			}
		}
		return false;
	}

	[[nodiscard]] constexpr uint32_t get_prepared_count() const noexcept { return m_prepared_count; }
	[[nodiscard]] constexpr uint32_t get_active_count() const noexcept { return m_active_count; }
	[[nodiscard]] constexpr uint32_t get_retiring_count() const noexcept { return m_retiring_count; }
	[[nodiscard]] constexpr uint32_t get_aborted_count() const noexcept { return m_aborted_count; }

private:
	std::array<CollisionClusterRecord, MAX_ACTIVE_CLUSTERS> m_clusters{};
	size_t m_cluster_count{ 0 };

	uint32_t m_prepared_count{ 0 };
	uint32_t m_active_count{ 0 };
	uint32_t m_retiring_count{ 0 };
	uint32_t m_aborted_count{ 0 };
};

} // namespace Multinet

#endif // MULTINET_COLLISION_CHANNEL_H
