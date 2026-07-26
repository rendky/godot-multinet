#ifndef MULTINET_RESOURCE_REGISTRY_H
#define MULTINET_RESOURCE_REGISTRY_H

#include <cstdint>
#include <unordered_map>

namespace Multinet {

enum class ResourceRole : uint8_t {
	NETWORK_SCHEMA = 0,
	REPLICATION_PROFILE = 1,
	INTEREST_PROFILE = 2,
	ROUTING_PROFILE = 3,
	CANON_EVENT_SCHEMA = 4,
	TRAVEL_PROFILE = 5,
	SECURITY_PROFILE = 6,
	NETWORK_FIXTURE = 7,
	COUNT = 8
};

struct ResourceRoleEntry {
	uint32_t role_id{ 0 };
	ResourceRole role{ ResourceRole::NETWORK_SCHEMA };
	uint16_t schema_version{ 1 };
	uint32_t payload_max_bytes{ 65536 };
};

class ResourceRoleRegistry {
private:
	std::unordered_map<uint32_t, ResourceRoleEntry> registry;

public:
	ResourceRoleRegistry() = default;

	bool register_role(uint32_t p_role_id, ResourceRole p_role, uint16_t p_version, uint32_t p_max_bytes) noexcept {
		if (p_role_id == 0) return false;

		ResourceRoleEntry entry;
		entry.role_id = p_role_id;
		entry.role = p_role;
		entry.schema_version = p_version;
		entry.payload_max_bytes = p_max_bytes;

		registry[p_role_id] = entry;
		return true;
	}

	[[nodiscard]] bool find_role(uint32_t p_role_id, ResourceRoleEntry &r_entry) const noexcept {
		auto it = registry.find(p_role_id);
		if (it == registry.end()) return false;

		r_entry = it->second;
		return true;
	}

	[[nodiscard]] size_t get_registered_count() const noexcept { return registry.size(); }
};

} // namespace Multinet

#endif // MULTINET_RESOURCE_REGISTRY_H
