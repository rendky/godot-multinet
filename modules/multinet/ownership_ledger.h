#ifndef MULTINET_OWNERSHIP_LEDGER_H
#define MULTINET_OWNERSHIP_LEDGER_H

#include <cstddef>
#include <cstdint>

namespace Multinet {

struct SubsystemResourceReport {
	const char *subsystem_name{ "multinet" };
	size_t arena_capacity_bytes{ 0 };
	size_t arena_used_bytes{ 0 };
	size_t pool_capacity_items{ 0 };
	size_t pool_active_items{ 0 };
	size_t job_queue_count{ 0 };
	uint64_t snapshot_version{ 0 };
	bool overflow_flagged{ false };
};

class OwnershipLedger {
public:
	static SubsystemResourceReport generate_report(
			size_t p_arena_cap,
			size_t p_arena_used,
			size_t p_pool_cap,
			size_t p_pool_active,
			size_t p_job_count,
			uint64_t p_snapshot_ver,
			bool p_overflow) noexcept {
		SubsystemResourceReport report{};
		report.subsystem_name = "multinet";
		report.arena_capacity_bytes = p_arena_cap;
		report.arena_used_bytes = p_arena_used;
		report.pool_capacity_items = p_pool_cap;
		report.pool_active_items = p_pool_active;
		report.job_queue_count = p_job_count;
		report.snapshot_version = p_snapshot_ver;
		report.overflow_flagged = p_overflow;
		return report;
	}
};

} // namespace Multinet

#endif // MULTINET_OWNERSHIP_LEDGER_H
