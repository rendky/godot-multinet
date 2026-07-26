#ifndef MULTINET_CANON_H
#define MULTINET_CANON_H

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace Multinet {

// ============================================================================
// CANON PLANE CONTRACT INTERFACES (GodotMultinet Architecture v1 - Section 12)
// Doctrine: "One canon. Many bounded local realities."
// ============================================================================

using EventID = uint64_t;
using ScopeID = uint64_t;
using PlayerID = uint64_t;

struct PayloadView {
	const uint8_t *data{ nullptr };
	size_t size{ 0 };
};

struct EvidenceView {
	const uint8_t *data{ nullptr };
	size_t size{ 0 };
};

struct PayloadRef {
	uint32_t offset{ 0 };
	uint32_t size{ 0 };
};

struct EvidenceRef {
	uint32_t offset{ 0 };
	uint32_t size{ 0 };
};

struct DurableProposalView {
	EventID proposal_id{ 0 };
	ScopeID scope_id{ 0 };
	uint16_t event_type{ 0 };
	uint16_t schema_version{ 1 };
	PlayerID actor{ 0 };
	uint32_t base_version{ 0 };
	PayloadView payload{};
	EvidenceView evidence{};
	uint64_t idempotency_key{ 0 };
};

struct DomainValidationResult {
	bool accepted{ false };
	uint16_t rejection_code{ 0 };
	uint16_t result_schema{ 1 };
	PayloadRef authoritative_result{};
	uint32_t domain_version{ 0 };
	uint64_t result_hash{ 0 };
};

struct CanonCommitRequest {
	DurableProposalView proposal{};
	DomainValidationResult domain_result{};
	uint32_t expected_scope_version{ 0 };
	uint64_t previous_event_hash{ 0 };
};

struct CanonCommitResult {
	bool committed{ false };
	EventID canonical_event_id{ 0 };
	uint32_t accepted_scope_version{ 0 };
	uint64_t canonical_sequence{ 0 };
	uint64_t authentication{ 0 };
	PayloadRef accepted_result{};
	uint16_t rejection_code{ 0 };
};

struct CanonicalEvent {
	EventID event_id{ 0 };
	ScopeID scope_id{ 0 };
	uint16_t event_type{ 0 };
	uint16_t schema_version{ 1 };
	PlayerID actor{ 0 };
	uint32_t base_version{ 0 };
	uint32_t accepted_version{ 0 };
	PayloadRef payload{};
	EvidenceRef evidence{};
	uint64_t idempotency_key{ 0 };
	uint64_t previous_event_hash{ 0 };
	uint64_t authentication{ 0 };
	uint64_t canonical_sequence{ 0 };
};

// ============================================================================
// M1 LOCAL CANON MOCK GATEWAY
// Stub implementation of Canon Plane durable ordering, idempotency & versioning.
// ============================================================================

class LocalCanonMock {
private:
	std::unordered_map<uint64_t, CanonCommitResult> idempotency_store;
	std::vector<CanonicalEvent> event_log;
	uint64_t global_sequence{ 0 };
	uint64_t last_event_hash{ 0 };
	uint32_t current_scope_version{ 0 };

public:
	LocalCanonMock() = default;

	CanonCommitResult process_commit_request(const CanonCommitRequest &p_request) noexcept {
		// Rule 12.4: Idempotency Enforcement - Repeated commit requests return recorded result
		if (p_request.proposal.idempotency_key != 0) {
			auto it = idempotency_store.find(p_request.proposal.idempotency_key);
			if (it != idempotency_store.end()) {
				return it->second;
			}
		}

		CanonCommitResult result{};

		// Domain validation check
		if (!p_request.domain_result.accepted) {
			result.committed = false;
			result.rejection_code = p_request.domain_result.rejection_code;
			return result;
		}

		// Sequence & Version Advancement
		global_sequence++;
		current_scope_version++;

		result.committed = true;
		result.canonical_event_id = p_request.proposal.proposal_id;
		result.accepted_scope_version = current_scope_version;
		result.canonical_sequence = global_sequence;
		result.accepted_result = p_request.domain_result.authoritative_result;

		// Hash Linkage (SquirrelNoise5 hash mock for event chain)
		uint64_t current_hash = last_event_hash ^ (global_sequence * 0x9E3779B97F4A7C15ULL) ^ p_request.proposal.idempotency_key;
		result.authentication = current_hash;
		last_event_hash = current_hash;

		// Log Canonical Event
		CanonicalEvent evt{};
		evt.event_id = result.canonical_event_id;
		evt.scope_id = p_request.proposal.scope_id;
		evt.event_type = p_request.proposal.event_type;
		evt.schema_version = p_request.proposal.schema_version;
		evt.actor = p_request.proposal.actor;
		evt.base_version = p_request.proposal.base_version;
		evt.accepted_version = current_scope_version;
		evt.idempotency_key = p_request.proposal.idempotency_key;
		evt.previous_event_hash = p_request.previous_event_hash;
		evt.authentication = result.authentication;
		evt.canonical_sequence = global_sequence;

		event_log.push_back(evt);

		// Record for Idempotency
		if (p_request.proposal.idempotency_key != 0) {
			idempotency_store[p_request.proposal.idempotency_key] = result;
		}

		return result;
	}

	[[nodiscard]] uint64_t get_global_sequence() const noexcept { return global_sequence; }
	[[nodiscard]] uint32_t get_scope_version() const noexcept { return current_scope_version; }
	[[nodiscard]] size_t get_log_count() const noexcept { return event_log.size(); }
	[[nodiscard]] const std::vector<CanonicalEvent> &get_event_log() const noexcept { return event_log; }
};

} // namespace Multinet

#endif // MULTINET_CANON_H
