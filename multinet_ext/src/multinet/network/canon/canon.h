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
	static constexpr size_t MAX_LOG_EVENTS = 1024;
	static constexpr size_t IDEMPOTENCY_CAPACITY = 512;

	struct IdempotencyEntry {
		uint64_t key{ 0 };
		CanonCommitResult result{};
		bool is_active{ false };
	};

	std::array<IdempotencyEntry, IDEMPOTENCY_CAPACITY> idempotency_store{};
	std::array<CanonicalEvent, MAX_LOG_EVENTS> event_log{};
	
	size_t event_head{ 0 };
	size_t event_count{ 0 };
	
	uint64_t global_sequence{ 0 };
	uint64_t last_event_hash{ 0 };
	uint32_t current_scope_version{ 0 };

	[[nodiscard]] CanonCommitResult* find_idempotency_result(uint64_t p_key) noexcept {
		size_t start_idx = p_key % IDEMPOTENCY_CAPACITY;
		for (size_t i = 0; i < IDEMPOTENCY_CAPACITY; ++i) {
			size_t idx = (start_idx + i) % IDEMPOTENCY_CAPACITY;
			if (!idempotency_store[idx].is_active) {
				return nullptr; // Not found
			}
			if (idempotency_store[idx].key == p_key) {
				return &idempotency_store[idx].result;
			}
		}
		return nullptr;
	}

	void store_idempotency(uint64_t p_key, const CanonCommitResult& p_result) noexcept {
		size_t start_idx = p_key % IDEMPOTENCY_CAPACITY;
		for (size_t i = 0; i < IDEMPOTENCY_CAPACITY; ++i) {
			size_t idx = (start_idx + i) % IDEMPOTENCY_CAPACITY;
			// Allow overwriting if it's inactive OR just overwrite the oldest/closest
			// For a true ring buffer, we'd evict, but this is a mock. We will just overwrite.
			if (!idempotency_store[idx].is_active || idempotency_store[idx].key == p_key) {
				idempotency_store[idx].key = p_key;
				idempotency_store[idx].result = p_result;
				idempotency_store[idx].is_active = true;
				return;
			}
		}
		// If full, force overwrite start_idx
		idempotency_store[start_idx].key = p_key;
		idempotency_store[start_idx].result = p_result;
		idempotency_store[start_idx].is_active = true;
	}

public:
	LocalCanonMock() = default;

	CanonCommitResult process_commit_request(const CanonCommitRequest &p_request) noexcept {
		// Rule 12.4: Idempotency Enforcement - Repeated commit requests return recorded result
		if (p_request.proposal.idempotency_key != 0) {
			if (CanonCommitResult* cached = find_idempotency_result(p_request.proposal.idempotency_key)) {
				return *cached;
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

		event_log[event_head] = evt;
		event_head = (event_head + 1) % MAX_LOG_EVENTS;
		if (event_count < MAX_LOG_EVENTS) event_count++;

		// Record for Idempotency
		if (p_request.proposal.idempotency_key != 0) {
			store_idempotency(p_request.proposal.idempotency_key, result);
		}

		return result;
	}

	[[nodiscard]] uint64_t get_global_sequence() const noexcept { return global_sequence; }
	[[nodiscard]] uint32_t get_scope_version() const noexcept { return current_scope_version; }
	[[nodiscard]] size_t get_log_count() const noexcept { return event_count; }

	// Ring buffer chronological access (idx 0 is oldest available, up to event_count-1)
	[[nodiscard]] const CanonicalEvent* get_event(size_t p_index) const noexcept {
		if (p_index >= event_count) return nullptr;
		
		size_t start_idx = 0;
		if (event_count == MAX_LOG_EVENTS) {
			start_idx = event_head; // Oldest is at head if full
		}
		
		size_t real_idx = (start_idx + p_index) % MAX_LOG_EVENTS;
		return &event_log[real_idx];
	}
};

} // namespace Multinet

#endif // MULTINET_CANON_H
