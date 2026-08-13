#include "concrete_terrain_render_source.h"
#include "multinet/world/terrain/canonical_terrain_signal.h"
#include "multinet/world/terrain/heightfield_generator.h"
#include "multinet/rendering/terrain/block_clipmap/terrain_sample_patch.h"

#include <algorithm>
#include <thread>
#include <atomic>
#include <mutex>
#include <array>
#include <optional>
#include <cmath>
#include <iostream>

namespace Multinet {

ConcreteTerrainRenderSource::ConcreteTerrainRenderSource(
	const TerrainRecipe& p_recipe,
	const WorldScaleManifest& p_manifest,
	BoundedBackgroundJobExecutor& p_executor,
	TerrainPageGenerationMode p_mode
)
	: recipe(p_recipe), manifest(p_manifest), executor(p_executor), generation_mode(p_mode),
	  slots(std::make_unique<std::array<RecordSlot, CPU_PAGE_POOL_SIZE>>()),
	  page_data(std::make_unique<std::array<ConcreteTerrainPageData, CPU_PAGE_POOL_SIZE>>()),
	  pending_queue(std::make_unique<std::array<PendingRequest, PENDING_REQUEST_CAPACITY>>()),
	  current_wanted_set(std::make_unique<std::array<WantedKeyEntry, WANTED_SET_CAPACITY>>()),
	  last_wanted_keys_(std::make_unique<std::array<WantedKeyEntry, WANTED_SET_CAPACITY>>()) {
	
	current_snapshot.recipe_identity = recipe.identity;
	current_snapshot.world_manifest_hash = manifest.manifest_hash;
	current_snapshot.topology_version = manifest.topology_version;
	current_snapshot.projection_version = manifest.projection_version;
	current_snapshot.terrain_version = 1;
	current_snapshot.source_version = 1;

	current_snapshot.page_contract_version = TERRAIN_PAGE_CONTRACT_VERSION_1;
	current_snapshot.payload_kind = TerrainPagePayloadKind::AbsoluteHeightDebugV1;

	current_snapshot.fallback_bounds.minimum_height = recipe.legacy_signals.min_elevation_m;
	current_snapshot.fallback_bounds.maximum_height = recipe.legacy_signals.max_elevation_m;
	current_snapshot.fallback_bounds.residual_bound = 1.0f;
	current_snapshot.fallback_bounds.morph_allowance = 0.5f;

	current_delta_snapshot.field = std::make_shared<NullTerrainCommittedDeltaField>();
}

ConcreteTerrainRenderSource::ConcreteTerrainRenderSource(
	const TerrainRecipe& p_recipe,
	const WorldDomainManifest& p_domain,
	BoundedBackgroundJobExecutor& p_executor,
	TerrainPageGenerationMode p_mode
)
	: recipe(p_recipe), manifest(make_compatibility_scale_manifest(p_domain)), domain(p_domain), executor(p_executor), generation_mode(p_mode),
	  slots(std::make_unique<std::array<RecordSlot, CPU_PAGE_POOL_SIZE>>()),
	  page_data(std::make_unique<std::array<ConcreteTerrainPageData, CPU_PAGE_POOL_SIZE>>()),
	  pending_queue(std::make_unique<std::array<PendingRequest, PENDING_REQUEST_CAPACITY>>()),
	  current_wanted_set(std::make_unique<std::array<WantedKeyEntry, WANTED_SET_CAPACITY>>()),
	  last_wanted_keys_(std::make_unique<std::array<WantedKeyEntry, WANTED_SET_CAPACITY>>()) {
	current_snapshot.recipe_identity = recipe.identity;
	current_snapshot.world_manifest_hash = domain.domain_manifest_hash;
	current_snapshot.topology_version = domain.topology_version;
	current_snapshot.projection_version = domain.projection_version;
	current_snapshot.terrain_version = 1;
	current_snapshot.source_version = 1;
	current_snapshot.page_contract_version = TERRAIN_PAGE_CONTRACT_VERSION_1;
	current_snapshot.payload_kind = TerrainPagePayloadKind::AbsoluteHeightDebugV1;
	current_snapshot.fallback_bounds.minimum_height = recipe.legacy_signals.min_elevation_m;
	current_snapshot.fallback_bounds.maximum_height = recipe.legacy_signals.max_elevation_m;
	current_snapshot.fallback_bounds.residual_bound = 1.0f;
	current_snapshot.fallback_bounds.morph_allowance = 0.5f;
	current_delta_snapshot.field = std::make_shared<NullTerrainCommittedDeltaField>();
}

void ConcreteTerrainRenderSource::set_payload_kind(TerrainPagePayloadKind kind) noexcept {
	std::lock_guard<std::mutex> lock(access_mutex);
	if (current_snapshot.payload_kind == kind) return;

	current_snapshot.payload_kind = kind;
	current_snapshot.source_version++;

	for (size_t i = 0; i < CPU_PAGE_POOL_SIZE; ++i) {
		if ((*slots)[i].in_use && (*slots)[i].record.state == TerrainSourceState::Pending) {
			(*slots)[i].cancellation_generation.fetch_add(1, std::memory_order_release);
			(*slots)[i].in_use = false;
			(*slots)[i].record.state = TerrainSourceState::Missing;
			(*slots)[i].generation = ++next_generation;
			cancelled_retryable_count_.fetch_add(1, std::memory_order_relaxed);
		}
	}

	pending_queue_size = 0;
}

void ConcreteTerrainRenderSource::set_committed_delta_snapshot(const TerrainCommittedDeltaSnapshot& snapshot) noexcept {
	std::lock_guard<std::mutex> lock(access_mutex);
	TerrainCommittedDeltaSnapshot candidate = snapshot;

	bool valid = true;
	if (candidate.contract_version != TERRAIN_PAGE_CONTRACT_VERSION_1) valid = false;
	if (candidate.publication_version == 0) valid = false;
	if (!std::isfinite(candidate.minimum_delta_m) ||
	    !std::isfinite(candidate.maximum_delta_m) ||
	    !std::isfinite(candidate.maximum_abs_gradient)) valid = false;
	if (candidate.minimum_delta_m > candidate.maximum_delta_m) valid = false;
	if (candidate.maximum_abs_gradient < 0.0f) valid = false;

	if (valid) {
		if (candidate.field) {
			TerrainDeltaEnvelope env = candidate.field->get_conservative_envelope();
			if (!std::isfinite(env.minimum_delta_m) ||
			    !std::isfinite(env.maximum_delta_m) ||
			    !std::isfinite(env.maximum_abs_gradient) ||
			    env.minimum_delta_m > env.maximum_delta_m ||
			    env.maximum_abs_gradient < 0.0f) {
				valid = false;
			} else {
				candidate.minimum_delta_m = std::min({ candidate.minimum_delta_m, env.minimum_delta_m, 0.0f });
				candidate.maximum_delta_m = std::max({ candidate.maximum_delta_m, env.maximum_delta_m, 0.0f });
				candidate.maximum_abs_gradient = std::max(candidate.maximum_abs_gradient, env.maximum_abs_gradient);
			}
		} else {
			candidate.minimum_delta_m = 0.0f;
			candidate.maximum_delta_m = 0.0f;
			candidate.maximum_abs_gradient = 0.0f;
		}
	}

	if (!valid) {
		rejected_delta_publication_count_.fetch_add(1, std::memory_order_relaxed);
		return;
	}

	current_delta_snapshot = candidate;
	current_snapshot.committed_delta_version = candidate.publication_version;
}

TerrainCommittedDeltaSnapshot ConcreteTerrainRenderSource::get_committed_delta_snapshot() const noexcept {
	std::lock_guard<std::mutex> lock(access_mutex);
	return current_delta_snapshot;
}

ConcreteTerrainRenderSource::~ConcreteTerrainRenderSource() {
	shutdown();
}

void ConcreteTerrainRenderSource::decrement_in_flight() noexcept {
	// Must hold the mutex while modifying the count to prevent the shutdown thread
	// from returning and destroying the object before we finish notifying.
	std::lock_guard<std::mutex> lock(shutdown_mutex_);
	if (in_flight_count_.fetch_sub(1, std::memory_order_acq_rel) == 1) {
		shutdown_cv_.notify_all();
	}
}

void ConcreteTerrainRenderSource::shutdown() noexcept {
	shutting_down_.store(true, std::memory_order_release);

	// Cancel all tokens in the pending queue (already-submitted jobs check shutting_down_).
	{
		std::lock_guard<std::mutex> lock(access_mutex);
		pending_queue_size = 0;
	}

	{
		std::unique_lock<std::mutex> lock(shutdown_mutex_);
		shutdown_cv_.wait(lock, [this] {
			return in_flight_count_.load(std::memory_order_acquire) == 0;
		});
	}
}

TerrainRenderSourceSnapshot ConcreteTerrainRenderSource::get_snapshot() const noexcept {
	std::lock_guard<std::mutex> lock(access_mutex);
	return current_snapshot;
}

TerrainRenderPublicationView ConcreteTerrainRenderSource::get_publication_view() const noexcept {
	std::lock_guard<std::mutex> lock(access_mutex);
	return TerrainRenderPublicationView{ current_snapshot, current_delta_snapshot };
}

std::optional<uint64_t> ConcreteTerrainRenderSource::find_existing_slot(const ConcreteTerrainSourceKey& key) const noexcept {
	for (size_t i = 0; i < CPU_PAGE_POOL_SIZE; ++i) {
		if ((*slots)[i].in_use && (*slots)[i].full_key == key) {
			return i;
		}
	}
	return std::nullopt;
}

std::optional<uint64_t> ConcreteTerrainRenderSource::allocate_slot() noexcept {
	// First try a truly free slot.
	for (size_t i = 0; i < CPU_PAGE_POOL_SIZE; ++i) {
		if (!(*slots)[i].in_use) {
			(*slots)[i].in_use = true;
			return i;
		}
	}

	auto is_key_wanted = [this](const multinet::rendering::TerrainRenderBlockKey& key) -> bool {
		for (size_t k = 0; k < current_wanted_set_size_; ++k) {
			if ((*current_wanted_set)[k].identity.block_key == key) return true;
		}
		for (size_t k = 0; k < last_wanted_keys_count_; ++k) {
			if ((*last_wanted_keys_)[k].identity.block_key == key) return true;
		}
		return false;
	};

	// 1. Evict an unwanted Ready, ReadyEmpty, or Invalid slot first.
	for (size_t i = 0; i < CPU_PAGE_POOL_SIZE; ++i) {
		if ((*slots)[i].in_use) {
			auto state = (*slots)[i].record.state;
			if ((state == TerrainSourceState::Ready || state == TerrainSourceState::ReadyEmpty || state == TerrainSourceState::Invalid) &&
			    !is_key_wanted((*slots)[i].record.canonical_key)) {
				(*slots)[i].generation = ++next_generation;
				return i;
			}
		}
	}

	// 2. Fallback: Evict any Ready, ReadyEmpty, or Invalid slot.
	for (size_t i = 0; i < CPU_PAGE_POOL_SIZE; ++i) {
		if ((*slots)[i].in_use) {
			auto state = (*slots)[i].record.state;
			if (state == TerrainSourceState::Ready || state == TerrainSourceState::ReadyEmpty || state == TerrainSourceState::Invalid) {
				(*slots)[i].generation = ++next_generation;
				return i;
			}
		}
	}

	return std::nullopt;
}

TerrainSourceRecord ConcreteTerrainRenderSource::get_or_request_record(const multinet::rendering::TerrainRenderBlockKey& key) noexcept {
	TerrainRequestMetadata meta;
	meta.request_class = TerrainRequestClass::CoarseCoverage;
	meta.distance_sq_m = 0;
	TerrainPageRequestContext ctx = make_page_request_context(key, profile, get_publication_view(), manifest);
	return request_record(ctx, meta).record;
}
bool ConcreteTerrainRenderSource::try_query_record(
	const TerrainPageRequestIdentity& identity,
	TerrainSourceRecord& out_record
) const noexcept {
	std::lock_guard<std::mutex> lock(access_mutex);

	ConcreteTerrainSourceKey source_key;
	source_key.block_key = identity.block_key;
	source_key.sample_patch = identity.sample_patch;
	source_key.world_manifest_hash = identity.world_manifest_hash;
	source_key.recipe_hash = identity.recipe_hash;
	source_key.page_contract_version = identity.page_contract_version;
	source_key.payload_kind = identity.payload_kind;
	source_key.terrain_version = identity.terrain_version;
	source_key.source_version = identity.source_version;
	source_key.committed_delta_version = identity.publication_version;
	source_key.block_delta_content_version = identity.block_content_version;

	std::optional<uint64_t> existing = find_existing_slot(source_key);
	if (existing.has_value()) {
		uint64_t handle = existing.value();
		out_record = (*slots)[handle].record;
		return true;
	}

	out_record = TerrainSourceRecord{};
	out_record.canonical_key = identity.block_key;
	out_record.state = TerrainSourceState::Missing;
	return false;
}

TerrainSourceRequestResult ConcreteTerrainRenderSource::request_record(
	const TerrainPageRequestContext& context,
	const TerrainRequestMetadata& metadata
) noexcept {
	std::lock_guard<std::mutex> lock(access_mutex);
	request_record_call_count_++;

	const auto& identity = context.identity;

	ConcreteTerrainSourceKey source_key;
	source_key.block_key = identity.block_key;
	source_key.sample_patch = identity.sample_patch;
	source_key.world_manifest_hash = identity.world_manifest_hash;
	source_key.recipe_hash = identity.recipe_hash;
	source_key.page_contract_version = identity.page_contract_version;
	source_key.payload_kind = identity.payload_kind;
	source_key.terrain_version = identity.terrain_version;
	source_key.source_version = identity.source_version;
	source_key.committed_delta_version = identity.publication_version;
	source_key.block_delta_content_version = identity.block_content_version;

	TerrainSourceRequestResult res;

	// Check if block is guaranteed zero additive displacement -> ReadyEmpty
	if (identity.payload_kind == TerrainPagePayloadKind::AdditiveHeightDeltaV1) {
		double required_apron_m = profile.get_lod_spacing(identity.block_key.lod);
		const bool may_have_delta = context.committed_delta.field &&
			(identity.sample_patch.is_valid() || (domain.is_valid()
				? context.committed_delta.field->block_may_have_nonzero_delta(identity.block_key, domain, profile, required_apron_m)
				: context.committed_delta.field->block_may_have_nonzero_delta(identity.block_key, manifest, profile, required_apron_m)));
		if (!may_have_delta)
		{
			std::optional<uint64_t> existing = find_existing_slot(source_key);
			if (existing.has_value()) {
				uint64_t handle = existing.value();
				res.record = (*slots)[handle].record;
				res.disposition = TerrainSourceRequestDisposition::ExistingReadyEmpty;
				return res;
			}

			std::optional<uint64_t> new_slot = allocate_slot();
			if (!new_slot.has_value()) {
				res.record.canonical_key = identity.block_key;
				res.record.state = TerrainSourceState::Missing;
				res.disposition = TerrainSourceRequestDisposition::RejectedPoolFull;
				return res;
			}

			uint64_t handle = new_slot.value();
			uint32_t gen = (*slots)[handle].generation;

			(*slots)[handle].full_key = source_key;
			(*slots)[handle].record.canonical_key = identity.block_key;
			(*slots)[handle].record.cpu_page_handle = INVALID_CPU_PAGE_HANDLE;
			(*slots)[handle].record.cpu_page_generation = gen;
			(*slots)[handle].record.state = TerrainSourceState::ReadyEmpty;
			(*slots)[handle].record.page_contract_version = identity.page_contract_version;
			(*slots)[handle].record.payload_kind = identity.payload_kind;
			(*slots)[handle].record.terrain_version = identity.terrain_version;
			(*slots)[handle].record.source_version = identity.source_version;
			(*slots)[handle].record.committed_delta_version = identity.publication_version;
			(*slots)[handle].record.block_delta_content_version = identity.block_content_version;
			(*slots)[handle].record.min_height = 0.0f;
			(*slots)[handle].record.max_height = 0.0f;
			(*slots)[handle].record.residual_bound = 0.0f;
			(*slots)[handle].record.gradient_bound = 0.0f;

			(*page_data)[handle].source_key = source_key;
			(*page_data)[handle].is_valid = false;
			(*page_data)[handle].generation = gen;

			res.record = (*slots)[handle].record;
			res.disposition = TerrainSourceRequestDisposition::CreatedReadyEmpty;
			return res;
		}
	}

	std::optional<uint64_t> existing = find_existing_slot(source_key);
	if (existing.has_value()) {
		uint64_t handle = existing.value();
		res.record = (*slots)[handle].record;
		if (res.record.state == TerrainSourceState::Pending) {
			res.disposition = TerrainSourceRequestDisposition::ExistingPending;
		} else if (res.record.state == TerrainSourceState::Ready) {
			res.disposition = TerrainSourceRequestDisposition::ExistingReady;
		} else if (res.record.state == TerrainSourceState::ReadyEmpty) {
			res.disposition = TerrainSourceRequestDisposition::ExistingReadyEmpty;
		} else {
			res.disposition = TerrainSourceRequestDisposition::ExistingInvalid;
		}
		return res;
	}

	if (pending_queue_size >= PENDING_REQUEST_CAPACITY) {
		res.record.canonical_key = identity.block_key;
		res.record.state = TerrainSourceState::Missing;
		res.disposition = TerrainSourceRequestDisposition::RejectedQueueFull;
		return res;
	}

	std::optional<uint64_t> new_slot = allocate_slot();
	if (!new_slot.has_value()) {
		res.record.canonical_key = identity.block_key;
		res.record.state = TerrainSourceState::Missing;
		res.disposition = TerrainSourceRequestDisposition::RejectedPoolFull;
		return res;
	}

	uint64_t handle = new_slot.value();
	uint32_t gen = (*slots)[handle].generation;
	uint64_t cancel_gen = (*slots)[handle].cancellation_generation.load(std::memory_order_relaxed);

	(*slots)[handle].full_key = source_key;
	(*slots)[handle].record.canonical_key = identity.block_key;
	(*slots)[handle].record.cpu_page_handle = handle;
	(*slots)[handle].record.cpu_page_generation = gen;
	(*slots)[handle].record.state = TerrainSourceState::Pending;
	(*slots)[handle].record.page_contract_version = identity.page_contract_version;
	(*slots)[handle].record.payload_kind = identity.payload_kind;
	(*slots)[handle].record.terrain_version = identity.terrain_version;
	(*slots)[handle].record.source_version = identity.source_version;
	(*slots)[handle].record.committed_delta_version = identity.publication_version;
	(*slots)[handle].record.block_delta_content_version = identity.block_content_version;

	(*page_data)[handle].source_key = source_key;
	(*page_data)[handle].is_valid = false;
	(*page_data)[handle].generation = gen;

	PendingRequest pending_req;
	pending_req.context = context;
	pending_req.slot_handle = handle;
	pending_req.slot_generation = gen;
	pending_req.cancellation_generation = cancel_gen;
	pending_req.request_class = metadata.request_class;
	pending_req.distance_sq_m = metadata.distance_sq_m;
	pending_req.wanted_set_epoch = metadata.wanted_set_epoch;

	(*pending_queue)[pending_queue_size++] = pending_req;

	res.record = (*slots)[handle].record;
	res.disposition = TerrainSourceRequestDisposition::CreatedPending;
	return res;
}

void ConcreteTerrainRenderSource::release_unpublished_slot(uint64_t handle, uint32_t expected_generation) noexcept {
	if (handle >= CPU_PAGE_POOL_SIZE) return;
	auto& slot = (*slots)[handle];
	if (slot.generation == expected_generation && slot.in_use) {
		slot.in_use = false;
		slot.record.state = TerrainSourceState::Missing;
		slot.record.cpu_page_handle = INVALID_CPU_PAGE_HANDLE;
		slot.generation = ++next_generation;
		(*page_data)[handle].is_valid = false;
	}
}

void ConcreteTerrainRenderSource::begin_wanted_set(uint64_t epoch) noexcept {
	std::lock_guard<std::mutex> lock(access_mutex);
	wanted_set_epoch_ = epoch;
	current_wanted_set_size_ = 0;
	wanted_set_overflow_ = false;
}

bool ConcreteTerrainRenderSource::mark_wanted(
	const TerrainPageRequestIdentity& identity,
	TerrainRequestClass request_class,
	int64_t distance_sq_m,
	uint64_t epoch
) noexcept {
	std::lock_guard<std::mutex> lock(access_mutex);
	if (epoch != wanted_set_epoch_) return false;
	if (current_wanted_set_size_ >= WANTED_SET_CAPACITY) {
		wanted_set_overflow_ = true;
		return false;
	}
	(*current_wanted_set)[current_wanted_set_size_++] = WantedKeyEntry{ identity, request_class, distance_sq_m };
	return true;
}

void ConcreteTerrainRenderSource::cancel_all_page_work_and_advance_epoch() noexcept {
	std::lock_guard<std::mutex> lock(access_mutex);

	next_generation++;
	current_snapshot.source_version++;

	pending_queue_size = 0;
	current_wanted_set_size_ = 0;
	last_wanted_keys_count_ = 0;
	wanted_set_epoch_++;

	for (size_t i = 0; i < CPU_PAGE_POOL_SIZE; ++i) {
		if ((*slots)[i].in_use) {
			(*slots)[i].cancellation_generation.fetch_add(1, std::memory_order_release);
			auto st = (*slots)[i].record.state;
			if (st == TerrainSourceState::Pending) {
				(*slots)[i].record.state = TerrainSourceState::Missing;
				(*slots)[i].in_use = false;
				(*slots)[i].generation = ++next_generation;
				cancelled_retryable_count_.fetch_add(1, std::memory_order_relaxed);
			}
		}
	}
}

void ConcreteTerrainRenderSource::end_wanted_set() noexcept {
	std::lock_guard<std::mutex> lock(access_mutex);

	for (size_t w = 0; w < current_wanted_set_size_; ++w) {
		(*last_wanted_keys_)[w] = (*current_wanted_set)[w];
	}
	last_wanted_keys_count_ = current_wanted_set_size_;

	// Full-Identity wanted-set matching (Section H)
	for (size_t i = 0; i < CPU_PAGE_POOL_SIZE; ++i) {
		if ((*slots)[i].in_use && (*slots)[i].record.state == TerrainSourceState::Pending) {
			bool is_wanted = false;
			const auto& slot_rec = (*slots)[i].record;
			for (size_t w = 0; w < current_wanted_set_size_; ++w) {
				const auto& wanted_id = (*current_wanted_set)[w].identity;
				if (wanted_id.block_key == slot_rec.canonical_key &&
				    wanted_id.page_contract_version == slot_rec.page_contract_version &&
				    wanted_id.payload_kind == slot_rec.payload_kind &&
				    wanted_id.terrain_version == slot_rec.terrain_version &&
				    wanted_id.source_version == slot_rec.source_version &&
				    wanted_id.block_content_version == slot_rec.block_delta_content_version) {
					is_wanted = true;
					break;
				}
			}
			if (!is_wanted) {
				(*slots)[i].cancellation_generation.fetch_add(1, std::memory_order_release);
				(*slots)[i].in_use = false;
				(*slots)[i].record.state = TerrainSourceState::Missing;
				(*slots)[i].generation = ++next_generation;
				cancelled_retryable_count_.fetch_add(1, std::memory_order_relaxed);
			}
		}
	}

	// Compact pending_queue immediately
	size_t write_idx = 0;
	for (size_t read_idx = 0; read_idx < pending_queue_size; ++read_idx) {
		uint64_t h = (*pending_queue)[read_idx].slot_handle;
		if (h < CPU_PAGE_POOL_SIZE && (*slots)[h].in_use &&
		    (*slots)[h].generation == (*pending_queue)[read_idx].slot_generation &&
		    (*slots)[h].cancellation_generation.load(std::memory_order_acquire) == (*pending_queue)[read_idx].cancellation_generation) {
			(*pending_queue)[write_idx++] = (*pending_queue)[read_idx];
		}
	}
	pending_queue_size = write_idx;
}

void ConcreteTerrainRenderSource::commit_pending_requests(const multinet::rendering::TerrainRenderBlockKey& camera_key) noexcept {
	commit_pending_call_count_.fetch_add(1, std::memory_order_relaxed);
	struct RankedRequest {
		PendingRequest req;
		uint8_t rank;
	};

	static thread_local std::array<PendingRequest, PENDING_REQUEST_CAPACITY> tls_to_submit;
	static thread_local std::array<PendingRequest, PENDING_REQUEST_CAPACITY> tls_unsubmitted;
	static thread_local std::array<RankedRequest, PENDING_REQUEST_CAPACITY> tls_ranked;

	size_t count = 0;

	{
		std::lock_guard<std::mutex> lock(access_mutex);
		if (pending_queue_size == 0) return;

		for (size_t i = 0; i < pending_queue_size; ++i) {
			const auto& r = (*pending_queue)[i];
			tls_ranked[i] = { r, admission_rank(r.request_class) };
		}
		size_t ranked_count = pending_queue_size;
		pending_queue_size = 0;

		std::sort(tls_ranked.begin(), tls_ranked.begin() + ranked_count, [](const RankedRequest& a, const RankedRequest& b) {
			if (a.rank != b.rank) return a.rank < b.rank;
			if (a.req.context.identity.block_key.lod != b.req.context.identity.block_key.lod) {
				return a.req.context.identity.block_key.lod > b.req.context.identity.block_key.lod;
			}
			return a.req.distance_sq_m < b.req.distance_sq_m;
		});

		for (size_t i = 0; i < ranked_count; ++i) {
			tls_to_submit[i] = tls_ranked[i].req;
		}
		count = ranked_count;
	}

	size_t unsubmitted_count = 0;

	for (size_t i = 0; i < count; ++i) {
		if (!submit_generation_job(tls_to_submit[i])) {
			tls_unsubmitted[unsubmitted_count++] = tls_to_submit[i];
		}
	}

	if (unsubmitted_count > 0) {
		std::lock_guard<std::mutex> lock(access_mutex);
		for (size_t i = 0; i < unsubmitted_count; ++i) {
			if (pending_queue_size < PENDING_REQUEST_CAPACITY) {
				(*pending_queue)[pending_queue_size++] = tls_unsubmitted[i];
			}
		}
	}
}

bool ConcreteTerrainRenderSource::submit_generation_job(const PendingRequest& req) noexcept {
	PendingRequest captured_req = req;
	TerrainRecipe captured_recipe = recipe;
	WorldScaleManifest captured_manifest = manifest;

	in_flight_count_.fetch_add(1, std::memory_order_acquire);

	// Priority assignment according to Requirement 4:
	// TerminalBootstrap & CoarseCoverage -> HIGH
	// ImmediateVisible & AtomicSibling -> NORMAL
	// Prefetch -> LOW
	JobPriority prio = JobPriority::LOW;
	if (captured_req.request_class == TerrainRequestClass::TerminalBootstrap ||
	    captured_req.request_class == TerrainRequestClass::CoarseCoverage) {
		prio = JobPriority::HIGH;
	} else if (captured_req.request_class == TerrainRequestClass::ImmediateVisible ||
	           captured_req.request_class == TerrainRequestClass::AtomicSibling) {
		prio = JobPriority::NORMAL;
	} else {
		prio = JobPriority::LOW;
	}

	bool submitted = executor.submit(
		prio,
		[this, captured_req, captured_recipe, captured_manifest, captured_domain = domain]() mutable {
			auto check_cancel = [&]() -> bool {
				if (shutting_down_.load(std::memory_order_acquire)) return true;
				std::lock_guard<std::mutex> lock(access_mutex);
				uint64_t h = captured_req.slot_handle;
				if (h >= CPU_PAGE_POOL_SIZE || !(*slots)[h].in_use) return true;
				if ((*slots)[h].generation != captured_req.slot_generation) return true;
				if ((*slots)[h].cancellation_generation.load(std::memory_order_acquire) != captured_req.cancellation_generation) return true;
				if (captured_req.context.identity.world_manifest_hash != current_snapshot.world_manifest_hash ||
				    captured_req.context.identity.recipe_hash != current_snapshot.recipe_identity.recipe_hash) return true;
				return false;
			};

			if (check_cancel()) {
				std::lock_guard<std::mutex> lock(access_mutex);
				uint64_t h = captured_req.slot_handle;
				if (h < CPU_PAGE_POOL_SIZE && (*slots)[h].in_use && (*slots)[h].generation == captured_req.slot_generation) {
					(*slots)[h].in_use = false;
					(*slots)[h].record.state = TerrainSourceState::Missing;
					(*slots)[h].generation = ++next_generation;
					(*slots)[h].cancellation_generation.fetch_add(1, std::memory_order_release);
					cancelled_retryable_count_.fetch_add(1, std::memory_order_relaxed);
				}
				decrement_in_flight();
				return;
			}

			// Queue age tracking for diagnostics
			auto start_time = std::chrono::steady_clock::now();
			float queue_age_ms = std::chrono::duration<float, std::milli>(start_time - captured_req.enqueue_time).count();
			if (captured_req.request_class == TerrainRequestClass::TerminalBootstrap) {
				float prev = queue_age_terminal_ms_.load(std::memory_order_relaxed);
				queue_age_terminal_ms_.store(std::max(prev, queue_age_ms), std::memory_order_relaxed);
			} else if (captured_req.request_class == TerrainRequestClass::CoarseCoverage) {
				float prev = queue_age_coverage_ms_.load(std::memory_order_relaxed);
				queue_age_coverage_ms_.store(std::max(prev, queue_age_ms), std::memory_order_relaxed);
			}

			TerrainHeightPage new_page;
			float min_h = 1e9f;
			float max_h = -1e9f;
			bool all_valid = true;

			uint8_t lod = captured_req.context.identity.block_key.lod;
			const auto& prof = profile;
			double block_size = prof.get_lod_block_size(lod);
			double spacing = prof.get_lod_spacing(lod);
			int32_t bx = captured_req.context.identity.block_key.block_u;
			int32_t bv = captured_req.context.identity.block_key.block_v;

			struct WorkerEvaluatorCache {
				uint64_t recipe_hash{ 0 };
				uint64_t manifest_hash{ 0 };
				uint32_t terrain_version{ 0 };
				TerrainFieldEvaluator evaluator;
				bool valid{ false };
			};
			static thread_local WorkerEvaluatorCache tls_evaluator_cache;

			if (captured_req.context.identity.payload_kind != TerrainPagePayloadKind::AdditiveHeightDeltaV1 &&
				(!tls_evaluator_cache.valid ||
			    tls_evaluator_cache.recipe_hash != captured_recipe.identity.recipe_hash ||
			    tls_evaluator_cache.manifest_hash != captured_manifest.manifest_hash ||
			    tls_evaluator_cache.terrain_version != captured_req.context.identity.terrain_version))
			{
				try {
					if (captured_domain.is_finite()) {
						tls_evaluator_cache.evaluator = TerrainFieldEvaluator(
							FiniteCanonicalTerrainSignalV1(captured_recipe, captured_domain),
							captured_domain,
							captured_req.context.identity.terrain_version
						);
					} else {
						tls_evaluator_cache.evaluator = TerrainFieldEvaluator(
							CanonicalTerrainSignalV1(captured_recipe, captured_manifest),
							captured_manifest,
							captured_req.context.identity.terrain_version
						);
					}
					tls_evaluator_cache.recipe_hash = captured_recipe.identity.recipe_hash;
					tls_evaluator_cache.manifest_hash = captured_manifest.manifest_hash;
					tls_evaluator_cache.terrain_version = captured_req.context.identity.terrain_version;
					tls_evaluator_cache.valid = true;
				} catch (...) {
					all_valid = false;
				}
			}

			const TerrainFieldEvaluator& evaluator = tls_evaluator_cache.evaluator;

			if (all_valid) {
				for (int j = 0; j < 19 && all_valid; ++j) {
					if (j % 4 == 0 && check_cancel()) {
						all_valid = false;
						break;
					}
					for (int i = 0; i < 19 && all_valid; ++i) {
						SurfacePosition64 pos;
						double u_m = 0.0;
						double v_m = 0.0;
						if (captured_req.context.identity.sample_patch.is_valid()) {
							if (!captured_domain.is_valid() || !multinet::rendering::try_sample_patch_position(
								captured_req.context.identity.sample_patch,
								(i - 9) * spacing,
								(j - 9) * spacing,
								captured_domain,
								pos)) {
								all_valid = false;
								break;
							}
							u_m = pos.u_m;
							v_m = pos.v_m;
						} else {
							u_m = bx * block_size + (i - 1) * spacing;
							v_m = bv * block_size + (j - 1) * spacing;
							pos.face = captured_req.context.identity.block_key.face;
							pos.u_m = u_m;
							pos.v_m = v_m;
							pos.altitude_m = 0.0;
							pos.topology_version = captured_manifest.topology_version;
							pos.projection_version = captured_manifest.projection_version;
						}

						float h = 0.0f;
						if (captured_req.context.identity.payload_kind == TerrainPagePayloadKind::AdditiveHeightDeltaV1) {
							if (captured_req.context.committed_delta.field) {
								h = captured_req.context.committed_delta.field->sample_delta(pos);
							}
						} else {
							TerrainHeightEvaluation eval = evaluator.evaluate(pos);
							if (!eval.valid) {
								all_valid = false;
								std::cerr << "[TERRAIN-SOURCE] eval.valid=false face=" << (int)captured_req.context.identity.block_key.face
								          << " u=" << u_m << " v=" << v_m << std::endl;
								break;
							}
							h = static_cast<float>(eval.height);
						}
						new_page.samples_m[j * 19 + i] = h;

						// Track min/max over actual vertices only (not apron).
						if (i >= 1 && i <= 17 && j >= 1 && j <= 17) {
							min_h = std::min(min_h, h);
							max_h = std::max(max_h, h);
						}
					}
				}

				// BOUNDED DIAGNOSTIC: Print for LOD0 page at (0,0) only once
				static bool first_page_printed = false;
				if (!first_page_printed && lod == 0 && bx == 0 && bv == 0) {
					first_page_printed = true;
					std::cout << "[Step 12 Evidence] Generated Page LOD0(0,0): min_height=" << min_h << " max_height=" << max_h << std::endl;
					std::cout << "[Step 12 Evidence] Sample(0,0)=" << new_page.heights[0] << " Sample(9,9)=" << new_page.heights[9*19+9] << std::endl;
				}
			}

			if (min_h > max_h) {
				min_h = 0.0f;
				max_h = 0.0f;
			}

			// Publish — reacquire lock and validate generation matches
			{
				std::lock_guard<std::mutex> lock(access_mutex);

				if (shutting_down_.load(std::memory_order_acquire)) {
					decrement_in_flight();
					return;
				}

				uint64_t h = captured_req.slot_handle;
				ConcreteTerrainSourceKey req_key;
				req_key.block_key = captured_req.context.identity.block_key;
				req_key.sample_patch = captured_req.context.identity.sample_patch;
				req_key.world_manifest_hash = captured_req.context.identity.world_manifest_hash;
				req_key.recipe_hash = captured_req.context.identity.recipe_hash;
				req_key.page_contract_version = captured_req.context.identity.page_contract_version;
				req_key.payload_kind = captured_req.context.identity.payload_kind;
				req_key.terrain_version = captured_req.context.identity.terrain_version;
				req_key.source_version = captured_req.context.identity.source_version;
				req_key.committed_delta_version = captured_req.context.identity.publication_version;
				req_key.block_delta_content_version = captured_req.context.identity.block_content_version;

				if (h >= CPU_PAGE_POOL_SIZE ||
				    !(*slots)[h].in_use ||
				    (*slots)[h].generation != captured_req.slot_generation ||
				    !((*slots)[h].full_key == req_key))
				{
					decrement_in_flight();
					return;
				}

				// Also validate version identity.
				if (captured_req.context.identity.world_manifest_hash != current_snapshot.world_manifest_hash ||
				    captured_req.context.identity.recipe_hash != current_snapshot.recipe_identity.recipe_hash ||
				    captured_req.context.identity.terrain_version != current_snapshot.terrain_version ||
				    captured_req.context.identity.source_version != current_snapshot.source_version)
				{
					decrement_in_flight();
					return;
				}

				if (!all_valid) {
					// Mark Invalid — deterministic failure; not re-enqueued.
					(*slots)[h].record.state = TerrainSourceState::Invalid;
					(*page_data)[h].is_valid = false;
				} else {
					float lod_spacing = static_cast<float>(prof.get_lod_spacing(lod));
					float morph_allowance = lod_spacing * 0.5f;
					float residual_bound = 1.0f;
					float gradient_bound = 2.0f;

					new_page.page_contract_version = captured_req.context.identity.page_contract_version;
					new_page.payload_kind = captured_req.context.identity.payload_kind;
					new_page.committed_delta_version = captured_req.context.identity.publication_version;
					new_page.block_delta_content_version = captured_req.context.identity.block_content_version;

					(*page_data)[h].page = new_page;
					(*page_data)[h].min_height = min_h;
					(*page_data)[h].max_height = max_h;
					(*page_data)[h].residual_bound = residual_bound;
					(*page_data)[h].morph_allowance = morph_allowance;
					(*page_data)[h].gradient_bound = gradient_bound;
					(*page_data)[h].is_valid = true;
					(*page_data)[h].generation = (*slots)[h].generation;

					(*slots)[h].record.min_height = min_h;
					(*slots)[h].record.max_height = max_h;
					(*slots)[h].record.residual_bound = residual_bound;
					(*slots)[h].record.morph_allowance = morph_allowance;
					(*slots)[h].record.gradient_bound = gradient_bound;
					(*slots)[h].record.cpu_page_generation = (*slots)[h].generation;
					(*slots)[h].record.state = TerrainSourceState::Ready;
				}
			}

			decrement_in_flight();
		}
	);

	if (!submitted) {
		decrement_in_flight();
	} else {
		executor_submit_count_.fetch_add(1, std::memory_order_relaxed);
	}
	return submitted;
}

bool ConcreteTerrainRenderSource::try_read_page(
	uint64_t handle,
	uint32_t expected_generation,
	TerrainHeightPage& out_page
) const noexcept {
	std::lock_guard<std::mutex> lock(access_mutex);
	if (handle >= CPU_PAGE_POOL_SIZE) return false;
	if (!(*slots)[handle].in_use) return false;
	if ((*slots)[handle].generation != expected_generation) return false;
	if ((*slots)[handle].record.state != TerrainSourceState::Ready) return false;
	if (!(*page_data)[handle].is_valid) return false;
	if ((*page_data)[handle].page.payload_kind != (*slots)[handle].record.payload_kind) return false;

	out_page = (*page_data)[handle].page;
	return true;
}

#ifdef DEBUG_ENABLED
void ConcreteTerrainRenderSource::process_pending_jobs_sync(size_t max_jobs) noexcept {
	struct SyncJob {
		uint64_t slot_handle;
		uint32_t slot_generation;
		ConcreteTerrainSourceKey key;
		multinet::rendering::TerrainRenderBlockKey canonical_key;
		TerrainCommittedDeltaSnapshot delta_snapshot;
	};

	static constexpr size_t MAX_SYNC_JOBS = 256;
	std::array<SyncJob, MAX_SYNC_JOBS> jobs_to_run;
	size_t count = 0;

	size_t queue_jobs_processed = 0;
	{
		std::lock_guard<std::mutex> lock(access_mutex);
		size_t process_limit = std::min({ max_jobs, MAX_SYNC_JOBS, pending_queue_size });

		for (size_t p = 0; p < process_limit; ++p) {
			uint64_t h = (*pending_queue)[p].slot_handle;
			if (h < CPU_PAGE_POOL_SIZE && (*slots)[h].in_use && (*slots)[h].record.state == TerrainSourceState::Pending) {
				SyncJob job;
				job.slot_handle = h;
				job.slot_generation = (*slots)[h].generation;
				job.key = (*slots)[h].full_key;
				job.canonical_key = (*slots)[h].record.canonical_key;
				job.delta_snapshot = (*pending_queue)[p].context.committed_delta;
				jobs_to_run[count++] = job;
			}
		}
		queue_jobs_processed = process_limit;

		for (size_t i = 0; i < CPU_PAGE_POOL_SIZE && count < max_jobs && count < MAX_SYNC_JOBS; ++i) {
			if ((*slots)[i].in_use && (*slots)[i].record.state == TerrainSourceState::Pending) {
				bool already_collected = false;
				for (size_t c = 0; c < count; ++c) {
					if (jobs_to_run[c].slot_handle == i) {
						already_collected = true;
						break;
					}
				}
				if (!already_collected) {
					SyncJob job;
					job.slot_handle = i;
					job.slot_generation = (*slots)[i].generation;
					job.key = (*slots)[i].full_key;
					job.canonical_key = (*slots)[i].record.canonical_key;
					job.delta_snapshot = current_delta_snapshot;
					jobs_to_run[count++] = job;
				}
			}
		}

		if (queue_jobs_processed > 0) {
			size_t remaining = pending_queue_size - queue_jobs_processed;
			for (size_t r = 0; r < remaining; ++r) {
				(*pending_queue)[r] = (*pending_queue)[queue_jobs_processed + r];
			}
			pending_queue_size = remaining;
		}
	}

	if (count == 0) return;

	TerrainFieldEvaluator evaluator;
	bool evaluator_valid = false;
	try {
		if (domain.is_finite()) {
			evaluator = TerrainFieldEvaluator(
			FiniteCanonicalTerrainSignalV1(recipe, domain),
			domain,
			current_snapshot.terrain_version
		);
		} else {
			evaluator = TerrainFieldEvaluator(
				CanonicalTerrainSignalV1(recipe, manifest),
				manifest,
				current_snapshot.terrain_version
			);
		}
		evaluator_valid = true;
	} catch (...) {}

	const auto& prof = profile;

	for (size_t ji = 0; ji < count; ++ji) {
		const auto& job = jobs_to_run[ji];
		TerrainHeightPage new_page;
		float min_h = 1e9f;
		float max_h = -1e9f;
		bool all_valid = job.key.payload_kind == TerrainPagePayloadKind::AdditiveHeightDeltaV1 || evaluator_valid;

		if (all_valid) {
			uint8_t lod = job.canonical_key.lod;
			double block_size = prof.get_lod_block_size(lod);
			double spacing = prof.get_lod_spacing(lod);
			int32_t bx = job.canonical_key.block_u;
			int32_t bv = job.canonical_key.block_v;

			for (int j = 0; j < 19 && all_valid; ++j) {
				for (int i = 0; i < 19 && all_valid; ++i) {
					SurfacePosition64 pos;
					double u_m = 0.0;
					double v_m = 0.0;
					if (job.key.sample_patch.is_valid()) {
						if (!domain.is_valid() || !multinet::rendering::try_sample_patch_position(
							job.key.sample_patch,
							(i - 9) * spacing,
							(j - 9) * spacing,
							domain,
							pos)) {
							all_valid = false;
							break;
						}
						u_m = pos.u_m;
						v_m = pos.v_m;
					} else {
						u_m = bx * block_size + (i - 1) * spacing;
						v_m = bv * block_size + (j - 1) * spacing;
						pos.face = job.canonical_key.face;
						pos.u_m = u_m;
						pos.v_m = v_m;
						pos.altitude_m = 0.0;
						pos.topology_version = manifest.topology_version;
						pos.projection_version = manifest.projection_version;
					}

						float h = 0.0f;
						if (job.key.payload_kind == TerrainPagePayloadKind::AdditiveHeightDeltaV1) {
							if (job.delta_snapshot.field) {
								h = job.delta_snapshot.field->sample_delta(pos);
							}
						} else {
							TerrainHeightEvaluation eval = evaluator.evaluate(pos);
							if (!eval.valid) {
								all_valid = false;
								break;
							}
							h = static_cast<float>(eval.height);
						}
						new_page.samples_m[j * 19 + i] = h;

					if (i >= 1 && i <= 17 && j >= 1 && j <= 17) {
						min_h = std::min(min_h, h);
						max_h = std::max(max_h, h);
					}
				}
			}
		}

		if (min_h > max_h) {
			min_h = 0.0f;
			max_h = 0.0f;
		}

		{
			std::lock_guard<std::mutex> lock(access_mutex);
			uint64_t h = job.slot_handle;
			if (h >= CPU_PAGE_POOL_SIZE || !(*slots)[h].in_use) continue;
			if ((*slots)[h].generation != job.slot_generation) continue;
			if (!((*slots)[h].full_key == job.key)) continue;

			if (!all_valid) {
				(*slots)[h].record.state = TerrainSourceState::Invalid;
				(*page_data)[h].is_valid = false;
			} else {
				uint8_t lod = job.canonical_key.lod;
				float lod_spacing = static_cast<float>(prof.get_lod_spacing(lod));
				float morph_allowance = lod_spacing * 0.5f;
				float residual_bound = 1.0f;
				float gradient_bound = 2.0f;

				new_page.page_contract_version = job.key.page_contract_version;
				new_page.payload_kind = job.key.payload_kind;
				new_page.committed_delta_version = job.key.committed_delta_version;
				new_page.block_delta_content_version = job.key.block_delta_content_version;

				(*page_data)[h].page = new_page;
				(*page_data)[h].min_height = min_h;
				(*page_data)[h].max_height = max_h;
				(*page_data)[h].residual_bound = residual_bound;
				(*page_data)[h].morph_allowance = morph_allowance;
				(*page_data)[h].gradient_bound = gradient_bound;
				(*page_data)[h].is_valid = true;
				(*page_data)[h].generation = (*slots)[h].generation;

				(*slots)[h].record.min_height = min_h;
				(*slots)[h].record.max_height = max_h;
				(*slots)[h].record.residual_bound = residual_bound;
				(*slots)[h].record.morph_allowance = morph_allowance;
				(*slots)[h].record.gradient_bound = gradient_bound;
				(*slots)[h].record.cpu_page_generation = (*slots)[h].generation;
				(*slots)[h].record.state = TerrainSourceState::Ready;
			}
		}
	}
}
#endif

bool ConcreteTerrainRenderSource::try_query_record(
	const multinet::rendering::TerrainRenderBlockKey& key,
	TerrainSourceRecord& out_record
) const noexcept {
	TerrainRenderPublicationView pub = get_publication_view();
	TerrainPageRequestContext ctx = make_page_request_context(key, profile, pub, manifest);
	return try_query_record(ctx.identity, out_record);
}

TerrainSourceRequestResult ConcreteTerrainRenderSource::request_record(
	const multinet::rendering::TerrainRenderBlockKey& key,
	const TerrainRequestMetadata& metadata
) noexcept {
	TerrainRenderPublicationView pub = get_publication_view();
	TerrainPageRequestContext ctx = make_page_request_context(key, profile, pub, manifest);
	return request_record(ctx, metadata);
}

bool ConcreteTerrainRenderSource::mark_wanted(
	const multinet::rendering::TerrainRenderBlockKey& key,
	TerrainRequestClass request_class,
	int64_t distance_sq_m,
	uint64_t epoch
) noexcept {
	TerrainRenderPublicationView pub = get_publication_view();
	TerrainPageRequestContext ctx = make_page_request_context(key, profile, pub, manifest);
	return mark_wanted(ctx.identity, request_class, distance_sq_m, epoch);
}

size_t ConcreteTerrainRenderSource::in_use_record_count() const noexcept {
	std::lock_guard<std::mutex> lock(access_mutex);
	size_t count = 0;
	for (size_t i = 0; i < CPU_PAGE_POOL_SIZE; ++i) {
		if ((*slots)[i].in_use) ++count;
	}
	return count;
}

size_t ConcreteTerrainRenderSource::missing_in_use_record_count() const noexcept {
	std::lock_guard<std::mutex> lock(access_mutex);
	size_t count = 0;
	for (size_t i = 0; i < CPU_PAGE_POOL_SIZE; ++i) {
		if ((*slots)[i].in_use && (*slots)[i].record.state == TerrainSourceState::Missing) ++count;
	}
	return count;
}

uint32_t ConcreteTerrainRenderSource::get_pending_queue_count() const noexcept {
	std::lock_guard<std::mutex> lock(access_mutex);
	return static_cast<uint32_t>(pending_queue_size);
}

uint32_t ConcreteTerrainRenderSource::get_pending_record_count() const noexcept {
	std::lock_guard<std::mutex> lock(access_mutex);
	uint32_t count = 0;
	for (size_t i = 0; i < CPU_PAGE_POOL_SIZE; ++i) {
		if ((*slots)[i].in_use && (*slots)[i].record.state == TerrainSourceState::Pending) ++count;
	}
	return count;
}

uint32_t ConcreteTerrainRenderSource::get_in_flight_count() const noexcept {
	return in_flight_count_.load(std::memory_order_relaxed);
}

uint64_t ConcreteTerrainRenderSource::get_executor_submit_count() const noexcept {
	return executor_submit_count_.load(std::memory_order_relaxed);
}

uint32_t ConcreteTerrainRenderSource::get_cancelled_incompatible_count() const noexcept {
	return cancelled_retryable_count_.load(std::memory_order_relaxed);
}

uint64_t ConcreteTerrainRenderSource::get_request_record_call_count() const noexcept {
	return request_record_call_count_.load(std::memory_order_relaxed);
}

uint64_t ConcreteTerrainRenderSource::get_rejected_delta_publication_count() const noexcept {
	return rejected_delta_publication_count_.load(std::memory_order_relaxed);
}

uint32_t ConcreteTerrainRenderSource::get_ready_record_count() const noexcept {
	std::lock_guard<std::mutex> lock(access_mutex);
	uint32_t count = 0;
	for (size_t i = 0; i < CPU_PAGE_POOL_SIZE; ++i) {
		if ((*slots)[i].in_use && (*slots)[i].record.state == TerrainSourceState::Ready) ++count;
	}
	return count;
}

uint32_t ConcreteTerrainRenderSource::get_ready_empty_record_count() const noexcept {
	std::lock_guard<std::mutex> lock(access_mutex);
	uint32_t count = 0;
	for (size_t i = 0; i < CPU_PAGE_POOL_SIZE; ++i) {
		if ((*slots)[i].in_use && (*slots)[i].record.state == TerrainSourceState::Ready && (*slots)[i].record.block_delta_content_version == 1) ++count;
	}
	return count;
}

uint32_t ConcreteTerrainRenderSource::get_invalid_record_count() const noexcept {
	std::lock_guard<std::mutex> lock(access_mutex);
	uint32_t count = 0;
	for (size_t i = 0; i < CPU_PAGE_POOL_SIZE; ++i) {
		if ((*slots)[i].in_use && (*slots)[i].record.state == TerrainSourceState::Invalid) ++count;
	}
	return count;
}

uint32_t ConcreteTerrainRenderSource::get_missing_record_count() const noexcept {
	std::lock_guard<std::mutex> lock(access_mutex);
	uint32_t count = 0;
	for (size_t i = 0; i < CPU_PAGE_POOL_SIZE; ++i) {
		if (!(*slots)[i].in_use || (*slots)[i].record.state == TerrainSourceState::Missing) ++count;
	}
	return count;
}

} // namespace Multinet
