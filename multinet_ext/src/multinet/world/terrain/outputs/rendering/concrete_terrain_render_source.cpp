#include "concrete_terrain_render_source.h"
#include "multinet/world/terrain/canonical_terrain_signal.h"
#include "multinet/world/terrain/heightfield_generator.h"

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
	  page_data(std::make_unique<std::array<ConcreteTerrainPageData, CPU_PAGE_POOL_SIZE>>()) {
	
	current_snapshot.recipe_identity = recipe.identity;
	current_snapshot.world_manifest_hash = manifest.manifest_hash;
	current_snapshot.topology_version = manifest.topology_version;
	current_snapshot.projection_version = manifest.projection_version;
	current_snapshot.terrain_version = 1;
	current_snapshot.source_version = 1;

	current_snapshot.fallback_bounds.minimum_height = recipe.legacy_signals.min_elevation_m;
	current_snapshot.fallback_bounds.maximum_height = recipe.legacy_signals.max_elevation_m;
	current_snapshot.fallback_bounds.residual_bound = 1.0f;
	current_snapshot.fallback_bounds.morph_allowance = 0.5f;
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
	return current_snapshot;
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

	// Evict an Invalid or Ready (non-Pending) slot.
	for (size_t i = 0; i < CPU_PAGE_POOL_SIZE; ++i) {
		auto state = (*slots)[i].record.state;
		if (state == TerrainSourceState::Ready || state == TerrainSourceState::Invalid) {
			(*slots)[i].in_use = true;
			// Increment generation so any in-flight job for the old slot is invalidated on publish.
			(*slots)[i].generation = ++next_generation;
			return i;
		}
	}

	return std::nullopt;
}

TerrainSourceRecord ConcreteTerrainRenderSource::get_or_request_record(
	const multinet::rendering::TerrainRenderBlockKey& key
) noexcept {
	// Reject unknown profiles or out-of-range LOD for profile consistency.
	if (key.profile != ORDINARY_BCCM_V1_PROFILE || key.lod >= profile.level_count) {
		TerrainSourceRecord missing{};
		missing.canonical_key = key;
		missing.state = TerrainSourceState::Missing;
		return missing;
	}

	std::unique_lock<std::mutex> lock(access_mutex);

	if (shutting_down_.load(std::memory_order_acquire)) {
		TerrainSourceRecord missing{};
		missing.canonical_key = key;
		missing.state = TerrainSourceState::Missing;
		return missing;
	}

	ConcreteTerrainSourceKey source_key;
	source_key.block_key = key;
	source_key.world_manifest_hash = current_snapshot.world_manifest_hash;
	source_key.recipe_hash = current_snapshot.recipe_identity.recipe_hash;
	source_key.terrain_version = current_snapshot.terrain_version;
	source_key.source_version = current_snapshot.source_version;

	auto existing_slot = find_existing_slot(source_key);
	if (existing_slot.has_value()) {
		uint64_t handle = existing_slot.value();
		TerrainSourceRecord record = (*slots)[handle].record;
		record.cpu_page_generation = (*slots)[handle].generation;
		if ((*page_data)[handle].is_valid) {
			record.state = TerrainSourceState::Ready;
		} else if (record.state == TerrainSourceState::Invalid) {
			// Already failed deterministically — do not re-enqueue.
		} else {
			record.state = TerrainSourceState::Pending;
		}
		return record;
	}

	auto new_slot = allocate_slot();
	if (!new_slot.has_value()) {
		TerrainSourceRecord empty{};
		empty.canonical_key = key;
		empty.state = TerrainSourceState::Missing;
		return empty;
	}

	uint64_t handle = new_slot.value();
	uint32_t gen = (*slots)[handle].generation;

	(*slots)[handle].full_key = source_key;
	(*slots)[handle].record.canonical_key = key;
	(*slots)[handle].record.cpu_page_handle = handle;
	(*slots)[handle].record.cpu_page_generation = gen;
	(*slots)[handle].record.state = TerrainSourceState::Pending;
	(*slots)[handle].record.terrain_version = source_key.terrain_version;
	(*slots)[handle].record.source_version = source_key.source_version;
	(*slots)[handle].record.min_height = 0.0f;
	(*slots)[handle].record.max_height = 0.0f;
	(*slots)[handle].record.residual_bound = 0.0f;
	(*slots)[handle].record.gradient_bound = 0.0f;
	(*slots)[handle].record.previous_fallback_handle = 0;

	(*page_data)[handle].source_key = source_key;
	(*page_data)[handle].is_valid = false;
	(*page_data)[handle].generation = gen;

	PendingRequest req;
	req.key = source_key;
	req.slot_handle = handle;
	req.slot_generation = gen;
	req.manifest_hash = source_key.world_manifest_hash;
	req.recipe_hash = source_key.recipe_hash;
	req.terrain_version = source_key.terrain_version;
	req.source_version = source_key.source_version;
	req.canonical_key = key;

	TerrainSourceRecord record_snapshot = (*slots)[handle].record;

	if (generation_mode == TerrainPageGenerationMode::AsynchronousProduction) {
		if (pending_queue_size < PENDING_REQUEST_CAPACITY) {
			pending_queue[pending_queue_size++] = req;
		} else {
			// Queue full — release slot so it becomes retryable
			if (handle < CPU_PAGE_POOL_SIZE &&
			    (*slots)[handle].in_use &&
			    (*slots)[handle].generation == gen) {
				(*slots)[handle].in_use = false;
			}
			TerrainSourceRecord missing{};
			missing.canonical_key = key;
			missing.state = TerrainSourceState::Missing;
			return missing;
		}
		return record_snapshot;
	} else {
		// SynchronousDiagnostic mode — do not submit to background executor
		return record_snapshot;
	}
}

void ConcreteTerrainRenderSource::commit_pending_requests(const multinet::rendering::TerrainRenderBlockKey& camera_key) noexcept {
	std::unique_lock<std::mutex> lock(access_mutex);
	
	if (pending_queue_size == 0) return;

	// Calculate priority for each queued request.
	struct RankedRequest {
		PendingRequest req;
		float priority;
		bool submitted;
	};

	std::array<RankedRequest, PENDING_REQUEST_CAPACITY> ranked;
	size_t ranked_count = 0;

	for (size_t i = 0; i < pending_queue_size; ++i) {
		const auto& r = pending_queue[i];
		// Distance proxy in grid space
		float du = static_cast<float>(r.canonical_key.block_u - camera_key.block_u);
		float dv = static_cast<float>(r.canonical_key.block_v - camera_key.block_v);
		float dist = std::sqrt(du * du + dv * dv);
		float lod_penalty = static_cast<float>(r.canonical_key.lod) * 1000.0f;
		
		RankedRequest rr;
		rr.req = r;
		rr.priority = dist + lod_penalty;
		rr.submitted = false;
		ranked[ranked_count++] = rr;
	}

	// Pass 1: Reserve up to 2 slots per active LOD.
	// Sort primarily by LOD (coarse to fine) and then by priority.
	std::sort(ranked.begin(), ranked.begin() + ranked_count, [](const RankedRequest& a, const RankedRequest& b) {
		if (a.req.canonical_key.lod != b.req.canonical_key.lod) {
			return a.req.canonical_key.lod > b.req.canonical_key.lod; // coarse to fine
		}
		return a.priority < b.priority;
	});

	int current_lod = -1;
	int lod_count = 0;
	for (size_t i = 0; i < ranked_count; ++i) {
		auto& rr = ranked[i];
		if (rr.req.canonical_key.lod != current_lod) {
			current_lod = rr.req.canonical_key.lod;
			lod_count = 0;
		}
		if (lod_count < 2) {
			if (submit_generation_job(rr.req)) {
				rr.submitted = true;
				lod_count++;
			} else {
				// Executor queue is completely full
				break;
			}
		}
	}

	// Pass 2: Spend remaining budget on highest importance candidates.
	std::sort(ranked.begin(), ranked.begin() + ranked_count, [](const RankedRequest& a, const RankedRequest& b) {
		return a.priority < b.priority;
	});

	for (size_t i = 0; i < ranked_count; ++i) {
		auto& rr = ranked[i];
		if (rr.submitted) continue;
		
		if (submit_generation_job(rr.req)) {
			rr.submitted = true;
		} else {
			// Executor queue is full
			break;
		}
	}

	// For any requests that weren't submitted, we must release their slot
	for (size_t i = 0; i < ranked_count; ++i) {
		auto& rr = ranked[i];
		if (!rr.submitted) {
			uint64_t handle = rr.req.slot_handle;
			uint32_t gen = rr.req.slot_generation;
			if (handle < CPU_PAGE_POOL_SIZE &&
			    (*slots)[handle].in_use &&
			    (*slots)[handle].generation == gen) {
				(*slots)[handle].in_use = false;
			}
		}
	}

	pending_queue_size = 0;
}

bool ConcreteTerrainRenderSource::submit_generation_job(const PendingRequest& req) noexcept {
	PendingRequest captured_req = req;
	TerrainRecipe captured_recipe = recipe;
	WorldScaleManifest captured_manifest = manifest;

	in_flight_count_.fetch_add(1, std::memory_order_acquire);

	bool submitted = executor.submit(
		JobPriority::NORMAL,
		[this, captured_req, captured_recipe, captured_manifest]() mutable {
			if (shutting_down_.load(std::memory_order_acquire)) {
				decrement_in_flight();
				return;
			}

			TerrainHeightPage new_page;
			float min_h = 1e9f;
			float max_h = -1e9f;
			bool all_valid = true;

			uint8_t lod = captured_req.canonical_key.lod;
			const auto& prof = profile;
			double block_size = prof.get_lod_block_size(lod);
			double spacing = prof.get_lod_spacing(lod);
			int32_t bx = captured_req.canonical_key.block_u;
			int32_t bv = captured_req.canonical_key.block_v;

			TerrainFieldEvaluator evaluator;
			try {
				// Step 2: Use CanonicalTerrainSignalV1 which projects correctly onto a sphere
				evaluator = TerrainFieldEvaluator(
					CanonicalTerrainSignalV1(captured_recipe, captured_manifest),
					captured_manifest,
					captured_req.terrain_version
				);
			} catch (const std::exception& e) {
				// Evaluator construction failed (version mismatch etc.); mark Invalid.
				all_valid = false;
				std::cerr << "[TERRAIN-SOURCE] Evaluator construction THREW: " << e.what()
				          << " recipe_hash=" << captured_recipe.identity.recipe_hash
				          << " manifest_hash=" << captured_manifest.manifest_hash
				          << " recipe.manifest_hash=" << captured_recipe.identity.manifest_hash
				          << std::endl;
			} catch (...) {
				all_valid = false;
				std::cerr << "[TERRAIN-SOURCE] Evaluator construction THREW unknown exception" << std::endl;
			}

			if (all_valid) {
				for (int j = 0; j < 19 && all_valid; ++j) {
					for (int i = 0; i < 19 && all_valid; ++i) {
						double u_m = bx * block_size + (i - 1) * spacing;
						double v_m = bv * block_size + (j - 1) * spacing;

						SurfacePosition64 pos;
						pos.face = captured_req.canonical_key.face;
						pos.u_m = u_m;
						pos.v_m = v_m;
						pos.altitude_m = 0.0;
						pos.topology_version = captured_manifest.topology_version;
						pos.projection_version = captured_manifest.projection_version;

						TerrainHeightEvaluation eval = evaluator.evaluate(pos);
						if (!eval.valid) {
							all_valid = false;
							std::cerr << "[TERRAIN-SOURCE] eval.valid=false face=" << (int)captured_req.canonical_key.face
							          << " u=" << u_m << " v=" << v_m << std::endl;
							break;
						}

						float h = static_cast<float>(eval.height);
						new_page.heights[j * 19 + i] = h;

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

			// Publish — reacquire lock and validate generation matches.
			{
				std::lock_guard<std::mutex> lock(access_mutex);

				if (shutting_down_.load(std::memory_order_acquire)) {
					decrement_in_flight();
					return;
				}

				uint64_t h = captured_req.slot_handle;
				if (h >= CPU_PAGE_POOL_SIZE ||
				    !(*slots)[h].in_use ||
				    (*slots)[h].generation != captured_req.slot_generation ||
				    !((*slots)[h].full_key == captured_req.key))
				{
					// Slot was reused for a different request — discard.
					decrement_in_flight();
					return;
				}

				// Also validate version identity.
				if (captured_req.manifest_hash != current_snapshot.world_manifest_hash ||
				    captured_req.recipe_hash != current_snapshot.recipe_identity.recipe_hash ||
				    captured_req.terrain_version != current_snapshot.terrain_version ||
				    captured_req.source_version != current_snapshot.source_version)
				{
					std::cerr << "[TERRAIN-SOURCE] Version validation FAILED: mh=" << captured_req.manifest_hash
					          << " vs " << current_snapshot.world_manifest_hash
					          << " rh=" << captured_req.recipe_hash
					          << " vs " << current_snapshot.recipe_identity.recipe_hash
					          << " tv=" << captured_req.terrain_version
					          << " vs " << current_snapshot.terrain_version
					          << " sv=" << captured_req.source_version
					          << " vs " << current_snapshot.source_version
					          << std::endl;
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
					static int ready_count = 0;
					if (++ready_count <= 5) {
						std::cerr << "[TERRAIN-SOURCE] Page READY slot=" << h
						          << " face=" << (int)captured_req.canonical_key.face
						          << " lod=" << (int)captured_req.canonical_key.lod
						          << " h_range=[" << min_h << "," << max_h << "]"
						          << std::endl;
					}
				}
			}

			decrement_in_flight();
		}
	);

	if (!submitted) {
		decrement_in_flight();
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
	};

	static constexpr size_t MAX_SYNC_JOBS = 256;
	std::array<SyncJob, MAX_SYNC_JOBS> jobs_to_run;
	size_t count = 0;

	{
		std::lock_guard<std::mutex> lock(access_mutex);
		for (size_t i = 0; i < CPU_PAGE_POOL_SIZE && count < max_jobs && count < MAX_SYNC_JOBS; ++i) {
			if ((*slots)[i].in_use && (*slots)[i].record.state == TerrainSourceState::Pending) {
				SyncJob job;
				job.slot_handle = i;
				job.slot_generation = (*slots)[i].generation;
				job.key = (*slots)[i].full_key;
				job.canonical_key = (*slots)[i].record.canonical_key;
				jobs_to_run[count++] = job;
			}
		}
	}

	if (count == 0) return;

	TerrainFieldEvaluator evaluator;
	bool evaluator_valid = false;
	try {
		// Same as async path: use CanonicalTerrainSignalV1 for runtime accuracy.
		evaluator = TerrainFieldEvaluator(
			CanonicalTerrainSignalV1(recipe, manifest),
			manifest,
			current_snapshot.terrain_version
		);
		evaluator_valid = true;
	} catch (...) {}

	const auto& prof = profile;

	for (size_t ji = 0; ji < count; ++ji) {
		const auto& job = jobs_to_run[ji];
		TerrainHeightPage new_page;
		float min_h = 1e9f;
		float max_h = -1e9f;
		bool all_valid = evaluator_valid;

		if (all_valid) {
			uint8_t lod = job.canonical_key.lod;
			double block_size = prof.get_lod_block_size(lod);
			double spacing = prof.get_lod_spacing(lod);
			int32_t bx = job.canonical_key.block_u;
			int32_t bv = job.canonical_key.block_v;

			for (int j = 0; j < 19 && all_valid; ++j) {
				for (int i = 0; i < 19 && all_valid; ++i) {
					double u_m = bx * block_size + (i - 1) * spacing;
					double v_m = bv * block_size + (j - 1) * spacing;

					SurfacePosition64 pos;
					pos.face = job.canonical_key.face;
					pos.u_m = u_m;
					pos.v_m = v_m;
					pos.altitude_m = 0.0;
					pos.topology_version = manifest.topology_version;
					pos.projection_version = manifest.projection_version;

					TerrainHeightEvaluation eval = evaluator.evaluate(pos);
					if (!eval.valid) {
						all_valid = false;
						break;
					}

					float h = static_cast<float>(eval.height);
					new_page.heights[j * 19 + i] = h;

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

} // namespace Multinet
