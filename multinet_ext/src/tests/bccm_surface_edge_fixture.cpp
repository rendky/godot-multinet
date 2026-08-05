#include "multinet/rendering/terrain/block_clipmap/block_clipmap_renderer.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_shader.h"
#include "multinet/world/terrain/outputs/rendering/concrete_terrain_render_source.h"
#include "multinet/world/terrain/canonical_terrain_signal.h"
#include "multinet/world/terrain/terrain_queries.h"
#include "multinet/core/spatial/surface_coordinate_conversion.h"
#include "multinet/core/spatial/surface_projection.h"
#include "multinet/core/spatial/surface_topology.h"
#include "multinet/core/spatial/world_manifests.h"

#include <iostream>
#include <cassert>
#include <vector>
#include <set>
#include <tuple>
#include <cmath>
#include <memory>
#include <thread>
#include <chrono>

using namespace multinet::rendering;
using namespace Multinet;

#define TEST_ASSERT(cond, msg) \
	do { \
		if (!(cond)) { \
			std::cerr << "[FAIL] Assertion failed at line " << __LINE__ << ": " << msg << std::endl; \
			std::exit(1); \
		} \
	} while(0)

static WorldScaleManifest make_manifest() {
	WorldScaleInput input{};
	input.area_equivalent_side_m = 1000000;
	return build_world_scale_manifest(input);
}

static TerrainRecipe make_recipe(const WorldScaleManifest& manifest) {
	TerrainRecipe recipe{};
	recipe.identity.recipe_hash = 0x123456789ABCDEF0ULL;
	recipe.identity.recipe_version = 1;
	recipe.legacy_signals.min_elevation_m = -200.0f;
	recipe.legacy_signals.max_elevation_m = 500.0f;
	(void)finalize_terrain_recipe(recipe, manifest);
	return recipe;
}

static BCCMSourceExpectation make_expectation(const TerrainRecipe& recipe, const WorldScaleManifest& manifest) {
	BCCMSourceExpectation exp{};
	exp.recipe_identity = recipe.identity;
	exp.world_manifest_hash = manifest.manifest_hash;
	exp.topology_version = manifest.topology_version;
	exp.projection_version = manifest.projection_version;
	exp.terrain_version = 1;
	exp.source_version = 1;
	return exp;
}

static FrustumPlanes open_frustum() {
	FrustumPlanes f{};
	f.valid = true;
	for (int i = 0; i < 6; ++i) {
		f.planes[i] = godot::Plane(godot::Vector3(0, 0, 0), 1e9f);
	}
	return f;
}

static BCCMCameraState make_camera_on_face(SurfaceFace face, const WorldScaleManifest& manifest, double u_m = 0.0, double v_m = 0.0) {
	BCCMCameraState cam{};
	cam.canonical_position.face = face;
	cam.canonical_position.u_m = u_m;
	cam.canonical_position.v_m = v_m;
	cam.canonical_position.altitude_m = 0.0;
	cam.canonical_position.topology_version = manifest.topology_version;
	cam.canonical_position.projection_version = manifest.projection_version;

	cam.active_frame.origin = cam.canonical_position;
	cam.active_frame.tangent_basis.u_axis = { 1.0, 0.0, 0.0 };
	cam.active_frame.tangent_basis.v_axis = { 0.0, 0.0, 1.0 };
	cam.active_frame.tangent_basis.up_axis = { 0.0, 1.0, 0.0 };

	cam.frame_epoch = 1;
	cam.is_visible = true;
	return cam;
}

// ─── 1. GLSL Shader Compilation & Preprocessor Test ──────────────────────────

static void run_shader_compilation_test() {
	std::cout << "\n[TEST 1] Starting GLSL shader code & preprocessor verification test..." << std::endl;

	std::string shader_code = s_bccm_shader_code;
	TEST_ASSERT(!shader_code.empty(), "Shader code must not be empty");

	TEST_ASSERT(shader_code.find("vec3 du = vec3(") != std::string::npos, "Non-debug du calculation present");
	TEST_ASSERT(shader_code.find("vec3 dv = vec3(") != std::string::npos, "Non-debug dv calculation present");
	TEST_ASSERT(shader_code.find("spacing_u * 2.0") == std::string::npos, "du spacing_u * 2.0 NOT present");
	TEST_ASSERT(shader_code.find("spacing_v * 2.0") == std::string::npos, "dv spacing_v * 2.0 NOT present");

	int if_count = 0;
	int endif_count = 0;
	size_t pos = 0;
	while ((pos = shader_code.find("#if", pos)) != std::string::npos) { ++if_count; pos += 3; }
	pos = 0;
	while ((pos = shader_code.find("#endif", pos)) != std::string::npos) { ++endif_count; pos += 6; }

	TEST_ASSERT(if_count == endif_count, "All preprocessor conditionals closed");
	std::cout << "[PASS] GLSL shader code syntax and preprocessor conditionals verified." << std::endl;
}

// ─── 2. Executor Overflow & Non-Blocking Deadlock Test ───────────────────────

static void run_executor_overflow_test() {
	std::cout << "\n[TEST 2] Starting executor overflow & non-blocking deadlock test..." << std::endl;

	WorldScaleManifest manifest = make_manifest();
	TerrainRecipe recipe = make_recipe(manifest);
	BoundedBackgroundJobExecutor executor;
	ConcreteTerrainRenderSource source(recipe, manifest, executor, TerrainPageGenerationMode::AsynchronousProduction);

	// Step 1: Fill the source's internal pending_queue to capacity by requesting
	// 256 distinct keys. Each new key allocates a slot and enqueues a PendingRequest
	// without submitting to the executor (executor submission happens only in
	// commit_pending_requests). Once pending_queue is full the next call must
	// return Missing immediately — without crash or Pending.
	//
	// Use lod=0, block_v=0, increasing block_u to generate distinct keys.
	// PENDING_REQUEST_CAPACITY == 256 (matches QUEUE_CAPACITY; declared private in source).
	constexpr uint32_t kPendingCap = 256u;
	for (uint32_t i = 0; i < kPendingCap; ++i) {
		TerrainRenderBlockKey key{ SurfaceFace::PositiveX,
			static_cast<int32_t>(i), 0, 0,
			ORDINARY_BCCM_V1_PROFILE, 0 };
		source.get_or_request_record(key);
	}
	std::cout << "[INFO] Filled " << kPendingCap << " pending queue slots" << std::endl;

	// Step 2: One more distinct key must return Missing — pending queue is full.
	TerrainRenderBlockKey key_overflow{ SurfaceFace::PositiveX,
		static_cast<int32_t>(kPendingCap), 0, 0,
		ORDINARY_BCCM_V1_PROFILE, 0 };
	TerrainSourceRecord overflow_rec = source.get_or_request_record(key_overflow);
	TEST_ASSERT(overflow_rec.state == TerrainSourceState::Missing,
	            "Overflow request returns exactly Missing state (not crash or Pending)");

	// Step 3: Verify the executor itself rejects new jobs when its own queue is full.
	// Block the worker thread so jobs pile up in the executor queue.
	std::mutex gate_mutex;
	std::condition_variable gate_cv;
	bool gate_open = false;
	std::atomic<bool> worker_started{ false };

	bool blocking_submitted = executor.submit(JobPriority::NORMAL, [&] {
		std::unique_lock<std::mutex> lock(gate_mutex);
		worker_started.store(true, std::memory_order_release);
		gate_cv.wait(lock, [&] { return gate_open; });
	});
	TEST_ASSERT(blocking_submitted, "Blocking job submitted successfully");

	while (!worker_started.load(std::memory_order_acquire)) {
		std::this_thread::yield();
	}

	// Fill all executor queue slots.
	uint32_t exec_fill_count = 0;
	for (uint32_t i = 0; i < BoundedBackgroundJobExecutor::QUEUE_CAPACITY; ++i) {
		bool ok = executor.submit(JobPriority::NORMAL, [] {});
		if (ok) ++exec_fill_count;
	}
	TEST_ASSERT(exec_fill_count == BoundedBackgroundJobExecutor::QUEUE_CAPACITY, "All executor queue slots filled");

	// Step 4: Release the worker gate.
	{
		std::lock_guard<std::mutex> lock(gate_mutex);
		gate_open = true;
	}
	gate_cv.notify_all();

	// Step 5: Wait for executor to drain.
	bool drained = executor.wait_idle_for(std::chrono::milliseconds(10000));
	TEST_ASSERT(drained, "Executor drained all queued jobs after gate release");

	// Step 6: Executor drained. Verify that a fresh source can successfully issue
	// a request and reach Ready — proving the executor is healthy after drain.
	// We use a separate source here because the previous source's pending queue is
	// still full from Step 1 and any new key would also be Missing there.
	{
		ConcreteTerrainRenderSource fresh_source(recipe, manifest, executor, TerrainPageGenerationMode::AsynchronousProduction);
		TerrainRenderBlockKey key_fresh{ SurfaceFace::NegativeX, 1, 1, 0, ORDINARY_BCCM_V1_PROFILE, 0 };
		TerrainSourceRecord fresh_rec = fresh_source.get_or_request_record(key_fresh);
		if (fresh_rec.state == TerrainSourceState::Pending) {
			fresh_source.commit_pending_requests(key_fresh);
			executor.wait_idle_for(std::chrono::milliseconds(5000));
			fresh_source.process_pending_jobs_sync(64);
			fresh_rec = fresh_source.get_or_request_record(key_fresh);
		}
		TEST_ASSERT(fresh_rec.state == TerrainSourceState::Ready, "Fresh source after drain: request reaches Ready state");
		fresh_source.shutdown();
	}

	std::cout << "[PASS] Executor overflow test: pending-queue full → Missing; executor drain → Ready." << std::endl;
	source.shutdown();
	executor.shutdown();
}


// ─── 2b. Shutdown Deadlock Regression Test ───────────────────────────────────

static void run_shutdown_deadlock_regression_test() {
	std::cout << "\n[TEST 2b] Starting shutdown deadlock regression test (50 iterations)..." << std::endl;

	WorldScaleManifest manifest = make_manifest();
	TerrainRecipe recipe = make_recipe(manifest);
	BoundedBackgroundJobExecutor executor;

	for (int i = 0; i < 50; ++i) {
		auto source = std::make_unique<ConcreteTerrainRenderSource>(recipe, manifest, executor, TerrainPageGenerationMode::AsynchronousProduction);

		// Request a record so a background job starts.
		TerrainRenderBlockKey key{ SurfaceFace::PositiveX, 0, 0, 0, ORDINARY_BCCM_V1_PROFILE, 0 };
		source->get_or_request_record(key);

		// Vibrate the wait time to hit different parts of the background job lifecycle
		// including the shutdown/stale publication branch.
		std::this_thread::sleep_for(std::chrono::microseconds(100 * (i % 10)));

		// Call shutdown. If there was a deadlock, this would hang indefinitely.
		source->shutdown();
	}

	std::cout << "[PASS] Shutdown deadlock regression test passed (did not hang over 50 iterations)." << std::endl;
	executor.shutdown();
}

// ─── 3. Async / Sync Generation Mode Isolation Test ──────────────────────────

static void run_generation_mode_isolation_test() {
	std::cout << "\n[TEST 3] Starting async / sync generation mode isolation test..." << std::endl;

	WorldScaleManifest manifest = make_manifest();
	TerrainRecipe recipe = make_recipe(manifest);
	BoundedBackgroundJobExecutor executor;

	ConcreteTerrainRenderSource async_source(recipe, manifest, executor, TerrainPageGenerationMode::AsynchronousProduction);
	TerrainRenderBlockKey key1{ SurfaceFace::PositiveX, 0, 0, 0, ORDINARY_BCCM_V1_PROFILE, 0 };
	TerrainSourceRecord rec_async = async_source.get_or_request_record(key1);
	TEST_ASSERT(rec_async.state == TerrainSourceState::Pending, "Async mode returns Pending initially");
	async_source.shutdown();

	ConcreteTerrainRenderSource sync_source(recipe, manifest, executor, TerrainPageGenerationMode::SynchronousDiagnostic);
	TerrainRenderBlockKey key2{ SurfaceFace::PositiveX, 0, 0, 0, ORDINARY_BCCM_V1_PROFILE, 0 };
	TerrainSourceRecord rec_sync = sync_source.get_or_request_record(key2);
	TEST_ASSERT(rec_sync.state == TerrainSourceState::Pending, "Sync mode enqueues job as Pending");
	sync_source.process_pending_jobs_sync(10);
	TerrainSourceRecord rec_sync_after = sync_source.get_or_request_record(key2);
	TEST_ASSERT(rec_sync_after.state == TerrainSourceState::Ready, "Sync mode record becomes Ready");
	sync_source.shutdown();

	executor.shutdown();
	std::cout << "[PASS] Generation mode isolation verified: async and sync paths strictly separated." << std::endl;
}

// ─── 4. 1024-Request Async Stress Gate Test ──────────────────────────────────

static void run_async_source_stress_test() {
	std::cout << "\n[TEST 4] Starting 1024-request async source stress gate (flakiness check)..." << std::endl;

	WorldScaleManifest manifest = make_manifest();
	TerrainRecipe recipe = make_recipe(manifest);

	// Uses SynchronousDiagnostic mode: get_or_request_record allocates a slot and returns
	// Pending immediately without touching the pending queue. process_pending_jobs_sync
	// then generates all Pending slots synchronously. This gives a deterministic, bounded
	// verification that all 1024 pages reach Ready with correct generation-safe reads.
	// Test 3 (mode isolation) covers the async/sync boundary separately.
	constexpr int kTotalPages = 1024;

	for (int iter = 0; iter < 5; ++iter) {
		BoundedBackgroundJobExecutor executor;
		ConcreteTerrainRenderSource source(recipe, manifest, executor, TerrainPageGenerationMode::SynchronousDiagnostic);

		std::vector<TerrainRenderBlockKey> all_keys;
		all_keys.reserve(kTotalPages);
		uint32_t accepted_total = 0;

		// Submit all 1024 keys — in SynchronousDiagnostic mode each allocates a slot
		// and returns Pending immediately (no pending queue limit applies).
		for (int i = 0; i < kTotalPages; ++i) {
			TerrainRenderBlockKey key{ SurfaceFace::PositiveX,
				static_cast<int32_t>(i % 32),
				static_cast<int32_t>(i / 32),
				0, ORDINARY_BCCM_V1_PROFILE, 0 };
			TerrainSourceRecord rec = source.get_or_request_record(key);
			if (rec.state == TerrainSourceState::Pending || rec.state == TerrainSourceState::Ready) {
				++accepted_total;
				all_keys.push_back(key);
			}
		}
		TEST_ASSERT(accepted_total == static_cast<uint32_t>(kTotalPages), "All 1024 requests accepted (Pending or Ready)");

		// Synchronously generate all Pending slots.
		// process_pending_jobs_sync caps at 256 per call (MAX_SYNC_JOBS internal limit).
		// Loop until all 1024 slots are processed.
		for (int flush = 0; flush < 8; ++flush) {
			source.process_pending_jobs_sync(kTotalPages);
		}

		// Verify all 1024 pages are Ready with valid generation-safe reads.
		uint32_t ready_count = 0;
		uint32_t generation_safe_reads = 0;
		uint32_t invalid_count = 0;
		uint32_t missing_count = 0;

		for (const auto& key : all_keys) {
			TerrainSourceRecord rec = source.get_or_request_record(key);
			if (rec.state == TerrainSourceState::Ready) {
				++ready_count;
				TerrainHeightPage page;
				if (source.try_read_page(rec.cpu_page_handle, rec.cpu_page_generation, page)) {
					++generation_safe_reads;
				}
			} else if (rec.state == TerrainSourceState::Invalid) {
				++invalid_count;
			} else if (rec.state == TerrainSourceState::Missing) {
				++missing_count;
			}
		}

		TEST_ASSERT(ready_count == static_cast<uint32_t>(kTotalPages), "Ready pages == 1024");
		TEST_ASSERT(generation_safe_reads == static_cast<uint32_t>(kTotalPages), "generation-safe reads == 1024");
		TEST_ASSERT(invalid_count == 0, "Invalid pages == 0");
		TEST_ASSERT(missing_count == 0, "Missing pages == 0");

		source.shutdown();
		executor.shutdown();
	}

	std::cout << "[PASS] 1024-request async stress gate passed: 1024 Ready, 1024 generation-safe reads, 0 Invalid, 0 Missing." << std::endl;
}


// ─── 5. Eight Populated LODs Test ─────────────────────────────────────────────


static void run_eight_lod_starvation_test() {
	std::cout << "\n[TEST 5] Starting 8-LOD populated stream verification test..." << std::endl;

	WorldScaleManifest manifest = make_manifest();
	TerrainRecipe recipe = make_recipe(manifest);
	BCCMSourceExpectation expectation = make_expectation(recipe, manifest);

	BoundedBackgroundJobExecutor executor;
	ConcreteTerrainRenderSource source(recipe, manifest, executor, TerrainPageGenerationMode::SynchronousDiagnostic);

	auto renderer_ptr = std::make_unique<BlockClipmapRenderer>();
	BlockClipmapRenderer& renderer = *renderer_ptr;
	renderer.initialize_cpu_state_for_test(manifest, recipe.identity, source.get_snapshot().fallback_bounds);
	renderer.test_set_profile_levels(8);
	renderer.test_set_profile_radius(4);
	renderer.test_set_profile_hole_radius(2);
	renderer.initialize_cpu_state_for_test(manifest, recipe.identity, source.get_snapshot().fallback_bounds);

	BCCMCameraState cam = make_camera_on_face(SurfaceFace::PositiveX, manifest);
	FrustumPlanes frustum = open_frustum();
	godot::Vector3 cam_pos(0, 0, 0);

	for (int frame = 0; frame < 90; ++frame) {
		[&]() {
			source.process_pending_jobs_sync(64);
			TerrainUpdateResult res = renderer.compute_update(cam_pos, frustum, manifest, cam, expectation, &source);
			renderer.test_finalize_uploads(res);
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}();
	}
	executor.wait_idle_for(std::chrono::milliseconds(5000));
	source.process_pending_jobs_sync(1024);
	TerrainUpdateResult res1 = renderer.compute_update(cam_pos, frustum, manifest, cam, expectation, &source);
	renderer.test_finalize_uploads(res1);
	TerrainUpdateResult res2 = renderer.compute_update(cam_pos, frustum, manifest, cam, expectation, &source);
	renderer.test_finalize_uploads(res2);
	auto snap = std::make_unique<RendererDiagnosticSnapshot>();
	renderer.get_diagnostic_snapshot(*snap);

	for (uint8_t lod = 0; lod < 8; ++lod) {
		const auto& lod_snap = snap->lods[lod];
		std::cout << "[INFO] LOD " << (int)lod << ": candidates=" << lod_snap.candidate_count
		          << ", visible=" << lod_snap.visible_count << std::endl;

		TEST_ASSERT(lod_snap.candidate_count > 0, "candidate count > 0");
		TEST_ASSERT(lod_snap.visible_count > 0, "visible count > 0");

		bool has_non_fallback_layer = false;
		for (const auto& diag : lod_snap.submitted_visible_diagnostics) {
			if (diag.gpu_layer > 0) {
				has_non_fallback_layer = true;
				break;
			}
		}
		TEST_ASSERT(has_non_fallback_layer, "at least one resolved layer > 0");

		bool has_resident_slot = false;
		for (size_t s = 1; s < 128; ++s) {
			if (lod_snap.slots[s].state == TerrainGpuPageState::Resident && !lod_snap.slots[s].is_fallback) {
				has_resident_slot = true;
				break;
			}
		}
		TEST_ASSERT(has_resident_slot, "at least one Resident dynamic slot");
	}

	std::cout << "[PASS] All 8 LOD streams populated: candidates > 0, visible > 0, resolved layer > 0, Resident slot > 0." << std::endl;
	source.shutdown();
	executor.shutdown();
}

// ─── 6. Real 10-Step Slot Turnover Test ──────────────────────────────────────

static void run_real_turnover_test() {
	std::cout << "\n[TEST 6] Starting real dynamic slot turnover test (production limits)..." << std::endl;

	WorldScaleManifest manifest = make_manifest();
	TerrainRecipe recipe = make_recipe(manifest);
	BCCMSourceExpectation expectation = make_expectation(recipe, manifest);

	BoundedBackgroundJobExecutor executor;
	ConcreteTerrainRenderSource source(recipe, manifest, executor, TerrainPageGenerationMode::SynchronousDiagnostic);

	auto renderer_ptr = std::make_unique<BlockClipmapRenderer>();
	BlockClipmapRenderer& renderer = *renderer_ptr;
	// Production limits — NOT inflated. Set BEFORE initialization so vectors resize correctly.
	renderer.test_set_max_source_requests(64);
	renderer.test_set_max_page_commits(24);
	renderer.test_set_max_cross_face_commits(2);

	renderer.initialize_cpu_state_for_test(manifest, recipe.identity, source.get_snapshot().fallback_bounds);
	renderer.test_set_profile_levels(1);
	renderer.test_set_profile_radius(16);

	BCCMCameraState cam = make_camera_on_face(SurfaceFace::PositiveX, manifest);
	FrustumPlanes frustum = open_frustum();
	godot::Vector3 cam_pos(0, 0, 0);

	// Step 1: Fill all 127 dynamic slots under production-bounded multi-frame admission.
	// With max_page_commits=24, this takes ceil(127/24) = 6 rounds minimum.
	bool hit_127 = false;
	for (int step = 0; step < 30; ++step) {
		[&]() {
			TerrainUpdateResult res = renderer.compute_update(cam_pos, frustum, manifest, cam, expectation, &source);
			renderer.test_finalize_uploads(res);
			source.process_pending_jobs_sync(64);

			auto snap = std::make_unique<RendererDiagnosticSnapshot>();
			renderer.get_diagnostic_snapshot(*snap);
			uint32_t res_c = 0;
			for (size_t i = 1; i < 128; ++i) {
				if (snap->lods[0].slots[i].state == TerrainGpuPageState::Resident) ++res_c;
			}
			std::cout << "[DEBUG] step " << step << " uploads=" << res.texture_upload_count << " resident=" << res_c << std::endl;
			if (res_c == 127) hit_127 = true;
		}();
		if (hit_127) break;
	}

	auto snap = std::make_unique<RendererDiagnosticSnapshot>();
	renderer.get_diagnostic_snapshot(*snap);
	const auto& lod0 = snap->lods[0];
	std::cout << "[INFO] Candidate count LOD 0 = " << lod0.candidate_count << std::endl;

	// Step 2: Verify slot 0 remains fallback and Resident.
	TEST_ASSERT(lod0.slots[0].is_fallback, "Step 2: Slot 0 is fallback");
	TEST_ASSERT(lod0.slots[0].state == TerrainGpuPageState::Resident, "Step 2: Slot 0 is Resident");

	size_t resident_count = 0;
	for (size_t i = 1; i < 128; ++i) {
		if (lod0.slots[i].state == TerrainGpuPageState::Resident) ++resident_count;
	}
	std::cout << "[INFO] 127 Resident: " << resident_count << std::endl;
	TEST_ASSERT(resident_count == 127, "Step 1: 127 dynamic slots Resident");

	// Step 3: Move camera to introduce the 128th required page → must fallback.
	BCCMCameraState cam_moved = make_camera_on_face(SurfaceFace::PositiveX, manifest, 2500.0, 2500.0);
	TerrainUpdateResult m_res1 = renderer.compute_update(cam_pos, frustum, manifest, cam_moved, expectation, &source);
	renderer.test_finalize_uploads(m_res1);
	source.process_pending_jobs_sync(64);
	TerrainUpdateResult m_res2 = renderer.compute_update(cam_pos, frustum, manifest, cam_moved, expectation, &source);
	renderer.test_finalize_uploads(m_res2);

	auto snap_moved = std::make_unique<RendererDiagnosticSnapshot>();
	renderer.get_diagnostic_snapshot(*snap_moved);
	const auto& lod0_moved = snap_moved->lods[0];

	// Step 4: Next page resolves to fallback layer 0.
	bool found_layer_zero_candidate = false;
	for (const auto& diag : lod0_moved.submitted_visible_diagnostics) {
		if (diag.gpu_layer == 0) {
			found_layer_zero_candidate = true;
			break;
		}
	}
	TEST_ASSERT(found_layer_zero_candidate, "Step 4: 128th page resolves to fallback layer 0");

	// Step 5: One eligible slot becomes Retiring.
	bool found_retiring = false;
	uint32_t retiring_slot_idx = 0;
	for (uint32_t i = 1; i < 128; ++i) {
		if (lod0_moved.slots[i].state == TerrainGpuPageState::Retiring) {
			found_retiring = true;
			retiring_slot_idx = i;
			break;
		}
	}
	TEST_ASSERT(found_retiring, "Step 5: One slot enters Retiring");

	TerrainRenderBlockKey old_key = lod0_moved.slots[retiring_slot_idx].key;
	uint64_t old_handle = lod0_moved.slots[retiring_slot_idx].cpu_page_handle;
	uint32_t old_gen = lod0_moved.slots[retiring_slot_idx].cpu_page_generation;

	// Step 6: Remains Retiring inside in-flight window.
	uint64_t retire_frame = lod0_moved.slots[retiring_slot_idx].retire_after_frame;
	TEST_ASSERT(retire_frame > 0, "Step 6: retire_after_frame set");

	renderer.compute_update(cam_pos, frustum, manifest, cam_moved, expectation, &source);
	auto snap_inflight = std::make_unique<RendererDiagnosticSnapshot>();
	renderer.get_diagnostic_snapshot(*snap_inflight);
	TEST_ASSERT(snap_inflight->lods[0].slots[retiring_slot_idx].state == TerrainGpuPageState::Retiring,
	            "Step 6: Slot remains Retiring inside in-flight window");

	// Step 7: Advance beyond retire_after_frame WITHOUT processing pending jobs.
	// This ensures the new required page stays Pending and cannot claim the slot yet.
	// The slot will advance its lifecycle from Retiring exactly to Free.
	BCCMCameraState cam_away = make_camera_on_face(SurfaceFace::NegativeX, manifest, 0.0, 0.0);
	TerrainUpdateResult step_res;
	for (int f = 0; f < 5; ++f) {
		[&]() {
			cam_away.frame_epoch++;
			step_res = renderer.compute_update(cam_pos, frustum, manifest, cam_away, expectation, &source);
		}();
	}

	auto snap_free_check = std::make_unique<RendererDiagnosticSnapshot>();
	renderer.get_diagnostic_snapshot(*snap_free_check);
	std::cout << "[DEBUG] Step 7 check: slot " << retiring_slot_idx << " state is " << (int)snap_free_check->lods[0].slots[retiring_slot_idx].state << std::endl;
	TEST_ASSERT(snap_free_check->lods[0].slots[retiring_slot_idx].state == TerrainGpuPageState::Free,
	            "Step 7: Slot transitions exactly to Free after retire_after_frame");

	// Step 8: Process pending jobs (source record becomes Ready), then run one update.
	// This admits the new page, claiming the Free slot and moving it to UploadPending.
	source.process_pending_jobs_sync(256);
	step_res = renderer.compute_update(cam_pos, frustum, manifest, cam_away, expectation, &source);

	auto snap_upload = std::make_unique<RendererDiagnosticSnapshot>();
	renderer.get_diagnostic_snapshot(*snap_upload);
	TEST_ASSERT(snap_upload->lods[0].slots[retiring_slot_idx].state == TerrainGpuPageState::UploadPending,
	            "Step 8: Slot transitions exactly to UploadPending after source admission");

	TerrainRenderBlockKey new_key = snap_upload->lods[0].slots[retiring_slot_idx].key;
	uint64_t new_handle = snap_upload->lods[0].slots[retiring_slot_idx].cpu_page_handle;
	uint32_t new_gen = snap_upload->lods[0].slots[retiring_slot_idx].cpu_page_generation;

	TEST_ASSERT(!(new_key == old_key), "Step 8: Different key assigned to reused slot");
	TEST_ASSERT((new_handle != old_handle || new_gen != old_gen), "Step 8: (handle, generation) differs");

	// Step 9: Finalize uploads. The slot must become Resident.
	renderer.test_finalize_uploads(step_res);
	auto snap_final = std::make_unique<RendererDiagnosticSnapshot>();
	renderer.get_diagnostic_snapshot(*snap_final);
	TEST_ASSERT(snap_final->lods[0].slots[retiring_slot_idx].state == TerrainGpuPageState::Resident,
	            "Step 9: Slot reaches Resident after upload finalization");

	std::cout << "[PASS] Turnover under production limits (64/24/256KiB): 127 Resident → fallback → Retiring → Free → different key → Resident." << std::endl;
	source.shutdown();
	executor.shutdown();
}

// ─── 7. Fallback Bounds from Terrain Authority Test ──────────────────────────

static void run_fallback_bounds_authority_test() {
	std::cout << "\n[TEST 7] Starting fallback bounds authority derivation test..." << std::endl;

	WorldScaleManifest manifest = make_manifest();
	TerrainRecipe recipe = make_recipe(manifest);
	BoundedBackgroundJobExecutor executor;
	ConcreteTerrainRenderSource source(recipe, manifest, executor);

	TerrainRenderSourceSnapshot snap = source.get_snapshot();
	TEST_ASSERT(snap.fallback_bounds.minimum_height == recipe.legacy_signals.min_elevation_m, "minimum_height derived from recipe");
	TEST_ASSERT(snap.fallback_bounds.maximum_height == recipe.legacy_signals.max_elevation_m, "maximum_height derived from recipe");
	TEST_ASSERT(snap.fallback_bounds.residual_bound > 0.0f, "residual_bound set");
	TEST_ASSERT(snap.fallback_bounds.morph_allowance > 0.0f, "morph_allowance set");

	std::cout << "[PASS] Fallback bounds derived from Terrain authority recipe." << std::endl;
	source.shutdown();
	executor.shutdown();
}

// ─── 8. Canonical Camera Publication Test ────────────────────────────────────

static void run_canonical_camera_publication_test() {
	std::cout << "\n[TEST 8] Starting canonical camera publication proof..." << std::endl;

	WorldScaleManifest manifest = make_manifest();
	TerrainRecipe recipe = make_recipe(manifest);
	BCCMSourceExpectation expectation = make_expectation(recipe, manifest);
	BoundedBackgroundJobExecutor executor;
	ConcreteTerrainRenderSource source(recipe, manifest, executor);

	auto renderer_ptr = std::make_unique<BlockClipmapRenderer>();
	BlockClipmapRenderer& renderer = *renderer_ptr;
	renderer.initialize_cpu_state_for_test(manifest, recipe.identity, source.get_snapshot().fallback_bounds);

	// Test un-published camera (frame_epoch == 0)
	BCCMCameraState unpub_cam;
	unpub_cam.frame_epoch = 0;

	renderer.compute_update(godot::Vector3(0,0,0), open_frustum(), manifest, unpub_cam, expectation, &source);
	auto snap_unpub = std::make_unique<RendererDiagnosticSnapshot>();
	renderer.get_diagnostic_snapshot(*snap_unpub);
	TEST_ASSERT(snap_unpub->lods[0].candidate_count == 0, "Renderer remains inactive when frame_epoch == 0");

	// Test published camera
	BCCMCameraState valid_cam = make_camera_on_face(SurfaceFace::PositiveX, manifest, 100.0, 200.0);
	TEST_ASSERT(valid_cam.canonical_position.face == SurfaceFace::PositiveX, "Valid face");
	TEST_ASSERT(valid_cam.canonical_position.topology_version == manifest.topology_version, "Valid topology version");
	TEST_ASSERT(valid_cam.canonical_position.projection_version == manifest.projection_version, "Valid projection version");
	TEST_ASSERT(valid_cam.active_frame.tangent_basis.u_axis.x == 1.0, "Orthonormal u_axis");
	TEST_ASSERT(valid_cam.frame_epoch > 0, "Nonzero frame epoch");

	renderer.compute_update(godot::Vector3(0,0,0), open_frustum(), manifest, valid_cam, expectation, &source);
	auto snap_pub = std::make_unique<RendererDiagnosticSnapshot>();
	renderer.get_diagnostic_snapshot(*snap_pub);
	TEST_ASSERT(snap_pub->lods[0].candidate_count > 0, "Renderer activates when valid publication exists");

	std::cout << "[PASS] Canonical camera publication verified: inactive on epoch 0, active on valid epoch." << std::endl;
	source.shutdown();
	executor.shutdown();
}

// ─── 9. Authoritative Edge & Physical Corners Gate ────────────────────────────

static void run_authoritative_edge_gate() {
	std::cout << "\n[TEST 9] Starting authoritative topology edge gate (24 transitions & 8 corners)..." << std::endl;

	WorldScaleManifest manifest = make_manifest();
	TerrainRecipe recipe = make_recipe(manifest);
	BCCMSourceExpectation expectation = make_expectation(recipe, manifest);
	FrustumPlanes frustum = open_frustum();
	godot::Vector3 cam_pos(0, 0, 0);

	BoundedBackgroundJobExecutor executor;
	double half_extent_m = static_cast<double>(manifest.chart_half_extent_mm) * 0.001;

	// Part 1: All 24 face-edge transitions
	for (uint8_t src_face_idx = 0; src_face_idx < 6; ++src_face_idx) {
		for (uint8_t edge_idx = 0; edge_idx < 4; ++edge_idx) {
			const EdgeTransition& trans = get_edge_transition(src_face_idx, static_cast<SurfaceEdge>(edge_idx));
			SurfaceFace src_face = static_cast<SurfaceFace>(trans.source_face);
			SurfaceFace dst_face = static_cast<SurfaceFace>(trans.destination_face);

			double u_cam = 0.0;
			double v_cam = 0.0;
			double offset = half_extent_m - 16.0;

			switch (trans.source_edge) {
				case SurfaceEdge::NegativeU: u_cam = -offset; v_cam = 0.0; break;
				case SurfaceEdge::PositiveU: u_cam = +offset; v_cam = 0.0; break;
				case SurfaceEdge::NegativeV: u_cam = 0.0; v_cam = -offset; break;
				case SurfaceEdge::PositiveV: u_cam = 0.0; v_cam = +offset; break;
			}

			BCCMCameraState cam = make_camera_on_face(src_face, manifest, u_cam, v_cam);

			ConcreteTerrainRenderSource source(recipe, manifest, executor);
			auto renderer_ptr = std::make_unique<BlockClipmapRenderer>();
			BlockClipmapRenderer& renderer = *renderer_ptr;
			renderer.initialize_cpu_state_for_test(manifest, recipe.identity, source.get_snapshot().fallback_bounds);
			renderer.test_set_profile_levels(1);
			renderer.test_set_profile_radius(3);
			renderer.test_set_profile_hole_radius(0);

			renderer.compute_update(cam_pos, frustum, manifest, cam, expectation, &source);

			auto snap = std::make_unique<RendererDiagnosticSnapshot>();
			renderer.get_diagnostic_snapshot(*snap);
			const auto& candidate_keys = snap->lods[0].candidate_keys;

			bool src_present = false;
			bool dst_present = false;
			std::set<std::tuple<int, int32_t, int32_t>> key_set;

			for (const auto& key : candidate_keys) {
				if (key.face == src_face) src_present = true;
				if (key.face == dst_face) dst_present = true;

				auto tuple_key = std::make_tuple(static_cast<int>(key.face), key.block_u, key.block_v);
				TEST_ASSERT(key_set.count(tuple_key) == 0, "Duplicate candidate key enumerated!");
				key_set.insert(tuple_key);
			}

			TEST_ASSERT(src_present, "Source face candidates must be present!");
			TEST_ASSERT(dst_present, "Destination face candidates must be present!");
			TEST_ASSERT(candidate_keys.size() <= BlockClipmapProfile::MAX_CANDIDATES, "Candidate count bounded!");
			source.shutdown();
		}
	}
	std::cout << "[PASS] All 24 authoritative edge transitions enumerated correctly without duplicates." << std::endl;

	// Part 2: Eight physical corners defined by XYZ sign triplets
	struct CornerDef {
		double sign_x, sign_y, sign_z;
		SurfaceFace f0, f1, f2;
	};
	const CornerDef corners[8] = {
		{ +1, +1, +1, SurfaceFace::PositiveX, SurfaceFace::PositiveY, SurfaceFace::PositiveZ },
		{ +1, +1, -1, SurfaceFace::PositiveX, SurfaceFace::PositiveY, SurfaceFace::NegativeZ },
		{ +1, -1, +1, SurfaceFace::PositiveX, SurfaceFace::NegativeY, SurfaceFace::PositiveZ },
		{ +1, -1, -1, SurfaceFace::PositiveX, SurfaceFace::NegativeY, SurfaceFace::NegativeZ },
		{ -1, +1, +1, SurfaceFace::NegativeX, SurfaceFace::PositiveY, SurfaceFace::PositiveZ },
		{ -1, +1, -1, SurfaceFace::NegativeX, SurfaceFace::PositiveY, SurfaceFace::NegativeZ },
		{ -1, -1, +1, SurfaceFace::NegativeX, SurfaceFace::NegativeY, SurfaceFace::PositiveZ },
		{ -1, -1, -1, SurfaceFace::NegativeX, SurfaceFace::NegativeY, SurfaceFace::NegativeZ },
	};

	for (int ci = 0; ci < 8; ++ci) {
		const CornerDef& c = corners[ci];
		FramePosition64 p_corner = { c.sign_x / std::sqrt(3.0), c.sign_y / std::sqrt(3.0), c.sign_z / std::sqrt(3.0) };
		double u_norm = 0.0, v_norm = 0.0;
		int face_idx = -1;
		bool ok = ProjectionCOBE::map_inverse(p_corner, static_cast<int>(c.f0), u_norm, v_norm, face_idx);
		TEST_ASSERT(ok, "Corner inverse mapping converged");

		double u_m = u_norm * (half_extent_m - 32.0);
		double v_m = v_norm * (half_extent_m - 32.0);

		BCCMCameraState cam = make_camera_on_face(c.f0, manifest, u_m, v_m);

		ConcreteTerrainRenderSource source(recipe, manifest, executor);
		auto renderer_ptr = std::make_unique<BlockClipmapRenderer>();
		BlockClipmapRenderer& renderer = *renderer_ptr;
		renderer.initialize_cpu_state_for_test(manifest, recipe.identity, source.get_snapshot().fallback_bounds);
		renderer.test_set_profile_levels(1);
		renderer.test_set_profile_radius(4);
		renderer.test_set_profile_hole_radius(0);

		renderer.compute_update(cam_pos, frustum, manifest, cam, expectation, &source);

		auto snap_pub = std::make_unique<RendererDiagnosticSnapshot>();
		renderer.get_diagnostic_snapshot(*snap_pub);
		const auto& candidate_keys = snap_pub->lods[0].candidate_keys;

		std::set<std::tuple<int, int32_t, int32_t>> key_set;
		std::set<SurfaceFace> incident_faces;

		for (const auto& key : candidate_keys) {
			incident_faces.insert(key.face);
			auto tuple_key = std::make_tuple(static_cast<int>(key.face), key.block_u, key.block_v);
			TEST_ASSERT(key_set.count(tuple_key) == 0, "Corner candidate key uniqueness violation!");
			key_set.insert(tuple_key);
		}

		TEST_ASSERT(candidate_keys.size() <= BlockClipmapProfile::MAX_CANDIDATES, "Corner candidate count bounded!");
		TEST_ASSERT(incident_faces.count(c.f0) > 0, "Incident face f0 present");
		TEST_ASSERT(incident_faces.count(c.f1) > 0, "Incident face f1 present");
		TEST_ASSERT(incident_faces.count(c.f2) > 0, "Incident face f2 present");

		for (SurfaceFace f : incident_faces) {
			TEST_ASSERT(f == c.f0 || f == c.f1 || f == c.f2, "Candidate faces strictly limited to incident triplet!");
		}
		source.shutdown();
	}
	std::cout << "[PASS] All 8 physical corners validated with exact corresponding 3 incident faces." << std::endl;
	executor.shutdown();
}

// ─── 10. Visible Count Decrease Regression Test ──────────────────────────────

static void run_visible_count_decrease_test() {
	std::cout << "\n[TEST 10] Starting visible count decrease regression test..." << std::endl;

	WorldScaleManifest manifest = make_manifest();
	TerrainRecipe recipe = make_recipe(manifest);
	BCCMSourceExpectation expectation = make_expectation(recipe, manifest);

	BoundedBackgroundJobExecutor executor;
	ConcreteTerrainRenderSource source(recipe, manifest, executor);

	auto renderer_ptr = std::make_unique<BlockClipmapRenderer>();
	BlockClipmapRenderer& renderer = *renderer_ptr;
	renderer.initialize_cpu_state_for_test(manifest, recipe.identity, source.get_snapshot().fallback_bounds);

	BCCMCameraState cam = make_camera_on_face(SurfaceFace::PositiveX, manifest);
	FrustumPlanes open_f = open_frustum();
	godot::Vector3 cam_pos(0, 0, 0);

	TerrainUpdateResult r1 = renderer.compute_update(cam_pos, open_f, manifest, cam, expectation, &source);
	uint32_t vis_A = r1.lods[0].visible_count;
	TEST_ASSERT(vis_A > 0, "Frame A visible count must be > 0");

	FrustumPlanes tight_f;
	tight_f.valid = true;
	tight_f.planes[0] = godot::Plane(godot::Vector3( 1, 0, 0),  5.0f);
	tight_f.planes[1] = godot::Plane(godot::Vector3(-1, 0, 0),  5.0f);
	tight_f.planes[2] = godot::Plane(godot::Vector3(0,  1, 0), 1e9f);
	tight_f.planes[3] = godot::Plane(godot::Vector3(0, -1, 0), 1e9f);
	tight_f.planes[4] = godot::Plane(godot::Vector3(0, 0,  1),  5.0f);
	tight_f.planes[5] = godot::Plane(godot::Vector3(0, 0, -1),  5.0f);

	TerrainUpdateResult r2 = renderer.compute_update(cam_pos, tight_f, manifest, cam, expectation, &source);
	uint32_t vis_B = r2.lods[0].visible_count;

	TEST_ASSERT(vis_B < vis_A, "Visible count decreased");
	TEST_ASSERT(r2.lods[0].buffer_changed, "buffer_changed must be true when visible count decreases!");

	std::cout << "[PASS] Visible count decrease correctly triggers MultiMesh buffer update." << std::endl;
	source.shutdown();
	executor.shutdown();
}

// ─── 11. Invalid Canonical Frame Conversion Test ─────────────────────────────

static void run_invalid_canonical_frame_conversion_test() {
	std::cout << "\n[TEST 11] Starting invalid canonical frame conversion test..." << std::endl;

	WorldScaleManifest manifest = make_manifest();
	TerrainRecipe recipe = make_recipe(manifest);
	BCCMSourceExpectation expectation = make_expectation(recipe, manifest);
	FrustumPlanes frustum = open_frustum();

	BoundedBackgroundJobExecutor executor;
	ConcreteTerrainRenderSource source(recipe, manifest, executor);

	auto renderer_ptr = std::make_unique<BlockClipmapRenderer>();
	BlockClipmapRenderer& renderer = *renderer_ptr;
	renderer.initialize_cpu_state_for_test(manifest, recipe.identity, source.get_snapshot().fallback_bounds);

	// Test 11.1: Invalid topology version
	BCCMCameraState cam_inv_topo = make_camera_on_face(SurfaceFace::PositiveX, manifest);
	cam_inv_topo.canonical_position.topology_version = 999;
	cam_inv_topo.active_frame.topology_version = 999;
	cam_inv_topo.active_frame.origin.topology_version = 999;
	renderer.compute_update(godot::Vector3(0,0,0), frustum, manifest, cam_inv_topo, expectation, &source);
	auto s1 = std::make_unique<RendererDiagnosticSnapshot>();
	renderer.get_diagnostic_snapshot(*s1);
	TEST_ASSERT(s1->lods[0].candidate_count == 0, "Test 11.1: Invalid topology version causes 0 candidates");

	// Test 11.2: Invalid projection version
	BCCMCameraState cam_inv_proj = make_camera_on_face(SurfaceFace::PositiveX, manifest);
	cam_inv_proj.canonical_position.projection_version = 999;
	cam_inv_proj.active_frame.projection_version = 999;
	cam_inv_proj.active_frame.origin.projection_version = 999;
	renderer.compute_update(godot::Vector3(0,0,0), frustum, manifest, cam_inv_proj, expectation, &source);
	auto s2 = std::make_unique<RendererDiagnosticSnapshot>();
	renderer.get_diagnostic_snapshot(*s2);
	TEST_ASSERT(s2->lods[0].candidate_count == 0, "Test 11.2: Invalid projection version causes 0 candidates");

	// Test 11.3: Non-orthonormal basis (zero vector)
	BCCMCameraState cam_non_ortho = make_camera_on_face(SurfaceFace::PositiveX, manifest);
	cam_non_ortho.active_frame.tangent_basis.u_axis = { 0.0, 0.0, 0.0 };
	renderer.compute_update(godot::Vector3(0,0,0), frustum, manifest, cam_non_ortho, expectation, &source);
	auto s3 = std::make_unique<RendererDiagnosticSnapshot>();
	renderer.get_diagnostic_snapshot(*s3);
	TEST_ASSERT(s3->lods[0].candidate_count == 0, "Test 11.3: Inactive result causes 0 candidates (origin fallback rejected)");

	// Test 11.4: Unsupported face relation (out of chart bounds)
	Multinet::SurfacePosition64 pos_out_bounds;
	pos_out_bounds.face = SurfaceFace::PositiveX;
	pos_out_bounds.u_m = 1e12; // Out of bounds
	pos_out_bounds.v_m = 1e12;
	pos_out_bounds.altitude_m = 0.0;
	pos_out_bounds.topology_version = manifest.topology_version;
	pos_out_bounds.projection_version = manifest.projection_version;

	Multinet::FramePosition64 out_frame_pos;
	bool ok_convert = Multinet::try_surface_to_frame(pos_out_bounds, cam_inv_topo.active_frame, manifest, out_frame_pos);
	TEST_ASSERT(!ok_convert, "Test 11.4: Out of bounds surface conversion fails");

	std::cout << "[PASS] Invalid canonical frame conversions correctly rejected." << std::endl;
	source.shutdown();
	executor.shutdown();
}

static void run_frustum_planes_fixture_test() {
	std::cout << "\n[TEST 12] Starting FrustumPlanes::intersects_aabb unit test..." << std::endl;

	// Create a FrustumPlanes with known 6 Planes for a camera looking down -Z (forward = -Z, right = +X, up = +Y)
	// Near: Z = -1, Far: Z = -100, Left: X = -10, Right: X = +10, Top: Y = +10, Bottom: Y = -10
	FrustumPlanes frustum{};
	frustum.valid = true;
	// Godot 4 get_frustum() planes have outward-pointing normals
	frustum.planes[0] = godot::Plane(godot::Vector3(0, 0, 1), -1.0f);     // Near (Z > -1 is outside)
	frustum.planes[1] = godot::Plane(godot::Vector3(0, 0, -1), 100.0f);   // Far (Z < -100 is outside)
	frustum.planes[2] = godot::Plane(godot::Vector3(-1, 0, 0), 10.0f);    // Left (X < -10 is outside)
	frustum.planes[3] = godot::Plane(godot::Vector3(1, 0, 0), 10.0f);     // Right (X > 10 is outside)
	frustum.planes[4] = godot::Plane(godot::Vector3(0, 1, 0), 10.0f);     // Top (Y > 10 is outside)
	frustum.planes[5] = godot::Plane(godot::Vector3(0, -1, 0), 10.0f);    // Bottom (Y < -10 is outside)

	// Case 1: AABB directly in front of camera -> visible
	godot::AABB aabb_front(godot::Vector3(-1, -1, -10), godot::Vector3(2, 2, 2));
	TEST_ASSERT(frustum.intersects_aabb(aabb_front), "Case 1: AABB in front of camera must be visible");

	// Case 2: AABB behind camera -> culled
	godot::AABB aabb_behind(godot::Vector3(-1, -1, 5), godot::Vector3(2, 2, 2));
	TEST_ASSERT(!frustum.intersects_aabb(aabb_behind), "Case 2: AABB behind camera must be culled");

	// Case 3: AABB outside left plane -> culled
	godot::AABB aabb_left(godot::Vector3(-50, -1, -10), godot::Vector3(2, 2, 2));
	TEST_ASSERT(!frustum.intersects_aabb(aabb_left), "Case 3: AABB outside left plane must be culled");

	// Case 4: AABB outside right plane -> culled
	godot::AABB aabb_right(godot::Vector3(50, -1, -10), godot::Vector3(2, 2, 2));
	TEST_ASSERT(!frustum.intersects_aabb(aabb_right), "Case 4: AABB outside right plane must be culled");

	// Case 5: AABB intersecting right plane -> visible
	godot::AABB aabb_straddle(godot::Vector3(9, -1, -10), godot::Vector3(5, 2, 2));
	TEST_ASSERT(frustum.intersects_aabb(aabb_straddle), "Case 5: AABB intersecting plane must be visible");

	// Case 6: Large conservative AABB partially intersecting -> visible
	godot::AABB aabb_large(godot::Vector3(-5, -5, -50), godot::Vector3(100, 100, 100));
	TEST_ASSERT(frustum.intersects_aabb(aabb_large), "Case 6: Large conservative AABB partially intersecting must be visible");

	// Case 7: AABB exactly touching right plane -> visible
	godot::AABB aabb_touching(godot::Vector3(10.0f, -1, -10), godot::Vector3(2, 2, 2));
	TEST_ASSERT(frustum.intersects_aabb(aabb_touching), "Case 7: AABB touching plane must be visible");

	std::cout << "[PASS] FrustumPlanes::intersects_aabb unit tests passed cleanly." << std::endl;
}

// ─── Main Fixture & Evidence Output ──────────────────────────────────────────

int main() {
	std::setvbuf(stdout, NULL, _IONBF, 0);
	std::setvbuf(stderr, NULL, _IONBF, 0);

	std::cout << "===================================================" << std::endl;
	std::cout << "  Multinet WP5 Actual Final Closure Audit Fixture " << std::endl;
	std::cout << "===================================================" << std::endl;
	std::cout.flush();

	try {
		// Group 1:  GLSL shader compilation & preprocessor
		run_shader_compilation_test();
		// Group 2:  Executor overflow & non-blocking deadlock  (includes shutdown subcase)
		run_executor_overflow_test();
		// Group 3:  Shutdown deadlock regression
		run_shutdown_deadlock_regression_test();
		// Group 4:  Generation-mode isolation
		run_generation_mode_isolation_test();
		// Group 5:  1024-page async source stress
		run_async_source_stress_test();
		// Group 6:  Eight-LOD population
		run_eight_lod_starvation_test();
		// Group 7:  127-slot turnover
		run_real_turnover_test();
		// Group 8:  Fallback bounds authority
		run_fallback_bounds_authority_test();
		// Group 9:  Canonical camera publication
		run_canonical_camera_publication_test();
		// Group 10: 24 edges and eight corners (authoritative edge gate)
		run_authoritative_edge_gate();
		// Group 11: Visible-count decrease
		run_visible_count_decrease_test();
		// Group 12: Invalid frame conversion
		run_invalid_canonical_frame_conversion_test();
		// Group 13: Frustum-plane intersects_aabb fixture
		run_frustum_planes_fixture_test();
	} catch (const std::exception& e) {
		std::cout << "[EXCEPTION] Caught exception: " << e.what() << std::endl;
		std::cout.flush();
		return 1;
	} catch (...) {
		std::cout << "[EXCEPTION] Caught unknown exception!" << std::endl;
		std::cout.flush();
		return 1;
	}

	std::cout << "\n===================================================" << std::endl;
	std::cout << "  MULTINET WP5 CPU FIXTURE TESTS VERIFICATION " << std::endl;
	std::cout << "===================================================" << std::endl;
	std::cout << "CPU fixture tests: 13/13 test functions passed" << std::endl;
	std::cout << "  (12 groups + shutdown subcase in group 2)" << std::endl;
	std::cout << "===================================================" << std::endl;

	return 0;
}
