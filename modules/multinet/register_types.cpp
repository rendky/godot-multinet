#include "register_types.h"

#include "core/coordinates.h"
#include "debug/ownership_ledger.h"
#include "events/typed_events.h"
#include "io/bundle_io.h"
#include "jobs/job_system.h"
#include "memory/arena_allocator.h"
#include "memory/bounded_pool.h"
#include "memory/generation_handle.h"
#include "quality/quality_authority.h"
#include "replication/net_latejoin.h"
#include "replication/net_reconciliation.h"
#include "schema/binary_schema.h"
#include "schema/schema_migration.h"
#include "spatial/net_interest.h"
#include "thread/snapshot_publisher.h"

#include "core/config/engine.h"
#include "core/os/os.h"

namespace Multinet {

struct TestParticle {
	float x{ 0.0f };
	float y{ 0.0f };
	uint32_t id{ 0 };
};

struct WorldStateSnapshot {
	uint32_t active_entities{ 0 };
	float time_step{ 0.0f };
};

static bool run_core_mem_01_verification() {
	ArenaAllocator arena(1024);
	TestParticle *p1 = arena.allocate<TestParticle>(10);
	if (!p1 || arena.get_used() < sizeof(TestParticle) * 10) return false;
	arena.reset();
	if (arena.get_used() != 0) return false;

	BoundedPool<TestParticle, 16> pool;
	GenerationHandle h1 = pool.allocate(TestParticle{ 1.0f, 2.0f, 100 });
	if (!h1.is_valid()) return false;

	TestParticle *retrieved = pool.get(h1);
	if (!retrieved || retrieved->id != 100) return false;

	if (!pool.free(h1)) return false;
	if (pool.get(h1) != nullptr) return false;

	BoundedPool<int, 2> small_pool;
	GenerationHandle s1 = small_pool.allocate(10);
	GenerationHandle s2 = small_pool.allocate(20);
	GenerationHandle s3 = small_pool.allocate(30);

	if (!s1.is_valid() || !s2.is_valid()) return false;
	if (s3.is_valid()) return false;
	if (!small_pool.has_overflowed()) return false;

	return true;
}

static bool run_core_job_01_verification() {
	BoundedJobQueue<8> job_queue;

	bool executed_1 = false;
	if (!job_queue.enqueue(JobPriority::HIGH, [&executed_1]() { executed_1 = true; })) return false;
	if (!job_queue.dequeue_and_execute() || !executed_1) return false;

	bool executed_cancelled = false;
	JobToken cancel_token;
	cancel_token.cancel();

	if (!job_queue.enqueue(JobPriority::NORMAL, [&executed_cancelled]() { executed_cancelled = true; }, &cancel_token)) return false;
	job_queue.dequeue_and_execute();
	if (executed_cancelled) return false;

	BoundedJobQueue<2> small_queue;
	small_queue.enqueue(JobPriority::LOW, []() {});
	small_queue.enqueue(JobPriority::LOW, []() {});
	bool overflow_enqueue = small_queue.enqueue(JobPriority::LOW, []() {});

	if (overflow_enqueue) return false;
	if (!small_queue.has_overflowed()) return false;

	return true;
}

static bool run_core_thread_01_verification() {
	SnapshotPublisher<WorldStateSnapshot> publisher(WorldStateSnapshot{ 10, 0.016f });

	if (publisher.get_read_snapshot().active_entities != 10 || publisher.get_version() != 0) return false;

	WorldStateSnapshot &back = publisher.get_back_buffer();
	back.active_entities = 42;
	back.time_step = 0.033f;

	if (publisher.get_read_snapshot().active_entities != 10) return false;

	uint64_t v1 = publisher.publish();
	if (v1 != 1 || publisher.get_version() != 1) return false;

	const auto &s1 = publisher.get_read_snapshot();
	if (s1.active_entities != 42 || s1.time_step != 0.033f) return false;

	return true;
}

static bool run_core_schema_01_verification() {
	uint8_t buffer[128]{};

	BinaryWriter writer(buffer, sizeof(buffer));
	if (!writer.write_u32_le(SchemaHeader::EXPECTED_MAGIC)) return false;
	if (!writer.write_u16_le(1)) return false;
	if (!writer.write_u16_le(0x0102)) return false;
	if (!writer.write_u32_le(16)) return false;
	if (!writer.write_u32_le(0xDEADBEEF)) return false;

	if (!writer.write_u8(0xFF)) return false;
	if (!writer.write_u16_le(0x1234)) return false;
	if (!writer.write_u32_le(0x87654321)) return false;
	if (!writer.write_f32_le(3.14159f)) return false;

	BinaryReader reader(buffer, writer.get_offset());
	SchemaHeader header{};
	if (!reader.validate_header(header)) return false;

	if (header.magic != SchemaHeader::EXPECTED_MAGIC || header.version != 1 || header.payload_size != 16) {
		return false;
	}

	uint8_t v8 = 0;
	uint16_t v16 = 0;
	uint32_t v32 = 0;
	float vf32 = 0.0f;

	if (!reader.read_u8(v8) || v8 != 0xFF) return false;
	if (!reader.read_u16_le(v16) || v16 != 0x1234) return false;
	if (!reader.read_u32_le(v32) || v32 != 0x87654321) return false;
	if (!reader.read_f32_le(vf32) || (vf32 < 3.14f || vf32 > 3.15f)) return false;

	uint8_t bad_buffer[32]{};
	BinaryWriter bad_writer(bad_buffer, sizeof(bad_buffer));
	bad_writer.write_u32_le(SchemaHeader::EXPECTED_MAGIC);
	bad_writer.write_u16_le(1);
	bad_writer.write_u16_le(0);
	bad_writer.write_u32_le(99999999);

	BinaryReader bad_reader(bad_buffer, bad_writer.get_offset());
	SchemaHeader bad_header{};
	if (bad_reader.validate_header(bad_header)) {
		return false;
	}

	return true;
}

static bool run_core_resource_fuzz_01_verification() {
	uint8_t short_buffer[6]{ 0x54, 0x45, 0x4E, 0x4D, 0x01, 0x00 };
	BinaryReader truncated_reader(short_buffer, sizeof(short_buffer));
	SchemaHeader header{};
	if (truncated_reader.validate_header(header)) return false;

	uint8_t valid_buffer[64]{};
	BinaryWriter writer(valid_buffer, sizeof(valid_buffer));
	writer.write_u32_le(SchemaHeader::EXPECTED_MAGIC);
	writer.write_u16_le(1);
	writer.write_u16_le(0x0001);
	writer.write_u32_le(8);
	writer.write_u32_le(0x12345678);
	writer.write_u32_le(100);
	writer.write_u32_le(200);

	for (uint32_t seed = 1; seed <= 100; ++seed) {
		uint8_t fuzzed[64];
		std::memcpy(fuzzed, valid_buffer, sizeof(valid_buffer));

		size_t corrupt_pos = seed % sizeof(valid_buffer);
		fuzzed[corrupt_pos] ^= static_cast<uint8_t>(seed & 0xFF);

		BinaryReader fuzz_reader(fuzzed, sizeof(fuzzed));
		SchemaHeader fuzzed_header{};
		uint32_t v1 = 0, v2 = 0;

		if (fuzz_reader.validate_header(fuzzed_header)) {
			bool r1 = fuzz_reader.read_u32_le(v1);
			bool r2 = fuzz_reader.read_u32_le(v2);
			(void)r1;
			(void)r2;
		}
	}

	return true;
}

static bool run_core_debug_01_verification() {
	ArenaAllocator arena(512);
	uint8_t *dummy_mem = arena.allocate<uint8_t>(128);
	(void)dummy_mem;

	BoundedPool<TestParticle, 10> pool;
	GenerationHandle dummy_h = pool.allocate(TestParticle{ 1.0f, 1.0f, 1 });
	(void)dummy_h;

	BoundedJobQueue<16> jobs;
	jobs.enqueue(JobPriority::HIGH, []() {});

	SnapshotPublisher<WorldStateSnapshot> pub;
	pub.publish();

	SubsystemResourceReport report = OwnershipLedger::generate_report(
			arena.get_capacity(),
			arena.get_used(),
			pool.get_capacity(),
			pool.get_active_count(),
			jobs.get_count(),
			pub.get_version(),
			pool.has_overflowed());

	if (report.arena_capacity_bytes != 512 || report.arena_used_bytes < 128) return false;
	if (report.pool_capacity_items != 10 || report.pool_active_items != 1) return false;
	if (report.job_queue_count != 1 || report.snapshot_version != 1) return false;

	return true;
}

static bool run_core_coord_01_verification() {
	WorldPosition64 w1{ 123456.789, 500.0, 98765.432 };
	RegionPosition r1 = RegionPosition::from_world(w1);
	WorldPosition64 w1_back = r1.to_world();

	if (std::abs(w1.x - w1_back.x) > 0.001 ||
			std::abs(w1.y - w1_back.y) > 0.001 ||
			std::abs(w1.z - w1_back.z) > 0.001) {
		return false;
	}

	WorldPosition64 w2{ -1500.0, -2500.0, -50.0 };
	RegionPosition r2 = RegionPosition::from_world(w2);
	WorldPosition64 w2_back = r2.to_world();

	if (r2.local_x < 0.0f || r2.local_x >= 1024.0f ||
			r2.local_y < 0.0f || r2.local_y >= 1024.0f ||
			r2.local_z < 0.0f || r2.local_z >= 1024.0f) {
		return false;
	}

	if (std::abs(w2.x - w2_back.x) > 0.001 ||
			std::abs(w2.y - w2_back.y) > 0.001 ||
			std::abs(w2.z - w2_back.z) > 0.001) {
		return false;
	}

	return true;
}

static bool run_core_event_01_verification() {
	EventDeduplicator<16> deduplicator;
	uint64_t canonical_event_id = 9988776655;

	if (deduplicator.is_duplicate(canonical_event_id)) return false;
	deduplicator.record(canonical_event_id);
	if (!deduplicator.is_duplicate(canonical_event_id)) return false;

	BoundedEventQueue<int, 4> event_queue;
	EventHeader h1{ 101, 1, 1, EventCategory::CANONICAL };
	EventHeader h2{ 102, 2, 1, EventCategory::SIMULATION };

	if (!event_queue.push(h1, 42)) return false;
	if (!event_queue.push(h2, 84)) return false;

	EventHeader pop_h1{}, pop_h2{};
	int payload1 = 0, payload2 = 0;

	if (!event_queue.pop(pop_h1, payload1) || payload1 != 42 || pop_h1.category != EventCategory::CANONICAL) {
		return false;
	}
	if (!event_queue.pop(pop_h2, payload2) || payload2 != 84 || pop_h2.category != EventCategory::SIMULATION) {
		return false;
	}

	BoundedEventQueue<int, 2> small_queue;
	small_queue.push(h1, 1);
	small_queue.push(h2, 2);
	bool overflow_push = small_queue.push(h1, 3);

	if (overflow_push) return false;
	if (!small_queue.has_overflowed()) return false;

	return true;
}

static bool run_core_quality_01_verification() {
	QualityAuthorityManager mgr(QualityTier::HIGH, 2000);

	if (mgr.get_active_tier() != QualityTier::HIGH) return false;

	if (!mgr.request_tier_change(QualityTier::LOW, 1000)) return false;
	if (mgr.get_active_tier() != QualityTier::LOW) return false;

	if (mgr.request_tier_change(QualityTier::HIGH, 1500)) return false;
	if (mgr.get_active_tier() != QualityTier::LOW) return false;

	if (!mgr.request_tier_change(QualityTier::HIGH, 3500)) return false;
	if (mgr.get_active_tier() != QualityTier::HIGH) return false;

	return true;
}

static bool run_core_resource_migrate_01_verification() {
	uint8_t v1_buffer[32]{};
	BinaryWriter writer(v1_buffer, sizeof(v1_buffer));
	writer.write_u32_le(0x55667788);
	writer.write_u32_le(0x11223344);
	writer.write_f32_le(10.0f);
	writer.write_f32_le(20.0f);

	uint8_t v2_buffer[64]{};
	size_t written_size = 0;
	if (!SchemaMigrator::migrate_payload(1, 2, v1_buffer, writer.get_offset(), v2_buffer, sizeof(v2_buffer), written_size)) {
		return false;
	}

	BinaryReader reader(v2_buffer, written_size);
	uint32_t low_id = 0, high_id = 0;
	float x = 0.0f, y = 0.0f, z = -1.0f;

	if (!reader.read_u32_le(low_id) || low_id != 0x55667788) return false;
	if (!reader.read_u32_le(high_id) || high_id != 0x11223344) return false;
	if (!reader.read_f32_le(x) || x != 10.0f) return false;
	if (!reader.read_f32_le(y) || y != 20.0f) return false;
	if (!reader.read_f32_le(z) || z != 0.0f) return false;

	size_t dummy_written = 0;
	if (SchemaMigrator::migrate_payload(999, 2, v1_buffer, sizeof(v1_buffer), v2_buffer, sizeof(v2_buffer), dummy_written)) {
		return false;
	}

	return true;
}

static bool run_io_bundle_hdd_01_verification() {
	uint8_t bundle_buffer[256]{};
	BinaryWriter writer(bundle_buffer, sizeof(bundle_buffer));
	writer.write_u32_le(BundleHeader::EXPECTED_MAGIC);
	writer.write_u16_le(1);
	writer.write_u16_le(0);
	writer.write_u32_le(1);
	writer.write_u32_le(128);

	writer.write_u32_le(0x00000040);
	writer.write_u32_le(0x00000000);
	writer.write_u32_le(64);
	writer.write_u32_le(128);
	writer.write_u32_le(0xAABBCCDD);

	BinaryReader reader(bundle_buffer, writer.get_offset());
	LocalityBundleReader bundle_reader;
	if (!bundle_reader.parse_header_and_index(reader)) return false;

	if (bundle_reader.get_header().block_count != 1) return false;
	if (bundle_reader.get_block_table().empty()) return false;

	const auto &entry = bundle_reader.get_block_table()[0];
	if (entry.file_offset != 64 || entry.cache_key != 0xAABBCCDD) return false;

	ArenaAllocator arena(512);
	uint8_t *staged_buf = bundle_reader.stage_decompression_buffer(arena, entry.uncompressed_size);
	if (!staged_buf || arena.get_used() < 128) return false;

	return true;
}

static bool run_net_recon_01_verification() {
	ClientReconciler reconciler;

	TransformSnapshotPacket pkt1{};
	pkt1.sequence_num = 1;
	pkt1.timestamp_ms = 1000;
	pkt1.world_pos = WorldPosition64{ 100.0, 0.0, 0.0 };

	if (!reconciler.process_server_snapshot(pkt1)) return false;
	if (reconciler.get_last_sequence() != 1) return false;

	TransformSnapshotPacket pkt_dup = pkt1;
	TransformSnapshotPacket pkt_old{};
	pkt_old.sequence_num = 0;

	if (reconciler.process_server_snapshot(pkt_dup)) return false;
	if (reconciler.process_server_snapshot(pkt_old)) return false;

	WorldPosition64 predicted_pos{ 105.0, 0.0, 0.0 };
	double err = reconciler.calculate_error_distance(predicted_pos);
	if (std::abs(err - 5.0) > 0.001) return false;

	WorldPosition64 client_pos{ 105.0, 0.0, 0.0 };
	WorldPosition64 corrected = reconciler.apply_smooth_correction(client_pos, 0.5f);
	if (std::abs(corrected.x - 102.5) > 0.001) return false;

	return true;
}

static bool run_net_tier_01_verification() {
	SpatialInterestGrid grid;
	PlayerID player1 = 101;
	SessionID session1 = 9001;
	RegionPosition p_pos{ 10, 10, 0, 0.0f, 0.0f, 0.0f };

	if (!grid.register_player(player1, session1, p_pos, 1)) return false;
	if (grid.get_active_player_count() != 1) return false;

	RegionPosition entity_inside{ 11, 10, 0, 0.0f, 0.0f, 0.0f };
	if (!grid.is_in_interest_range(player1, entity_inside)) return false;

	RegionPosition entity_outside{ 15, 10, 0, 0.0f, 0.0f, 0.0f };
	if (grid.is_in_interest_range(player1, entity_outside)) return false;

	RegionPosition p_new_pos{ 15, 10, 0, 0.0f, 0.0f, 0.0f };
	if (!grid.update_player_position(player1, p_new_pos)) return false;

	if (!grid.is_in_interest_range(player1, entity_outside)) return false;
	if (grid.is_in_interest_range(player1, p_pos)) return false;

	if (!grid.unregister_player(player1)) return false;
	if (grid.get_active_player_count() != 0) return false;

	return true;
}

static bool run_net_late_01_verification() {
	LateJoinManager late_join;
	PlayerID player1 = 202;
	SessionID session1 = 8001;
	SessionToken token1 = 0xABCD1234;

	if (!late_join.register_new_session(player1, session1, token1)) return false;
	late_join.record_canonical_event(501);
	late_join.record_canonical_event(502);

	if (!late_join.handle_disconnect(player1)) return false;

	uint64_t recovered_seq = 0;
	if (late_join.attempt_reconnect(player1, 0xBAD10000, recovered_seq)) return false;

	if (!late_join.attempt_reconnect(player1, token1, recovered_seq)) return false;

	std::vector<uint64_t> missed_events;
	size_t count = late_join.get_missed_events_for_reconnect(0, missed_events);
	if (count != 2 || missed_events.size() != 2) return false;
	if (missed_events[0] != 501 || missed_events[1] != 502) return false;

	return true;
}

} // namespace Multinet

void initialize_multinet_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SERVERS) {
		return;
	}

	bool mem_pass = Multinet::run_core_mem_01_verification();
	bool job_pass = Multinet::run_core_job_01_verification();
	bool thread_pass = Multinet::run_core_thread_01_verification();
	bool schema_pass = Multinet::run_core_schema_01_verification();
	bool fuzz_pass = Multinet::run_core_resource_fuzz_01_verification();
	bool debug_pass = Multinet::run_core_debug_01_verification();
	bool coord_pass = Multinet::run_core_coord_01_verification();
	bool event_pass = Multinet::run_core_event_01_verification();
	bool quality_pass = Multinet::run_core_quality_01_verification();
	bool migrate_pass = Multinet::run_core_resource_migrate_01_verification();
	bool bundle_pass = Multinet::run_io_bundle_hdd_01_verification();
	bool recon_pass = Multinet::run_net_recon_01_verification();
	bool tier_pass = Multinet::run_net_tier_01_verification();
	bool late_pass = Multinet::run_net_late_01_verification();

	if (mem_pass && job_pass && thread_pass && schema_pass && fuzz_pass && debug_pass && coord_pass && event_pass && quality_pass && migrate_pass && bundle_pass && recon_pass && tier_pass && late_pass) {
		print_line("[multinet] NET-LATE-01 Verified OK (Late-Join Handshake & Disconnect Reconnect Recovery).");
	} else {
		print_error("[multinet] Verification FAILED!");
	}
}

void uninitialize_multinet_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SERVERS) {
		return;
	}
}
