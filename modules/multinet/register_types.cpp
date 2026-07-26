#include "register_types.h"

#include "arena_allocator.h"
#include "binary_schema.h"
#include "bounded_pool.h"
#include "generation_handle.h"
#include "job_system.h"
#include "ownership_ledger.h"
#include "snapshot_publisher.h"

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
			fuzz_reader.read_u32_le(v1);
			fuzz_reader.read_u32_le(v2);
		}
	}

	return true;
}

static bool run_core_debug_01_verification() {
	ArenaAllocator arena(512);
	arena.allocate<uint8_t>(128);

	BoundedPool<TestParticle, 10> pool;
	pool.allocate(TestParticle{ 1.0f, 1.0f, 1 });

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

	if (mem_pass && job_pass && thread_pass && schema_pass && fuzz_pass && debug_pass) {
		print_line("[multinet] MILESTONE M0 ALL EXIT GATES PASSED (BUILD, BASE, MEM, JOB, THREAD, SCHEMA, FUZZ, DEBUG OK).");
	} else {
		print_error("[multinet] Verification FAILED!");
	}
}

void uninitialize_multinet_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SERVERS) {
		return;
	}
}
