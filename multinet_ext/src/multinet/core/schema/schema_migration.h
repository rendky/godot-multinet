#ifndef MULTINET_SCHEMA_MIGRATION_H
#define MULTINET_SCHEMA_MIGRATION_H

#include "multinet/core/schema/binary_schema.h"

#include <cstddef>
#include <cstdint>
#include <functional>

namespace Multinet {

using MigrationStepFunc = std::function<bool(BinaryReader &r_reader, BinaryWriter &r_writer)>;

class SchemaMigrator {
public:
	static constexpr uint16_t CURRENT_SUPPORTED_VERSION = 2;

	static bool migrate_v1_to_v2(BinaryReader &p_reader, BinaryWriter &p_writer) noexcept {
		// Legacy V1 Payload format: u64 entity_id, float x, float y
		// Target V2 Payload format: u64 entity_id, float x, float y, float z (added z=0.0f default)
		uint64_t entity_id = 0;
		float x = 0.0f;
		float y = 0.0f;

		if (!p_reader.read_u32_le(reinterpret_cast<uint32_t &>(entity_id))) return false; // low 32
		if (!p_reader.read_u32_le(*(reinterpret_cast<uint32_t *>(&entity_id) + 1))) return false; // high 32
		if (!p_reader.read_f32_le(x)) return false;
		if (!p_reader.read_f32_le(y)) return false;

		// Write V2 migrated format
		if (!p_writer.write_u32_le(static_cast<uint32_t>(entity_id & 0xFFFFFFFF))) return false;
		if (!p_writer.write_u32_le(static_cast<uint32_t>((entity_id >> 32) & 0xFFFFFFFF))) return false;
		if (!p_writer.write_f32_le(x)) return false;
		if (!p_writer.write_f32_le(y)) return false;
		if (!p_writer.write_f32_le(0.0f)) return false; // Default z migration value

		return true;
	}

	static bool migrate_payload(
			uint16_t p_source_version,
			uint16_t p_target_version,
			const uint8_t *p_src_buffer,
			size_t p_src_size,
			uint8_t *p_dst_buffer,
			size_t p_dst_capacity,
			size_t &r_dst_written) noexcept {
		// Reject unsupported or future schema versions explicitly
		if (p_source_version > CURRENT_SUPPORTED_VERSION || p_target_version > CURRENT_SUPPORTED_VERSION) {
			return false; // Rejected: Future/Unsupported version
		}

		if (p_source_version == p_target_version) {
			if (p_dst_capacity < p_src_size) return false;
			std::memcpy(p_dst_buffer, p_src_buffer, p_src_size);
			r_dst_written = p_src_size;
			return true;
		}

		if (p_source_version == 1 && p_target_version == 2) {
			BinaryReader reader(p_src_buffer, p_src_size);
			BinaryWriter writer(p_dst_buffer, p_dst_capacity);

			if (!migrate_v1_to_v2(reader, writer)) {
				return false;
			}
			r_dst_written = writer.get_offset();
			return true;
		}

		return false;
	}
};

} // namespace Multinet

#endif // MULTINET_SCHEMA_MIGRATION_H
