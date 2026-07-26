#ifndef MULTINET_SAVE_SLOT_H
#define MULTINET_SAVE_SLOT_H

#include "schema/binary_schema.h"

#include <cstdint>

namespace Multinet {

struct SaveSlotHeader {
	static constexpr uint32_t EXPECTED_MAGIC = 0x4D4E5356; // 'MNSV'

	uint32_t magic{ EXPECTED_MAGIC };
	uint16_t format_version{ 1 };
	uint32_t slot_id{ 0 };
	uint64_t save_timestamp_ms{ 0 };
	uint64_t canonical_sequence{ 0 };
	uint32_t payload_bytes{ 0 };
};

class SaveSlotSerializer {
public:
	static bool write_slot_header(BinaryWriter &p_writer, const SaveSlotHeader &p_header) noexcept {
		if (!p_writer.write_u32_le(SaveSlotHeader::EXPECTED_MAGIC)) return false;
		if (!p_writer.write_u16_le(p_header.format_version)) return false;
		if (!p_writer.write_u32_le(p_header.slot_id)) return false;
		if (!p_writer.write_u64_le(p_header.save_timestamp_ms)) return false;
		if (!p_writer.write_u64_le(p_header.canonical_sequence)) return false;
		if (!p_writer.write_u32_le(p_header.payload_bytes)) return false;
		return true;
	}

	static bool read_slot_header(BinaryReader &p_reader, SaveSlotHeader &r_header) noexcept {
		if (!p_reader.read_u32_le(r_header.magic) || r_header.magic != SaveSlotHeader::EXPECTED_MAGIC) return false;
		if (!p_reader.read_u16_le(r_header.format_version)) return false;
		if (!p_reader.read_u32_le(r_header.slot_id)) return false;
		if (!p_reader.read_u64_le(r_header.save_timestamp_ms)) return false;
		if (!p_reader.read_u64_le(r_header.canonical_sequence)) return false;
		if (!p_reader.read_u32_le(r_header.payload_bytes)) return false;
		return true;
	}
};

} // namespace Multinet

#endif // MULTINET_SAVE_SLOT_H
