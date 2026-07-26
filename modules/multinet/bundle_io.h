#ifndef MULTINET_BUNDLE_IO_H
#define MULTINET_BUNDLE_IO_H

#include "arena_allocator.h"
#include "binary_schema.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace Multinet {

struct BundleHeader {
	uint32_t magic{ 0x4D4E424C }; // 'MNBL'
	uint16_t version{ 1 };
	uint16_t compression_type{ 0 }; // 0 = Uncompressed / RAW, 1 = Staged
	uint32_t block_count{ 0 };
	uint32_t total_size{ 0 };

	static constexpr uint32_t EXPECTED_MAGIC = 0x4D4E424C;
};

struct BundleBlockEntry {
	uint64_t file_offset{ 0 };
	uint32_t compressed_size{ 0 };
	uint32_t uncompressed_size{ 0 };
	uint32_t cache_key{ 0 };
};

class LocalityBundleReader {
private:
	BundleHeader header{};
	std::vector<BundleBlockEntry> block_table;

public:
	LocalityBundleReader() = default;

	bool parse_header_and_index(BinaryReader &p_reader) noexcept {
		if (!p_reader.read_u32_le(header.magic) || header.magic != BundleHeader::EXPECTED_MAGIC) {
			return false; // Magic bytes mismatch
		}
		if (!p_reader.read_u16_le(header.version) || header.version != 1) {
			return false; // Version mismatch
		}
		if (!p_reader.read_u16_le(header.compression_type)) return false;
		if (!p_reader.read_u32_le(header.block_count) || header.block_count > 1024) {
			return false; // Invalid or excessive block count cap
		}
		if (!p_reader.read_u32_le(header.total_size)) return false;

		block_table.resize(header.block_count);
		for (uint32_t i = 0; i < header.block_count; ++i) {
			BundleBlockEntry &entry = block_table[i];
			uint32_t low_off = 0, high_off = 0;

			if (!p_reader.read_u32_le(low_off)) return false;
			if (!p_reader.read_u32_le(high_off)) return false;
			entry.file_offset = (static_cast<uint64_t>(high_off) << 32) | low_off;

			if (!p_reader.read_u32_le(entry.compressed_size)) return false;
			if (!p_reader.read_u32_le(entry.uncompressed_size)) return false;
			if (!p_reader.read_u32_le(entry.cache_key)) return false;
		}

		return true;
	}

	[[nodiscard]] const BundleHeader &get_header() const noexcept { return header; }
	[[nodiscard]] const std::vector<BundleBlockEntry> &get_block_table() const noexcept { return block_table; }

	// Staged decompression allocation directly inside linear arena
	uint8_t *stage_decompression_buffer(ArenaAllocator &p_arena, uint32_t p_uncompressed_size) noexcept {
		return p_arena.allocate<uint8_t>(p_uncompressed_size);
	}
};

} // namespace Multinet

#endif // MULTINET_BUNDLE_IO_H
