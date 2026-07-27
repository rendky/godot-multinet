#ifndef MULTINET_BINARY_SCHEMA_H
#define MULTINET_BINARY_SCHEMA_H

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace Multinet {

struct SchemaHeader {
	uint32_t magic{ 0x4D4E4554 }; // 'MNET'
	uint16_t version{ 1 };
	uint16_t payload_type{ 0 };
	uint32_t payload_size{ 0 };
	uint32_t checksum{ 0 };

	static constexpr uint32_t EXPECTED_MAGIC = 0x4D4E4554;
	static constexpr uint32_t MAX_PAYLOAD_SIZE = 1048576; // 1 MB cap for M0
};

class BinaryWriter {
private:
	uint8_t *buffer{ nullptr };
	size_t capacity{ 0 };
	size_t offset{ 0 };

public:
	BinaryWriter(uint8_t *p_buffer, size_t p_capacity)
		: buffer(p_buffer), capacity(p_capacity) {}

	bool write_u8(uint8_t p_val) noexcept {
		if (offset + 1 > capacity) return false;
		buffer[offset++] = p_val;
		return true;
	}

	bool write_u16_le(uint16_t p_val) noexcept {
		if (offset + 2 > capacity) return false;
		buffer[offset++] = static_cast<uint8_t>(p_val & 0xFF);
		buffer[offset++] = static_cast<uint8_t>((p_val >> 8) & 0xFF);
		return true;
	}

	bool write_u32_le(uint32_t p_val) noexcept {
		if (offset + 4 > capacity) return false;
		buffer[offset++] = static_cast<uint8_t>(p_val & 0xFF);
		buffer[offset++] = static_cast<uint8_t>((p_val >> 8) & 0xFF);
		buffer[offset++] = static_cast<uint8_t>((p_val >> 16) & 0xFF);
		buffer[offset++] = static_cast<uint8_t>((p_val >> 24) & 0xFF);
		return true;
	}

	bool write_u64_le(uint64_t p_val) noexcept {
		if (offset + 8 > capacity) return false;
		buffer[offset++] = static_cast<uint8_t>(p_val & 0xFF);
		buffer[offset++] = static_cast<uint8_t>((p_val >> 8) & 0xFF);
		buffer[offset++] = static_cast<uint8_t>((p_val >> 16) & 0xFF);
		buffer[offset++] = static_cast<uint8_t>((p_val >> 24) & 0xFF);
		buffer[offset++] = static_cast<uint8_t>((p_val >> 32) & 0xFF);
		buffer[offset++] = static_cast<uint8_t>((p_val >> 40) & 0xFF);
		buffer[offset++] = static_cast<uint8_t>((p_val >> 48) & 0xFF);
		buffer[offset++] = static_cast<uint8_t>((p_val >> 56) & 0xFF);
		return true;
	}

	bool write_f32_le(float p_val) noexcept {
		static_assert(sizeof(float) == 4, "float must be 32-bit");
		uint32_t bits = 0;
		std::memcpy(&bits, &p_val, 4);
		return write_u32_le(bits);
	}

	[[nodiscard]] size_t get_offset() const noexcept { return offset; }
};

class BinaryReader {
private:
	const uint8_t *buffer{ nullptr };
	size_t capacity{ 0 };
	size_t offset{ 0 };

public:
	BinaryReader(const uint8_t *p_buffer, size_t p_capacity)
		: buffer(p_buffer), capacity(p_capacity) {}

	bool read_u8(uint8_t &r_val) noexcept {
		if (offset + 1 > capacity) return false;
		r_val = buffer[offset++];
		return true;
	}

	bool read_u16_le(uint16_t &r_val) noexcept {
		if (offset + 2 > capacity) return false;
		r_val = static_cast<uint16_t>(buffer[offset]) |
		        (static_cast<uint16_t>(buffer[offset + 1]) << 8);
		offset += 2;
		return true;
	}

	bool read_u32_le(uint32_t &r_val) noexcept {
		if (offset + 4 > capacity) return false;
		r_val = static_cast<uint32_t>(buffer[offset]) |
		        (static_cast<uint32_t>(buffer[offset + 1]) << 8) |
		        (static_cast<uint32_t>(buffer[offset + 2]) << 16) |
		        (static_cast<uint32_t>(buffer[offset + 3]) << 24);
		offset += 4;
		return true;
	}

	bool read_u64_le(uint64_t &r_val) noexcept {
		if (offset + 8 > capacity) return false;
		r_val = static_cast<uint64_t>(buffer[offset]) |
		        (static_cast<uint64_t>(buffer[offset + 1]) << 8) |
		        (static_cast<uint64_t>(buffer[offset + 2]) << 16) |
		        (static_cast<uint64_t>(buffer[offset + 3]) << 24) |
		        (static_cast<uint64_t>(buffer[offset + 4]) << 32) |
		        (static_cast<uint64_t>(buffer[offset + 5]) << 40) |
		        (static_cast<uint64_t>(buffer[offset + 6]) << 48) |
		        (static_cast<uint64_t>(buffer[offset + 7]) << 56);
		offset += 8;
		return true;
	}

	bool read_f32_le(float &r_val) noexcept {
		uint32_t bits = 0;
		if (!read_u32_le(bits)) return false;
		std::memcpy(&r_val, &bits, 4);
		return true;
	}

	bool validate_header(SchemaHeader &r_header) noexcept {
		if (!read_u32_le(r_header.magic) || r_header.magic != SchemaHeader::EXPECTED_MAGIC) {
			return false; // Magic mismatch
		}
		if (!read_u16_le(r_header.version) || r_header.version != 1) {
			return false; // Version mismatch
		}
		if (!read_u16_le(r_header.payload_type)) return false;
		if (!read_u32_le(r_header.payload_size) || r_header.payload_size > SchemaHeader::MAX_PAYLOAD_SIZE) {
			return false; // Malformed payload cap exceeded
		}
		if (!read_u32_le(r_header.checksum)) return false;
		return true;
	}

	[[nodiscard]] size_t get_offset() const noexcept { return offset; }
	[[nodiscard]] size_t get_remaining() const noexcept { return capacity - offset; }
};

} // namespace Multinet

#endif // MULTINET_BINARY_SCHEMA_H
