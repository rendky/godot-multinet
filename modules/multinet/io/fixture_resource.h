#ifndef MULTINET_FIXTURE_RESOURCE_H
#define MULTINET_FIXTURE_RESOURCE_H

#include "schema/binary_schema.h"

#include <cstdint>

namespace Multinet {

struct FixtureBundleHeader {
	static constexpr uint32_t EXPECTED_MAGIC = 0x4D4E4658; // 'MNFX'

	uint32_t magic{ EXPECTED_MAGIC };
	uint16_t version{ 1 };
	uint32_t fixture_id{ 0 };
	uint32_t payload_bytes{ 0 };
};

class FixtureResourceLoader {
public:
	static bool validate_fixture_header(BinaryReader &p_reader, FixtureBundleHeader &r_header) noexcept {
		if (!p_reader.read_u32_le(r_header.magic) || r_header.magic != FixtureBundleHeader::EXPECTED_MAGIC) return false;
		if (!p_reader.read_u16_le(r_header.version)) return false;
		if (!p_reader.read_u32_le(r_header.fixture_id)) return false;
		if (!p_reader.read_u32_le(r_header.payload_bytes)) return false;
		return true;
	}
};

} // namespace Multinet

#endif // MULTINET_FIXTURE_RESOURCE_H
