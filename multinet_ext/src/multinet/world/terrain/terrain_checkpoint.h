#ifndef MULTINET_TERRAIN_CHECKPOINT_H
#define MULTINET_TERRAIN_CHECKPOINT_H

#include "multinet/core/coordinates.h"
#include "multinet/core/schema/binary_schema.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace Multinet {

struct RegionVersionRecord {
	int64_t cell_x{ 0 };
	int64_t cell_z{ 0 };
	uint32_t version{ 1 };
};

template <size_t MaxSavedRegions = 32>
struct TerrainCheckpointState {
	static constexpr uint32_t EXPECTED_MAGIC = 0x4D4E5443; // 'MNTC'

	uint32_t magic{ EXPECTED_MAGIC };
	uint32_t recipe_version{ 1 };
	uint32_t world_seed{ 1337 };
	WorldPosition64 player_position{};

	uint32_t region_count{ 0 };
	RegionVersionRecord region_records[MaxSavedRegions]{};

	bool add_region_record(int64_t p_cell_x, int64_t p_cell_z, uint32_t p_version) noexcept {
		if (region_count >= MaxSavedRegions) {
			return false;
		}
		region_records[region_count++] = RegionVersionRecord{ p_cell_x, p_cell_z, p_version };
		return true;
	}
};

class TerrainCheckpointSerializer {
private:
	static bool write_f64_le(BinaryWriter &p_writer, double p_val) noexcept {
		static_assert(sizeof(double) == 8, "double must be 64-bit");
		uint64_t bits = 0;
		std::memcpy(&bits, &p_val, 8);
		return p_writer.write_u64_le(bits);
	}

	static bool read_f64_le(BinaryReader &p_reader, double &r_val) noexcept {
		static_assert(sizeof(double) == 8, "double must be 64-bit");
		uint64_t bits = 0;
		if (!p_reader.read_u64_le(bits)) return false;
		std::memcpy(&r_val, &bits, 8);
		return true;
	}

public:
	template <size_t N>
	static bool write_checkpoint(BinaryWriter &p_writer, const TerrainCheckpointState<N> &p_state) noexcept {
		if (!p_writer.write_u32_le(TerrainCheckpointState<N>::EXPECTED_MAGIC)) return false;
		if (!p_writer.write_u32_le(p_state.recipe_version)) return false;
		if (!p_writer.write_u32_le(p_state.world_seed)) return false;

		if (!write_f64_le(p_writer, p_state.player_position.x)) return false;
		if (!write_f64_le(p_writer, p_state.player_position.y)) return false;
		if (!write_f64_le(p_writer, p_state.player_position.z)) return false;

		if (!p_writer.write_u32_le(p_state.region_count)) return false;
		for (uint32_t i = 0; i < p_state.region_count; ++i) {
			const auto &rec = p_state.region_records[i];
			if (!p_writer.write_u64_le(static_cast<uint64_t>(rec.cell_x))) return false;
			if (!p_writer.write_u64_le(static_cast<uint64_t>(rec.cell_z))) return false;
			if (!p_writer.write_u32_le(rec.version)) return false;
		}
		return true;
	}

	template <size_t N>
	static bool read_checkpoint(BinaryReader &p_reader, TerrainCheckpointState<N> &r_state) noexcept {
		uint32_t magic{ 0 };
		if (!p_reader.read_u32_le(magic) || magic != TerrainCheckpointState<N>::EXPECTED_MAGIC) {
			return false;
		}
		if (!p_reader.read_u32_le(r_state.recipe_version)) return false;
		if (!p_reader.read_u32_le(r_state.world_seed)) return false;

		if (!read_f64_le(p_reader, r_state.player_position.x)) return false;
		if (!read_f64_le(p_reader, r_state.player_position.y)) return false;
		if (!read_f64_le(p_reader, r_state.player_position.z)) return false;

		if (!p_reader.read_u32_le(r_state.region_count)) return false;
		if (r_state.region_count > N) return false;

		for (uint32_t i = 0; i < r_state.region_count; ++i) {
			uint64_t cx{ 0 }, cz{ 0 };
			uint32_t ver{ 0 };
			if (!p_reader.read_u64_le(cx)) return false;
			if (!p_reader.read_u64_le(cz)) return false;
			if (!p_reader.read_u32_le(ver)) return false;

			r_state.region_records[i].cell_x = static_cast<int64_t>(cx);
			r_state.region_records[i].cell_z = static_cast<int64_t>(cz);
			r_state.region_records[i].version = ver;
		}
		return true;
	}
};

} // namespace Multinet

#endif // MULTINET_TERRAIN_CHECKPOINT_H
