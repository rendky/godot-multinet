#ifndef MULTINET_HYDROLOGY_SERIALIZATION_H
#define MULTINET_HYDROLOGY_SERIALIZATION_H

#include "multinet/core/schema/binary_schema.h"
#include "multinet/world/hydrology/catchment.h"
#include "multinet/world/hydrology/water_body.h"

namespace Multinet {

// ============================================================================
// Gate: WATER-NET-01 (Compact Water Network Replication Serializer)
// Gate: WATER-SAVE-01 (Durable Hydrology State Persistence Serializer)
// ============================================================================

class WaterBodySerializer {
public:
	static constexpr uint32_t EXPECTED_MAGIC = 0x4D4E5742; // 'MNWB'

	static bool write_water_body(BinaryWriter &p_writer, const WaterBodyRecord &p_record) noexcept {
		if (!p_writer.write_u32_le(EXPECTED_MAGIC)) return false;
		if (!p_writer.write_u64_le(p_record.id)) return false;
		if (!p_writer.write_u8(static_cast<uint8_t>(p_record.kind))) return false;

		if (!p_writer.write_f64_le(p_record.centroid.x)) return false;
		if (!p_writer.write_f64_le(p_record.centroid.y)) return false;
		if (!p_writer.write_f64_le(p_record.centroid.z)) return false;

		if (!p_writer.write_f64_le(p_record.volume_m3)) return false;
		if (!p_writer.write_f64_le(p_record.surface_elevation_m)) return false;

		if (!p_writer.write_f32_le(p_record.mean_depth_m)) return false;
		if (!p_writer.write_f32_le(p_record.max_depth_m)) return false;
		if (!p_writer.write_f32_le(p_record.temperature_k)) return false;
		if (!p_writer.write_f32_le(p_record.turbidity)) return false;
		if (!p_writer.write_f32_le(p_record.salinity_psu)) return false;

		if (!p_writer.write_f32_le(p_record.mean_surface_flow_ms.x)) return false;
		if (!p_writer.write_f32_le(p_record.mean_surface_flow_ms.y)) return false;

		if (!p_writer.write_u32_le(p_record.wave_profile)) return false;
		if (!p_writer.write_u32_le(p_record.shoreline_version)) return false;
		if (!p_writer.write_u32_le(p_record.state_version)) return false;
		if (!p_writer.write_u32_le(p_record.flags)) return false;
		return true;
	}

	static bool read_water_body(BinaryReader &p_reader, WaterBodyRecord &r_record) noexcept {
		uint32_t magic{ 0 };
		if (!p_reader.read_u32_le(magic) || magic != EXPECTED_MAGIC) return false;
		if (!p_reader.read_u64_le(r_record.id)) return false;

		uint8_t kind_raw{ 0 };
		if (!p_reader.read_u8(kind_raw)) return false;
		r_record.kind = static_cast<WaterBodyKind>(kind_raw);

		if (!p_reader.read_f64_le(r_record.centroid.x)) return false;
		if (!p_reader.read_f64_le(r_record.centroid.y)) return false;
		if (!p_reader.read_f64_le(r_record.centroid.z)) return false;

		if (!p_reader.read_f64_le(r_record.volume_m3)) return false;
		if (!p_reader.read_f64_le(r_record.surface_elevation_m)) return false;

		if (!p_reader.read_f32_le(r_record.mean_depth_m)) return false;
		if (!p_reader.read_f32_le(r_record.max_depth_m)) return false;
		if (!p_reader.read_f32_le(r_record.temperature_k)) return false;
		if (!p_reader.read_f32_le(r_record.turbidity)) return false;
		if (!p_reader.read_f32_le(r_record.salinity_psu)) return false;

		if (!p_reader.read_f32_le(r_record.mean_surface_flow_ms.x)) return false;
		if (!p_reader.read_f32_le(r_record.mean_surface_flow_ms.y)) return false;

		if (!p_reader.read_u32_le(r_record.wave_profile)) return false;
		if (!p_reader.read_u32_le(r_record.shoreline_version)) return false;
		if (!p_reader.read_u32_le(r_record.state_version)) return false;
		if (!p_reader.read_u32_le(r_record.flags)) return false;
		return true;
	}
};

class CatchmentSerializer {
public:
	static constexpr uint32_t EXPECTED_MAGIC = 0x4D4E4354; // 'MNCT'

	static bool write_catchment(BinaryWriter &p_writer, const CatchmentState &p_state) noexcept {
		if (!p_writer.write_u32_le(EXPECTED_MAGIC)) return false;
		if (!p_writer.write_u64_le(p_state.id)) return false;
		if (!p_writer.write_f64_le(p_state.area_m2)) return false;
		if (!p_writer.write_f64_le(p_state.retained_water_m3)) return false;
		if (!p_writer.write_f64_le(p_state.runoff_storage_m3)) return false;

		if (!p_writer.write_f32_le(p_state.mean_slope)) return false;
		if (!p_writer.write_f32_le(p_state.mean_soil_moisture)) return false;
		if (!p_writer.write_f32_le(p_state.impervious_fraction)) return false;
		if (!p_writer.write_f32_le(p_state.vegetation_roughness)) return false;
		if (!p_writer.write_f32_le(p_state.runoff_rate_m3s)) return false;
		if (!p_writer.write_f32_le(p_state.baseflow_rate_m3s)) return false;

		if (!p_writer.write_u32_le(p_state.routing_profile)) return false;
		if (!p_writer.write_u32_le(p_state.version)) return false;
		return true;
	}

	static bool read_catchment(BinaryReader &p_reader, CatchmentState &r_state) noexcept {
		uint32_t magic{ 0 };
		if (!p_reader.read_u32_le(magic) || magic != EXPECTED_MAGIC) return false;
		if (!p_reader.read_u64_le(r_state.id)) return false;
		if (!p_reader.read_f64_le(r_state.area_m2)) return false;
		if (!p_reader.read_f64_le(r_state.retained_water_m3)) return false;
		if (!p_reader.read_f64_le(r_state.runoff_storage_m3)) return false;

		if (!p_reader.read_f32_le(r_state.mean_slope)) return false;
		if (!p_reader.read_f32_le(r_state.mean_soil_moisture)) return false;
		if (!p_reader.read_f32_le(r_state.impervious_fraction)) return false;
		if (!p_reader.read_f32_le(r_state.vegetation_roughness)) return false;
		if (!p_reader.read_f32_le(r_state.runoff_rate_m3s)) return false;
		if (!p_reader.read_f32_le(r_state.baseflow_rate_m3s)) return false;

		if (!p_reader.read_u32_le(r_state.routing_profile)) return false;
		if (!p_reader.read_u32_le(r_state.version)) return false;
		return true;
	}
};

} // namespace Multinet

#endif // MULTINET_HYDROLOGY_SERIALIZATION_H
