#ifndef MULTINET_SETTLEMENT_GENERATOR_H
#define MULTINET_SETTLEMENT_GENERATOR_H

#include "../terrain/terrain_landform_contracts.h"
#include "settlement_types.h"
#include "../../core/squirrel_noise5.h"
#include <array>

namespace Multinet {

// ============================================================================
// SettlementGenerator - M4 Procedural District/Block Subdivider
// Deterministically generates bounded blocks, recursive compounds, and
// building requests using FeatureKey and SquirrelNoise5.
// ============================================================================

class SettlementGenerator {
public:
	static constexpr size_t MAX_COMPOUNDS_PER_BLOCK = 16;
	static constexpr size_t MAX_ROADS_PER_BLOCK = 4;
	
	struct BlockGenerationResult {
		BlockRecord block{};
		std::array<CompoundRecord, MAX_COMPOUNDS_PER_BLOCK> compounds{};
		std::array<BuildingDevelopmentRequest, MAX_COMPOUNDS_PER_BLOCK> buildings{};
		std::array<RoadSegmentRecord, MAX_ROADS_PER_BLOCK> roads{};
		std::array<LandformConstraint, MAX_COMPOUNDS_PER_BLOCK> pads{};
		size_t compound_count{0};
		size_t road_count{0};
	};

private:
	static constexpr void subdivide_rect(const AABB64& p_bounds, uint32_t p_seed, uint8_t p_depth, 
	                          std::array<CompoundRecord, MAX_COMPOUNDS_PER_BLOCK>& r_compounds, 
	                          size_t& r_count, const FeatureKey& p_parent_key) noexcept {
		if (r_count >= MAX_COMPOUNDS_PER_BLOCK) return;
		
		float min_size = 20.0f;
		
		// SquirrelNoise for split decision
		float r1 = squirrel_u01_24_v1(squirrel_noise5_i2_v1(p_depth, 0, p_seed));
		bool split_horiz = (p_bounds.size.x > p_bounds.size.z);
		// Force split if aspect ratio is extreme
		if (p_bounds.size.x / p_bounds.size.z > 1.5) split_horiz = true;
		if (p_bounds.size.z / p_bounds.size.x > 1.5) split_horiz = false;
		
		bool can_split_h = (p_bounds.size.x > min_size * 2.0);
		bool can_split_v = (p_bounds.size.z > min_size * 2.0);
		
		if (!can_split_h && !can_split_v || p_depth > 4) {
			// Leaf node - create compound
			CompoundRecord& comp = r_compounds[r_count];
			comp.compound_id = p_seed + r_count;
			comp.key = p_parent_key.derive_child(2, r_count, 1);
			comp.bounds = p_bounds;
			comp.zoning_class = 1; 
			comp.building_count = 1;
			r_count++;
			return;
		}
		
		if (split_horiz && !can_split_h) split_horiz = false;
		if (!split_horiz && !can_split_v) split_horiz = true;
		
		// Split ratio between 0.4 and 0.6
		double split_ratio = 0.4 + (squirrel_u01_24_v1(squirrel_noise5_i2_v1(p_depth, 1, p_seed)) * 0.2);
		
		AABB64 bounds1 = p_bounds;
		AABB64 bounds2 = p_bounds;
		
		if (split_horiz) {
			bounds1.size.x = p_bounds.size.x * split_ratio;
			bounds2.size.x = p_bounds.size.x * (1.0 - split_ratio);
			bounds2.position.x += bounds1.size.x;
		} else {
			bounds1.size.z = p_bounds.size.z * split_ratio;
			bounds2.size.z = p_bounds.size.z * (1.0 - split_ratio);
			bounds2.position.z += bounds1.size.z;
		}
		
		uint32_t seed1 = squirrel_noise5_i2_v1(p_depth, 2, p_seed);
		uint32_t seed2 = squirrel_noise5_i2_v1(p_depth, 3, p_seed);
		
		subdivide_rect(bounds1, seed1, p_depth + 1, r_compounds, r_count, p_parent_key);
		subdivide_rect(bounds2, seed2, p_depth + 1, r_compounds, r_count, p_parent_key);
	}

public:
	[[nodiscard]] static constexpr bool generate_block(
			uint64_t p_seed,
			BlockID p_block_id,
			const WorldPosition64& p_center,
			const Vec3f& p_extents,
			BlockGenerationResult& r_result) noexcept {
				
		const FeatureKey block_key = FeatureKey::make_root(p_seed ^ p_block_id);
		
		r_result.block.block_id = p_block_id;
		r_result.block.key = block_key;
		r_result.block.bounds.position = WorldPosition64{ p_center.x - p_extents.x, p_center.y, p_center.z - p_extents.z };
		r_result.block.bounds.size = WorldPosition64{ p_extents.x * 2.0, p_extents.y * 2.0, p_extents.z * 2.0 };
		r_result.compound_count = 0;
		r_result.road_count = 0;
		
		// 1. Recursive OBB Subdivision
		subdivide_rect(r_result.block.bounds, static_cast<uint32_t>(p_seed), 0, r_result.compounds, r_result.compound_count, block_key);
		r_result.block.compound_count = static_cast<uint16_t>(r_result.compound_count);
		
		// 2. Generate Building Requests and Pads for each compound
		for (size_t i = 0; i < r_result.compound_count; ++i) {
			const CompoundRecord& comp = r_result.compounds[i];
			BuildingDevelopmentRequest& req = r_result.buildings[i];
			LandformConstraint& pad = r_result.pads[i];
			
			const FeatureKey bldg_key = comp.key.derive_child(3, 0, 1);
			uint32_t bldg_seed = static_cast<uint32_t>(bldg_key.path_hash);
			
			// Leave a 2m setback from the compound bounds
			float padding = 2.0f;
			req.compound_id = comp.compound_id;
			req.building_key = bldg_key;
			req.position = WorldPosition64{ 
				comp.bounds.position.x + comp.bounds.size.x * 0.5, 
				comp.bounds.position.y, 
				comp.bounds.position.z + comp.bounds.size.z * 0.5 
			};
			req.extents_m = Vec3f{ 
				static_cast<float>(comp.bounds.size.x * 0.5 - padding), 
				6.0f, 
				static_cast<float>(comp.bounds.size.z * 0.5 - padding) 
			};
			req.seed = bldg_seed;
			
			// Select program deterministically
			float prog_roll = squirrel_u01_24_v1(squirrel_noise5_i2_v1(0, 0, bldg_seed));
			if (prog_roll < 0.6f) req.program_type = BuildingProgram::Dwelling;
			else if (prog_roll < 0.85f) req.program_type = BuildingProgram::Shop;
			else req.program_type = BuildingProgram::Warehouse;
			
			pad.type = LandformType::BuildingPad;
			pad.center = req.position;
			pad.extents_m = req.extents_m;
			pad.target_elevation_m = 0.0f; // Assumes flat ground for now
			pad.priority = 100;
		}
		
		return true;
	}
};

} // namespace Multinet

#endif // MULTINET_SETTLEMENT_GENERATOR_H
