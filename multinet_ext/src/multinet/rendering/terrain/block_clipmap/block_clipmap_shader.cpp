#include "block_clipmap_shader.h"
#include "multinet/core/spatial/surface_topology.h"

#ifndef MULTINET_TEST
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/variant/string.hpp>
#endif

#include <string>

namespace multinet::rendering {

static std::string build_full_shader_code() {
	std::string code;
	code += R"(shader_type spatial;
render_mode blend_mix, depth_draw_opaque, cull_back, diffuse_burley, specular_schlick_ggx;

uniform sampler2DArray height_pages : hint_default_black, filter_nearest;

// Recipe & Authority Uniforms
uniform uint terrain_seed = 1337u;
uniform float continental_frequency = 0.005;
uniform float persistence = 0.5;
uniform float lacunarity = 2.0;
uniform uint octave_count = 4u;
uniform float min_elevation = -100.0;
uniform float max_elevation = 2000.0;
uniform float chart_half_extent_m = 10000000.0;
uniform float logical_area_radius_m = 6371000.0;
uniform float lod_spacing = 2.0;
uniform float lod_block_size = 32.0;
uniform bool parent_morph_enabled = false;
// During Freeze Update, instance translations remain live/camera-relative so the
// frozen cut stays fixed in world space. Compensate the morph distance back to
// the frozen observer anchor so the transition band itself is frozen with the cut.
uniform vec2 parent_morph_view_offset_m = vec2(0.0);
uniform uint current_lod_index = 0u;
uniform uint active_ordinary_level_count = 8u;
uniform uint world_domain_topology = 1u;
uniform float finite_half_extent_x_m = 2500000.0;
uniform float finite_half_extent_z_m = 2500000.0;
uniform float analytic_normal_sample_step_m = 0.5;
uniform uint unfolding_root_face = 0u;
uniform float unfolding_root_u_m = 0.0;
uniform float unfolding_root_v_m = 0.0;
uniform uint unfolding_root_orientation = 0u;
uniform float unfolding_root_presentation_x_m = 0.0;
uniform float unfolding_root_presentation_z_m = 0.0;
uniform vec3 logical_chart_root_direction = vec3(1.0, 0.0, 0.0);
uniform vec3 logical_chart_presentation_x_tangent = vec3(0.0);
uniform vec3 logical_chart_presentation_z_tangent = vec3(0.0);
global uniform vec2 multinet_bccm_v5_root_presentation_m;
global uniform vec3 multinet_bccm_v5_root_direction;
global uniform vec3 multinet_bccm_v5_presentation_x_tangent;
global uniform vec3 multinet_bccm_v5_presentation_z_tangent;
uniform vec3 face_color_0 = vec3(0.86, 0.30, 0.28);
uniform vec3 face_color_1 = vec3(0.24, 0.62, 0.94);
uniform vec3 face_color_2 = vec3(0.34, 0.82, 0.42);
uniform vec3 face_color_3 = vec3(0.68, 0.38, 0.88);
uniform vec3 face_color_4 = vec3(0.94, 0.70, 0.24);
uniform vec3 face_color_5 = vec3(0.90, 0.34, 0.70);
uniform bool face_colors_enabled = true;

// Curved Horizon Presentation (R1 GPU Position Curvature)
uniform bool chp_gpu_effective = false;
uniform int chp_debug_reconstruction_mode = 2; // 0 = flat baseline, 1 = identity reconstruction, 2 = CHP curved position
uniform uint chp_function_class = 2u;
uniform float chp_radius_m = 0.0;
uniform float chp_inverse_radius = 0.0;
uniform float chp_inverse_radius_squared = 0.0;
uniform float chp_camera_altitude_m = 0.0;
uniform float chp_certified_max_distance_m = 0.0;
uniform float chp_certified_max_u = 0.0;
uniform bool chp_debug_negative_height_color = false;
uniform bool chp_debug_negative_height_exaggeration = false;
uniform int bccm_debug_visual_mode = 0; // 0 = default albedo, 1 = normal rgb, 2 = morph mu, 3 = recursion depth

// Canonical Root-Relative Noise Lattice Anchors (R1.2P Precision)
uniform ivec3 terrain_root_cell_0 = ivec3(0);
uniform ivec3 terrain_root_cell_1 = ivec3(0);
uniform ivec3 terrain_root_cell_2 = ivec3(0);
uniform ivec3 terrain_root_cell_3 = ivec3(0);
uniform ivec3 terrain_root_cell_4 = ivec3(0);
uniform ivec3 terrain_root_cell_5 = ivec3(0);
uniform ivec3 terrain_root_cell_6 = ivec3(0);
uniform ivec3 terrain_root_cell_7 = ivec3(0);

uniform vec3 terrain_root_fraction_0 = vec3(0.0);
uniform vec3 terrain_root_fraction_1 = vec3(0.0);
uniform vec3 terrain_root_fraction_2 = vec3(0.0);
uniform vec3 terrain_root_fraction_3 = vec3(0.0);
uniform vec3 terrain_root_fraction_4 = vec3(0.0);
uniform vec3 terrain_root_fraction_5 = vec3(0.0);
uniform vec3 terrain_root_fraction_6 = vec3(0.0);
uniform vec3 terrain_root_fraction_7 = vec3(0.0);

varying vec2 canonical_domain_uv_m;
varying float terrain_face;
varying vec3 terrain_direction;
varying float debug_final_y_m;
varying float debug_morph_mu;
varying float debug_recursion_depth;

// Canonical Bit-Noise Hash (Rule 4: SquirrelNoise5 v1)
uint squirrel_noise5_u2_v1(uint x_bits, uint y_bits, uint seed) {
	uint bits = x_bits;
	bits *= 0xD2A80A3Fu;
	bits += seed;
	bits ^= (bits >> 9u);
	bits += y_bits;
	bits ^= (bits >> 11u);
	bits *= 0xA884F197u;
	bits ^= (bits >> 13u);
	bits *= 0x6C736F4Bu;
	bits ^= (bits >> 15u);
	bits *= 0xB79F3ABBu;
	bits ^= (bits >> 17u);
	bits *= 0x1B56C4F5u;
	return bits;
}

uint squirrel_noise5_u3_v1(uint x_bits, uint y_bits, uint z_bits, uint seed) {
	uint z_seed = squirrel_noise5_u2_v1(z_bits, 0x55335631u, seed);
	return squirrel_noise5_u2_v1(x_bits, y_bits, z_seed);
}

float squirrel_u01_24_v1(uint bits) {
	return float(bits >> 8u) * 0.000000059604644775390625;
}

float smoothstep_val(float t) {
	return t * t * (3.0 - 2.0 * t);
}

void get_terrain_root_anchor(uint oct, out ivec3 cell, out vec3 fraction) {
	if (oct == 0u) { cell = terrain_root_cell_0; fraction = terrain_root_fraction_0; }
	else if (oct == 1u) { cell = terrain_root_cell_1; fraction = terrain_root_fraction_1; }
	else if (oct == 2u) { cell = terrain_root_cell_2; fraction = terrain_root_fraction_2; }
	else if (oct == 3u) { cell = terrain_root_cell_3; fraction = terrain_root_fraction_3; }
	else if (oct == 4u) { cell = terrain_root_cell_4; fraction = terrain_root_fraction_4; }
	else if (oct == 5u) { cell = terrain_root_cell_5; fraction = terrain_root_fraction_5; }
	else if (oct == 6u) { cell = terrain_root_cell_6; fraction = terrain_root_fraction_6; }
	else { cell = terrain_root_cell_7; fraction = terrain_root_fraction_7; }
}

float sample_noise_3d_lattice(ivec3 root_cell, vec3 root_fraction, vec3 delta_phys, float frequency, uint seed) {
	vec3 scaled_local = delta_phys * frequency;
	vec3 rel_lattice = root_fraction + scaled_local;
	vec3 floor_rel = floor(rel_lattice);
	ivec3 cell_offset = ivec3(floor_rel);
	ivec3 i0 = root_cell + cell_offset;
	ivec3 i1 = i0 + ivec3(1);
	vec3 f = rel_lattice - floor_rel;
	vec3 t = vec3(smoothstep_val(f.x), smoothstep_val(f.y), smoothstep_val(f.z));

	float n000 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i0.x), uint(i0.y), uint(i0.z), seed));
	float n100 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i1.x), uint(i0.y), uint(i0.z), seed));
	float n010 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i0.x), uint(i1.y), uint(i0.z), seed));
	float n110 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i1.x), uint(i1.y), uint(i0.z), seed));
	float n001 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i0.x), uint(i0.y), uint(i1.z), seed));
	float n101 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i1.x), uint(i0.y), uint(i1.z), seed));
	float n011 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i0.x), uint(i1.y), uint(i1.z), seed));
	float n111 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i1.x), uint(i1.y), uint(i1.z), seed));

	float nx00 = mix(n000, n100, t.x);
	float nx10 = mix(n010, n110, t.x);
	float nx01 = mix(n001, n101, t.x);
	float nx11 = mix(n011, n111, t.x);

	float ny0 = mix(nx00, nx10, t.y);
	float ny1 = mix(nx01, nx11, t.y);

	return mix(ny0, ny1, t.z);
}

vec4 sample_noise_3d_lattice_gradient(ivec3 root_cell, vec3 root_fraction, vec3 delta_phys, float frequency, uint seed) {
	vec3 scaled_local = delta_phys * frequency;
	vec3 rel_lattice = root_fraction + scaled_local;
	vec3 floor_rel = floor(rel_lattice);
	ivec3 cell_offset = ivec3(floor_rel);
	ivec3 i0 = root_cell + cell_offset;
	ivec3 i1 = i0 + ivec3(1);
	vec3 f = rel_lattice - floor_rel;
	vec3 t = vec3(smoothstep_val(f.x), smoothstep_val(f.y), smoothstep_val(f.z));
	vec3 dt = 6.0 * f * (vec3(1.0) - f);

	float n000 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i0.x), uint(i0.y), uint(i0.z), seed));
	float n100 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i1.x), uint(i0.y), uint(i0.z), seed));
	float n010 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i0.x), uint(i1.y), uint(i0.z), seed));
	float n110 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i1.x), uint(i1.y), uint(i0.z), seed));
	float n001 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i0.x), uint(i0.y), uint(i1.z), seed));
	float n101 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i1.x), uint(i0.y), uint(i1.z), seed));
	float n011 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i0.x), uint(i1.y), uint(i1.z), seed));
	float n111 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i1.x), uint(i1.y), uint(i1.z), seed));

	float nx00 = mix(n000, n100, t.x);
	float nx10 = mix(n010, n110, t.x);
	float nx01 = mix(n001, n101, t.x);
	float nx11 = mix(n011, n111, t.x);
	float ny0 = mix(nx00, nx10, t.y);
	float ny1 = mix(nx01, nx11, t.y);

	float dx0 = mix((n100 - n000) * dt.x, (n110 - n010) * dt.x, t.y);
	float dx1 = mix((n101 - n001) * dt.x, (n111 - n011) * dt.x, t.y);
	float dy0 = (nx10 - nx00) * dt.y;
	float dy1 = (nx11 - nx01) * dt.y;
	vec3 gradient = vec3(
		mix(dx0, dx1, t.z),
		mix(dy0, dy1, t.z),
		(ny1 - ny0) * dt.z
	) * frequency;
	return vec4(mix(ny0, ny1, t.z), gradient);
}

float sample_noise_3d(vec3 p_pos, float frequency, uint seed) {
	vec3 p = p_pos * frequency;
	vec3 floor_p = floor(p);
	ivec3 i0 = ivec3(floor_p);
	ivec3 i1 = i0 + ivec3(1);
	vec3 t = vec3(smoothstep_val(p.x - floor_p.x), smoothstep_val(p.y - floor_p.y), smoothstep_val(p.z - floor_p.z));

	float n000 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i0.x), uint(i0.y), uint(i0.z), seed));
	float n100 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i1.x), uint(i0.y), uint(i0.z), seed));
	float n010 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i0.x), uint(i1.y), uint(i0.z), seed));
	float n110 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i1.x), uint(i1.y), uint(i0.z), seed));
	float n001 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i0.x), uint(i0.y), uint(i1.z), seed));
	float n101 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i1.x), uint(i0.y), uint(i1.z), seed));
	float n011 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i0.x), uint(i1.y), uint(i1.z), seed));
	float n111 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i1.x), uint(i1.y), uint(i1.z), seed));

	float nx00 = mix(n000, n100, t.x);
	float nx10 = mix(n010, n110, t.x);
	float nx01 = mix(n001, n101, t.x);
	float nx11 = mix(n011, n111, t.x);

	float ny0 = mix(nx00, nx10, t.y);
	float ny1 = mix(nx01, nx11, t.y);

	return mix(ny0, ny1, t.z);
}

vec4 sample_noise_3d_value_gradient(vec3 p_pos, float frequency, uint seed) {
	vec3 p = p_pos * frequency;
	vec3 floor_p = floor(p);
	ivec3 i0 = ivec3(floor_p);
	ivec3 i1 = i0 + ivec3(1);
	vec3 f = p - floor_p;
	vec3 t = vec3(smoothstep_val(f.x), smoothstep_val(f.y), smoothstep_val(f.z));
	vec3 dt = 6.0 * f * (vec3(1.0) - f);

	float n000 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i0.x), uint(i0.y), uint(i0.z), seed));
	float n100 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i1.x), uint(i0.y), uint(i0.z), seed));
	float n010 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i0.x), uint(i1.y), uint(i0.z), seed));
	float n110 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i1.x), uint(i1.y), uint(i0.z), seed));
	float n001 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i0.x), uint(i0.y), uint(i1.z), seed));
	float n101 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i1.x), uint(i0.y), uint(i1.z), seed));
	float n011 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i0.x), uint(i1.y), uint(i1.z), seed));
	float n111 = squirrel_u01_24_v1(squirrel_noise5_u3_v1(uint(i1.x), uint(i1.y), uint(i1.z), seed));

	float nx00 = mix(n000, n100, t.x);
	float nx10 = mix(n010, n110, t.x);
	float nx01 = mix(n001, n101, t.x);
	float nx11 = mix(n011, n111, t.x);
	float ny0 = mix(nx00, nx10, t.y);
	float ny1 = mix(nx01, nx11, t.y);

	float dx0 = mix((n100 - n000) * dt.x, (n110 - n010) * dt.x, t.y);
	float dx1 = mix((n101 - n001) * dt.x, (n111 - n011) * dt.x, t.y);
	float dy0 = (nx10 - nx00) * dt.y;
	float dy1 = (nx11 - nx01) * dt.y;
	vec3 gradient = vec3(
		mix(dx0, dx1, t.z),
		mix(dy0, dy1, t.z),
		(ny1 - ny0) * dt.z
	) * frequency;
	return vec4(mix(ny0, ny1, t.z), gradient);
}

float f_forward_cobe(float a, float b) {
	float a2 = a * a;
	float b2 = b * b;
	float poly = -0.0941 * a2 + 0.0276 * b2 - 0.0623 * a2 * a2 + 0.0409 * a2 * b2 + 0.0342 * b2 * b2;
	return 0.7240 * a + (1.0 - 0.7240) * a * a2 + (1.0 - a2) * a * poly;
}

vec3 cobe_map_forward(uint face, float u, float v) {
	float X = f_forward_cobe(u, v);
	float Z = f_forward_cobe(v, u);
	vec3 p;
	if (face == 0u) {        // 0: +X
		p = vec3(1.0, -Z, -X);
	} else if (face == 1u) { // 1: -X
		p = vec3(-1.0, -Z, X);
	} else if (face == 2u) { // 2: +Y
		p = vec3(X, 1.0, Z);
	} else if (face == 3u) { // 3: -Y
		p = vec3(X, -1.0, -Z);
	} else if (face == 4u) { // 4: +Z
		p = vec3(X, -Z, 1.0);
	} else {                 // 5: -Z
		p = vec3(-X, -Z, -1.0);
	}
	float len = length(p);
	return len > 0.0 ? p / len : vec3(0.0);
}

)";

	// Authoritatively generated 24-transition lookup table for GLSL
	code += "// Authoritatively generated 24-transition lookup table for GLSL:\n";
	code += "// Packed uint: dst_face (bits 0..3), dst_edge (bits 4..7), parameter_sign (bit 8: 1 if +1, 0 if -1)\n";
	code += "const uint EDGE_TABLE_GLSL[24] = uint[24](\n";
	for (uint8_t f = 0; f < 6; ++f) {
		code += "\t";
		for (uint8_t e = 0; e < 4; ++e) {
			const auto& trans = Multinet::get_edge_transition(f, static_cast<Multinet::SurfaceEdge>(e));
			uint32_t packed_val = Multinet::pack_edge_transition_for_glsl(trans);
			code += std::to_string(packed_val) + "u";
			if (f != 5 || e != 3) code += ", ";
		}
		code += "\n";
	}
	code += ");\n\n";

code += R"(void canonicalize_face_uv(inout uint face, inout float u_m, inout float v_m, float H) {
	for (int iter = 0; iter < 8; ++iter) {
		float du = (abs(u_m) > H) ? (abs(u_m) - H) : 0.0;
		float dv = (abs(v_m) > H) ? (abs(v_m) - H) : 0.0;
		if (du == 0.0 && dv == 0.0) break;

		uint edge;
		float param;
		if (du >= dv) {
			edge = (u_m > 0.0) ? 1u : 0u;
			param = v_m;
		} else {
			edge = (v_m > 0.0) ? 3u : 2u;
			param = u_m;
		}

		uint entry = EDGE_TABLE_GLSL[face * 4u + edge];
		uint dst_face = entry & 15u;
		uint dst_edge = (entry >> 4u) & 15u;
		float param_sign = ((entry & 256u) != 0u) ? 1.0 : -1.0;

		float param_dst = param * param_sign;
		float overshoot = (du >= dv) ? du : dv;

		face = dst_face;
		if (dst_edge == 0u) {        // NegativeU (-H)
			u_m = -H + overshoot;
			v_m = param_dst;
		} else if (dst_edge == 1u) { // PositiveU (+H)
			u_m = H - overshoot;
			v_m = param_dst;
		} else if (dst_edge == 2u) { // NegativeV (-H)
			u_m = param_dst;
			v_m = -H + overshoot;
		} else {                     // PositiveV (+H)
			u_m = param_dst;
			v_m = H - overshoot;
		}
	}
}

float eval_closed_analytic_height_lattice(vec3 delta_phys) {
	float amp = 1.0;
	float freq = continental_frequency;
	float total_elev = 0.0;
	float max_poss = 0.0;

	for (uint oct = 0u; oct < octave_count; ++oct) {
		uint salt = oct * 1013u;
		uint seed = terrain_seed ^ salt;
		ivec3 rcell;
		vec3 rfrac;
		get_terrain_root_anchor(oct, rcell, rfrac);
		float n = sample_noise_3d_lattice(rcell, rfrac, delta_phys, freq, seed);
		total_elev += n * amp;
		max_poss += amp;

		amp *= persistence;
		freq *= lacunarity;
	}

	float norm01 = total_elev / max_poss;
	if (norm01 < 0.5) {
		float t = norm01 * 2.0;
		return min_elevation * (1.0 - t);
	} else {
		float t = (norm01 - 0.5) * 2.0;
		return max_elevation * t;
	}
}

vec4 eval_closed_analytic_height_gradient_lattice(vec3 delta_phys) {
	float amp = 1.0;
	float freq = continental_frequency;
	float total_elev = 0.0;
	float max_poss = 0.0;
	vec3 total_gradient = vec3(0.0);

	for (uint oct = 0u; oct < octave_count; ++oct) {
		uint seed = terrain_seed ^ (oct * 1013u);
		ivec3 rcell;
		vec3 rfrac;
		get_terrain_root_anchor(oct, rcell, rfrac);
		vec4 noise = sample_noise_3d_lattice_gradient(rcell, rfrac, delta_phys, freq, seed);
		total_elev += noise.x * amp;
		total_gradient += noise.yzw * amp;
		max_poss += amp;
		amp *= persistence;
		freq *= lacunarity;
	}

	float norm01 = total_elev / max_poss;
	vec3 normalized_gradient = total_gradient / max_poss;
	if (norm01 < 0.5) {
		return vec4(
			min_elevation * (1.0 - norm01 * 2.0),
			normalized_gradient * (-2.0 * min_elevation));
	}
	return vec4(
		max_elevation * ((norm01 - 0.5) * 2.0),
		normalized_gradient * (2.0 * max_elevation));
}
)";
	code += R"(
float eval_closed_analytic_height_direction(vec3 dir) {
	vec3 phys_pos = dir * logical_area_radius_m;

	float amp = 1.0;
	float freq = continental_frequency;
	float total_elev = 0.0;
	float max_poss = 0.0;

	for (uint oct = 0u; oct < octave_count; ++oct) {
		uint salt = oct * 1013u;
		uint seed = terrain_seed ^ salt;
		float n = sample_noise_3d(phys_pos, freq, seed);
		total_elev += n * amp;
		max_poss += amp;

		amp *= persistence;
		freq *= lacunarity;
	}

	float norm01 = total_elev / max_poss;
	if (norm01 < 0.5) {
		float t = norm01 * 2.0;
		return min_elevation * (1.0 - t);
	} else {
		float t = (norm01 - 0.5) * 2.0;
		return max_elevation * t;
	}
}

vec4 eval_closed_analytic_height_gradient_direction(vec3 dir) {
	vec3 phys_pos = dir * logical_area_radius_m;
	float amp = 1.0;
	float freq = continental_frequency;
	float total_elev = 0.0;
	float max_poss = 0.0;
	vec3 total_gradient = vec3(0.0);

	for (uint oct = 0u; oct < octave_count; ++oct) {
		uint seed = terrain_seed ^ (oct * 1013u);
		vec4 noise = sample_noise_3d_value_gradient(phys_pos, freq, seed);
		total_elev += noise.x * amp;
		total_gradient += noise.yzw * amp;
		max_poss += amp;
		amp *= persistence;
		freq *= lacunarity;
	}

	float norm01 = total_elev / max_poss;
	vec3 normalized_gradient = total_gradient / max_poss;
	if (norm01 < 0.5) {
		return vec4(
			min_elevation * (1.0 - norm01 * 2.0),
			normalized_gradient * (-2.0 * min_elevation));
	}
	return vec4(
		max_elevation * ((norm01 - 0.5) * 2.0),
		normalized_gradient * (2.0 * max_elevation));
}

)";

code += R"(

float eval_closed_analytic_height(uint face, float u_m, float v_m) {
	canonicalize_face_uv(face, u_m, v_m, chart_half_extent_m);
	float u_norm = u_m / chart_half_extent_m;
	float v_norm = v_m / chart_half_extent_m;
	return eval_closed_analytic_height_direction(cobe_map_forward(face, u_norm, v_norm));
}

float eval_finite_analytic_height(float u_m, float v_m) {
	float x_m = clamp(u_m, -finite_half_extent_x_m, finite_half_extent_x_m);
	float z_m = clamp(v_m, -finite_half_extent_z_m, finite_half_extent_z_m);
	vec3 phys_pos = vec3(x_m, 0.0, z_m);
	float amp = 1.0;
	float freq = continental_frequency;
	float total_elev = 0.0;
	float max_poss = 0.0;
	for (uint oct = 0u; oct < octave_count; ++oct) {
		uint salt = oct * 1013u;
		uint seed = terrain_seed ^ salt;
		float n = sample_noise_3d(phys_pos, freq, seed);
		total_elev += n * amp;
		max_poss += amp;
		amp *= persistence;
		freq *= lacunarity;
	}
	float norm01 = total_elev / max_poss;
	if (norm01 < 0.5) return min_elevation * (1.0 - norm01 * 2.0);
	return max_elevation * ((norm01 - 0.5) * 2.0);
}

float eval_domain_analytic_height(uint face, float u_m, float v_m) {
	return world_domain_topology == 0u
		? eval_finite_analytic_height(u_m, v_m)
		: eval_closed_analytic_height(face, u_m, v_m);
}

uint face_from_direction(vec3 direction) {
	vec3 absolute_direction = abs(direction);
	if (absolute_direction.x >= absolute_direction.y && absolute_direction.x >= absolute_direction.z) {
		return direction.x >= 0.0 ? 0u : 1u;
	}
	if (absolute_direction.y >= absolute_direction.z) {
		return direction.y >= 0.0 ? 2u : 3u;
	}
	return direction.z >= 0.0 ? 4u : 5u;
}

vec2 patch_plane_to_chart(uint orientation, vec2 plane_m) {
	if (orientation == 1u) return vec2(plane_m.y, -plane_m.x);
	if (orientation == 2u) return vec2(-plane_m.x, -plane_m.y);
	if (orientation == 3u) return vec2(-plane_m.y, plane_m.x);
	return plane_m;
}

vec2 instance_chart_uv(
	float coord_u,
	float coord_v,
	uint orientation,
	bool uses_sample_patch,
	bool uses_coherent_unfolding,
	vec2 plane_m
) {
	if (uses_coherent_unfolding) {
		vec2 presentation_m = vec2(coord_u, coord_v) * lod_block_size + plane_m;
		vec2 root_delta_m = presentation_m - vec2(
			unfolding_root_presentation_x_m, unfolding_root_presentation_z_m);
		return vec2(unfolding_root_u_m, unfolding_root_v_m) +
			patch_plane_to_chart(unfolding_root_orientation, root_delta_m);
	}
	if (uses_sample_patch) {
		return vec2(coord_u, coord_v) + patch_plane_to_chart(orientation, plane_m);
	}
	return vec2(coord_u * lod_block_size, coord_v * lod_block_size) + plane_m;
}

// V5 stays inside its guarded local-chart radius. A 10th/11th-order Taylor
// pair is materially cheaper than five sin/cos evaluations per terrain vertex
// (height plus normal samples) and is sub-millimetre at the supported range.
vec3 local_exp_chart_direction(vec3 root_direction, vec3 angular_tangent) {
	float angle_squared = dot(angular_tangent, angular_tangent);
	if (angle_squared > 2.4674011) {
		float angle = sqrt(angle_squared);
		return normalize(cos(angle) * root_direction + (sin(angle) / angle) * angular_tangent);
	}
	float angle4 = angle_squared * angle_squared;
	float angle6 = angle4 * angle_squared;
	float angle8 = angle4 * angle4;
	float angle10 = angle8 * angle_squared;
	float cos_angle = 1.0 - 0.5 * angle_squared + angle4 / 24.0 - angle6 / 720.0 +
		angle8 / 40320.0 - angle10 / 3628800.0;
	float sinc_angle = 1.0 - angle_squared / 6.0 + angle4 / 120.0 - angle6 / 5040.0 +
		angle8 / 362880.0 - angle10 / 39916800.0;
	return normalize(cos_angle * root_direction + sinc_angle * angular_tangent);
}

vec2 logical_chart_root_delta_m(float coord_u, float coord_v, vec2 plane_m) {
	vec2 block_origin_m = vec2(coord_u, coord_v) * lod_block_size;
	// Neighbouring blocks must reach a shared vertex through the same rounded
	// presentation coordinate. V3 normals no longer depend on tiny offset probes,
	// so there is no reason to trade that weld for subtraction precision here.
	return (block_origin_m + plane_m) - multinet_bccm_v5_root_presentation_m;
}

void normalized_direction_jacobian(
	vec3 raw_direction,
	vec3 raw_u,
	vec3 raw_v,
	out vec3 direction,
	out vec3 direction_u,
	out vec3 direction_v
) {
	float raw_length = max(length(raw_direction), 0.0000001);
	direction = raw_direction / raw_length;
	direction_u = (raw_u - direction * dot(direction, raw_u)) / raw_length;
	direction_v = (raw_v - direction * dot(direction, raw_v)) / raw_length;
}

void logical_chart_direction_jacobian(
	vec2 root_delta_m,
	bool uses_bounded_logical_chart,
	out vec3 direction,
	out vec3 direction_u,
	out vec3 direction_v
) {
	vec3 tangent_u = multinet_bccm_v5_presentation_x_tangent;
	vec3 tangent_v = multinet_bccm_v5_presentation_z_tangent;
	vec3 angular_tangent = root_delta_m.x * tangent_u + root_delta_m.y * tangent_v;

	if (uses_bounded_logical_chart) {
		normalized_direction_jacobian(
			multinet_bccm_v5_root_direction + angular_tangent,
			tangent_u,
			tangent_v,
			direction,
			direction_u,
			direction_v);
		return;
	}

	float q = dot(angular_tangent, angular_tangent);
	float cosine;
	float sinc;
	float cosine_derivative_q;
	float sinc_derivative_q;
	if (q > 2.4674011) {
		float angle = sqrt(q);
		float sin_angle = sin(angle);
		float cos_angle = cos(angle);
		cosine = cos_angle;
		sinc = sin_angle / angle;
		cosine_derivative_q = -sin_angle / (2.0 * angle);
		sinc_derivative_q = (angle * cos_angle - sin_angle) / (2.0 * angle * angle * angle);
	} else {
		float q2 = q * q;
		float q3 = q2 * q;
		float q4 = q2 * q2;
		float q5 = q4 * q;
		cosine = 1.0 - 0.5 * q + q2 / 24.0 - q3 / 720.0 + q4 / 40320.0 - q5 / 3628800.0;
		sinc = 1.0 - q / 6.0 + q2 / 120.0 - q3 / 5040.0 + q4 / 362880.0 - q5 / 39916800.0;
		cosine_derivative_q = -0.5 + q / 12.0 - q2 / 240.0 + q3 / 10080.0 - q4 / 725760.0;
		sinc_derivative_q = -1.0 / 6.0 + q / 60.0 - q2 / 1680.0 + q3 / 90720.0 - q4 / 7983360.0;
	}

	float q_u = 2.0 * dot(angular_tangent, tangent_u);
	float q_v = 2.0 * dot(angular_tangent, tangent_v);
	vec3 raw_direction = cosine * multinet_bccm_v5_root_direction + sinc * angular_tangent;
	vec3 raw_u = cosine_derivative_q * q_u * multinet_bccm_v5_root_direction +
		sinc_derivative_q * q_u * angular_tangent + sinc * tangent_u;
	vec3 raw_v = cosine_derivative_q * q_v * multinet_bccm_v5_root_direction +
		sinc_derivative_q * q_v * angular_tangent + sinc * tangent_v;
	normalized_direction_jacobian(
		raw_direction, raw_u, raw_v, direction, direction_u, direction_v);
}

)";

code += R"(

vec3 eval_instance_closed_direction(
	uint face,
	float coord_u,
	float coord_v,
	uint orientation,
	bool uses_sample_patch,
	bool uses_coherent_unfolding,
	bool uses_logical_chart,
	bool uses_bounded_logical_chart,
	vec2 plane_m
) {
	if (uses_logical_chart) {
		vec2 root_delta_m = logical_chart_root_delta_m(coord_u, coord_v, plane_m);
		vec3 angular_tangent =
			root_delta_m.x * multinet_bccm_v5_presentation_x_tangent +
			root_delta_m.y * multinet_bccm_v5_presentation_z_tangent;
		vec3 direction = uses_bounded_logical_chart
			? normalize(multinet_bccm_v5_root_direction + angular_tangent)
			: local_exp_chart_direction(multinet_bccm_v5_root_direction, angular_tangent);
		return normalize(direction);
	}

	vec2 uv_m = instance_chart_uv(
		coord_u, coord_v, orientation, uses_sample_patch, uses_coherent_unfolding, plane_m);
	uint canonical_face = face;
	float canonical_u = uv_m.x;
	float canonical_v = uv_m.y;
	canonicalize_face_uv(canonical_face, canonical_u, canonical_v, chart_half_extent_m);
	return cobe_map_forward(
		canonical_face,
		canonical_u / chart_half_extent_m,
		canonical_v / chart_half_extent_m);
}

float eval_instance_analytic_height(
	uint face,
	float coord_u,
	float coord_v,
	uint orientation,
	bool uses_sample_patch,
	bool uses_coherent_unfolding,
	bool uses_logical_chart,
	bool uses_bounded_logical_chart,
	vec2 plane_m
) {
	if (uses_logical_chart && world_domain_topology != 0u) {
		vec2 root_delta_m = logical_chart_root_delta_m(coord_u, coord_v, plane_m);
		vec3 tangent_u = multinet_bccm_v5_presentation_x_tangent;
		vec3 tangent_v = multinet_bccm_v5_presentation_z_tangent;
		vec3 angular_tangent = root_delta_m.x * tangent_u + root_delta_m.y * tangent_v;
		float q = dot(angular_tangent, angular_tangent);
		float sinc_val;
		float cos_minus_one;
		if (q > 2.4674011) {
			float angle = sqrt(q);
			sinc_val = sin(angle) / angle;
			cos_minus_one = cos(angle) - 1.0;
		} else {
			float q2 = q * q;
			float q3 = q2 * q;
			sinc_val = 1.0 - q / 6.0 + q2 / 120.0 - q3 / 5040.0;
			cos_minus_one = -0.5 * q + q2 / 24.0 - q3 / 720.0;
		}
		vec3 delta_phys = (sinc_val * logical_area_radius_m) * angular_tangent +
		                  (cos_minus_one * logical_area_radius_m) * multinet_bccm_v5_root_direction;
		return eval_closed_analytic_height_lattice(delta_phys);
	}
	vec2 uv_m = instance_chart_uv(
		coord_u, coord_v, orientation, uses_sample_patch, uses_coherent_unfolding, plane_m);
	return eval_domain_analytic_height(face, uv_m.x, uv_m.y);
}

float finite_axis_slope(float center, float plus, float minus, float coordinate, float half_extent, float ds) {
	if (coordinate <= -half_extent + ds) return (plus - center) / ds;
	if (coordinate >= half_extent - ds) return (center - minus) / ds;
	return (plus - minus) / (2.0 * ds);
}

vec3 eval_chp_curved_surface_position(vec2 q, float height_m) {
	float d2 = dot(q, q);
	if (chp_function_class == 0u) {
		float drop = d2 * 0.5 * chp_inverse_radius;
		return vec3(q.x, height_m - drop, q.y);
	}
	float u = d2 * chp_inverse_radius_squared;
	float a = 1.0;
	float b = 0.0;
	float c = 1.0;
	float u2 = u * u;
	if (chp_function_class == 1u) {
		a = 1.0 - u / 6.0 + u2 / 120.0;
		b = u / 2.0 - u2 / 24.0;
		c = 1.0 - u / 2.0 + u2 / 24.0;
	} else {
		float u3 = u2 * u;
		a = 1.0 - u / 6.0 + u2 / 120.0 - u3 / 5040.0;
		b = u / 2.0 - u2 / 24.0 + u3 / 720.0;
		c = 1.0 - u / 2.0 + u2 / 24.0 - u3 / 720.0;
	}
	vec3 base_pos = vec3(a * q.x, -chp_radius_m * b, a * q.y);
	vec3 raw_axis = vec3(a * q.x * chp_inverse_radius, c, a * q.y * chp_inverse_radius);
	float raw_len = length(raw_axis);
	vec3 axis = raw_len > 1.0e-15 ? raw_axis / raw_len : vec3(0.0, 1.0, 0.0);
	return base_pos + height_m * axis;
}

vec3 eval_chp_curved_surface_normal(vec2 q, float height_m, float h_u, float h_v) {
	float d2 = dot(q, q);
	if (chp_function_class == 0u) {
		vec3 tangent_x = vec3(1.0, h_u - q.x * chp_inverse_radius, 0.0);
		vec3 tangent_z = vec3(0.0, h_v - q.y * chp_inverse_radius, 1.0);
		vec3 n = cross(tangent_z, tangent_x);
		float len = length(n);
		return len > 1.0e-15 ? n / len : vec3(0.0, 1.0, 0.0);
	}
	float u = d2 * chp_inverse_radius_squared;
	float a = 1.0;
	float b = 0.0;
	float c = 1.0;
	float da_du = 0.0;
	float db_du = 0.0;
	float dc_du = 0.0;
	float u2 = u * u;
	if (chp_function_class == 1u) {
		a = 1.0 - u / 6.0 + u2 / 120.0;
		b = u / 2.0 - u2 / 24.0;
		c = 1.0 - u / 2.0 + u2 / 24.0;
		da_du = -1.0 / 6.0 + u / 60.0;
		db_du = 1.0 / 2.0 - u / 12.0;
		dc_du = -1.0 / 2.0 + u / 12.0;
	} else {
		float u3 = u2 * u;
		a = 1.0 - u / 6.0 + u2 / 120.0 - u3 / 5040.0;
		b = u / 2.0 - u2 / 24.0 + u3 / 720.0;
		c = 1.0 - u / 2.0 + u2 / 24.0 - u3 / 720.0;
		da_du = -1.0 / 6.0 + u / 60.0 - u2 / 1680.0;
		db_du = 1.0 / 2.0 - u / 12.0 + u2 / 240.0;
		dc_du = -1.0 / 2.0 + u / 12.0 - u2 / 240.0;
	}
	float ux = 2.0 * q.x * chp_inverse_radius_squared;
	float uz = 2.0 * q.y * chp_inverse_radius_squared;
	float a_x = a + q.x * da_du * ux;
	float a_z = a + q.y * da_du * uz;
	float p0_y_x = -chp_radius_m * db_du * ux;
	float p0_y_z = -chp_radius_m * db_du * uz;

	vec3 dbase_dx = vec3(a_x, p0_y_x, q.y * da_du * ux);
	vec3 dbase_dz = vec3(q.x * da_du * uz, p0_y_z, a_z);

	vec3 raw_axis = vec3(a * q.x * chp_inverse_radius, c, a * q.y * chp_inverse_radius);
	float raw_len = length(raw_axis);
	vec3 axis = raw_len > 1.0e-15 ? raw_axis / raw_len : vec3(0.0, 1.0, 0.0);

	vec3 draw_dx = vec3(a_x * chp_inverse_radius, dc_du * ux, q.y * da_du * ux * chp_inverse_radius);
	vec3 draw_dz = vec3(q.x * da_du * uz * chp_inverse_radius, dc_du * uz, a_z * chp_inverse_radius);

	vec3 daxis_dx = (raw_len > 1.0e-15) ? (draw_dx - axis * dot(axis, draw_dx)) / raw_len : vec3(0.0);
	vec3 daxis_dz = (raw_len > 1.0e-15) ? (draw_dz - axis * dot(axis, draw_dz)) / raw_len : vec3(0.0);

	vec3 tangent_x = dbase_dx + axis * h_u + daxis_dx * height_m;
	vec3 tangent_z = dbase_dz + axis * h_v + daxis_dz * height_m;

	vec3 raw_n = cross(tangent_z, tangent_x);
	float n_len = length(raw_n);
	return n_len > 1.0e-15 ? raw_n / n_len : vec3(0.0, 1.0, 0.0);
}
)";
	code += R"(
float sample_bilinear_page(sampler2DArray pages, vec2 local_coord, uint layer) {
	vec2 clamped_local = clamp(local_coord, 0.0, 16.0);
	vec2 page_uv = clamped_local + 1.0;
	ivec2 base = ivec2(floor(page_uv));
	vec2 frac = page_uv - vec2(base);
	float h00 = texelFetch(pages, ivec3(base.x, base.y, int(layer)), 0).r;
	float h10 = texelFetch(pages, ivec3(base.x + 1, base.y, int(layer)), 0).r;
	float h01 = texelFetch(pages, ivec3(base.x, base.y + 1, int(layer)), 0).r;
	float h11 = texelFetch(pages, ivec3(base.x + 1, base.y + 1, int(layer)), 0).r;
	return mix(mix(h00, h10, frac.x), mix(h01, h11, frac.x), frac.y);
}

vec3 sample_bilinear_page_with_derivatives(sampler2DArray pages, vec2 local_coord, uint layer, float spacing) {
	vec2 clamped_local = clamp(local_coord, 0.0, 16.0);
	vec2 page_uv = clamped_local + 1.0;
	ivec2 base = ivec2(floor(page_uv));
	vec2 frac = page_uv - vec2(base);
	float h00 = texelFetch(pages, ivec3(base.x, base.y, int(layer)), 0).r;
	float h10 = texelFetch(pages, ivec3(base.x + 1, base.y, int(layer)), 0).r;
	float h01 = texelFetch(pages, ivec3(base.x, base.y + 1, int(layer)), 0).r;
	float h11 = texelFetch(pages, ivec3(base.x + 1, base.y + 1, int(layer)), 0).r;
	float val = mix(mix(h00, h10, frac.x), mix(h01, h11, frac.x), frac.y);
	float du = mix(h10 - h00, h11 - h01, frac.y) / spacing;
	float dv = mix(h01 - h00, h11 - h10, frac.x) / spacing;
	return vec3(val, du, dv);
}

void vertex() {
	// Decode Dedicated Instance Attributes Contract (Step 2)
	uint r_bits = uint(round(INSTANCE_CUSTOM.r));
	uint face = r_bits & 7u;
	uint edge_mask = (r_bits >> 3u) & 15u;
	uint patch_orientation = (r_bits >> 7u) & 3u;
	bool uses_sample_patch = (r_bits & (1u << 9u)) != 0u;
	bool uses_coherent_unfolding = (r_bits & (1u << 10u)) != 0u;
	bool uses_logical_chart = (r_bits & (1u << 11u)) != 0u;
	bool uses_bounded_logical_chart = (r_bits & (1u << 12u)) != 0u;
	bool uses_camera_relative_render = (r_bits & (1u << 16u)) != 0u;

	uint g_bits = uint(round(INSTANCE_CUSTOM.g));
	uint source_mode = g_bits & 3u;
	uint gpu_layer = (g_bits >> 2u) & 127u;

	float coord_u = INSTANCE_CUSTOM.b;
	float coord_v = INSTANCE_CUSTOM.a;

	ivec2 fine_i = ivec2(int(round(VERTEX.x)), int(round(VERTEX.z)));
	float vx = VERTEX.x;
	float vz = VERTEX.z;

	// Outer-edge crack-prevention snapping
	bool is_odd_x = (fine_i.x % 2) != 0;
	bool is_odd_z = (fine_i.y % 2) != 0;

	bool on_x_edge = ((edge_mask & 1u) != 0u && fine_i.x == 16) || ((edge_mask & 2u) != 0u && fine_i.x == 0);
	bool on_z_edge = ((edge_mask & 4u) != 0u && fine_i.y == 16) || ((edge_mask & 8u) != 0u && fine_i.y == 0);

	float active_morph_mu = 0.0;
	float active_recursion_depth_val = 0.0;

	if (parent_morph_enabled && current_lod_index + 1u < active_ordinary_level_count) {
		// Stage points from local integer lattice bit masks.
		// Godot shader bitwise operators are scalar-only for signed integer operands,
		// so mask each ivec2 component explicitly. fine_i is the local 0..16
		// master-grid identity, preserving the exact B1 parent phase.
		ivec2 p1_i = ivec2(fine_i.x & ~1, fine_i.y & ~1);
		ivec2 p2_i = ivec2(fine_i.x & ~3, fine_i.y & ~3);
		ivec2 p3_i = ivec2(fine_i.x & ~7, fine_i.y & ~7);
		vec2 p0 = vec2(fine_i);
		vec2 p1 = vec2(p1_i);
		vec2 p2 = vec2(p2_i);
		vec2 p3 = vec2(p3_i);

		float B0 = 16.0 * lod_spacing;
		float B1 = 2.0 * B0;
		float B2 = 4.0 * B0;

		// Stage Chebyshev distances in active camera-relative flat frame
		vec2 q0 = (MODEL_MATRIX * vec4(p0.x, 0.0, p0.y, 1.0)).xz + parent_morph_view_offset_m;
		float d0 = max(abs(q0.x), abs(q0.y));
		float mu0 = clamp((d0 - 1.25 * B0) / (0.75 * B0), 0.0, 1.0);

		vec2 q1 = (MODEL_MATRIX * vec4(p1.x, 0.0, p1.y, 1.0)).xz + parent_morph_view_offset_m;
		float d1 = max(abs(q1.x), abs(q1.y));
		float mu1 = (current_lod_index + 2u < active_ordinary_level_count)
			? clamp((d1 - 1.25 * B1) / (0.75 * B1), 0.0, 1.0)
			: 0.0;

		vec2 q2 = (MODEL_MATRIX * vec4(p2.x, 0.0, p2.y, 1.0)).xz + parent_morph_view_offset_m;
		float d2 = max(abs(q2.x), abs(q2.y));
		float mu2 = (current_lod_index + 3u < active_ordinary_level_count)
			? clamp((d2 - 1.25 * B2) / (0.75 * B2), 0.0, 1.0)
			: 0.0;

		active_morph_mu = mu0;
		active_recursion_depth_val = (mu0 > 0.0) ? (1.0 + ((mu1 > 0.0) ? 1.0 : 0.0) + ((mu2 > 0.0) ? 1.0 : 0.0)) : 0.0;

		// Coarse to fine live parent composition
		vec2 live3 = p3;
		vec2 live2 = (current_lod_index + 3u < active_ordinary_level_count) ? mix(p2, live3, mu2) : p2;
		vec2 live1 = (current_lod_index + 2u < active_ordinary_level_count) ? mix(p1, live2, mu1) : p1;
		vec2 live0 = mix(p0, live1, mu0);

		vec2 morphed_local = live0;

		// Hard outer-edge crack collapse: exact live parent endpoint
		if (on_x_edge && is_odd_z) {
			morphed_local.y = live1.y;
		}
		if (on_z_edge && is_odd_x) {
			morphed_local.x = live1.x;
		}

		vx = morphed_local.x;
		vz = morphed_local.y;
	} else {
		// Fallback / uncertified profile: pre-B1 one-level hard edge collapse
		if (on_x_edge && is_odd_z) {
			vz = float(fine_i.y & ~1);
		}
		if (on_z_edge && is_odd_x) {
			vx = float(fine_i.x & ~1);
		}
	}
	VERTEX.x = vx;
	VERTEX.z = vz;

	debug_morph_mu = active_morph_mu;
	debug_recursion_depth = active_recursion_depth_val;

	// Patch anchors are stored at the 8,8 centre vertex. Legacy finite and test
	// instances retain block-index coordinates.
	vec2 plane_m = uses_coherent_unfolding
		? vec2(vx * lod_spacing, vz * lod_spacing)
		: uses_sample_patch
		? vec2((vx - 8.0) * lod_spacing, (vz - 8.0) * lod_spacing)
		: vec2(vx * lod_spacing, vz * lod_spacing);
	vec2 raw_uv_m = instance_chart_uv(
		coord_u, coord_v, patch_orientation, uses_sample_patch, uses_coherent_unfolding, plane_m);
	float u_m = raw_uv_m.x;
	float v_m = raw_uv_m.y;
	if (world_domain_topology == 0u) {
		vec2 clipped_uv_m = clamp(
			vec2(u_m, v_m),
			vec2(-finite_half_extent_x_m, -finite_half_extent_z_m),
			vec2(finite_half_extent_x_m, finite_half_extent_z_m));
		// Boundary blocks are shared by several LODs. Collapse their outside
		// vertices onto the exact rectangle instead of leaving detached strips.
		VERTEX.x += (clipped_uv_m.x - u_m) / lod_spacing;
		VERTEX.z += (clipped_uv_m.y - v_m) / lod_spacing;
		u_m = clipped_uv_m.x;
		v_m = clipped_uv_m.y;
		plane_m = vec2(VERTEX.x * lod_spacing, VERTEX.z * lod_spacing);
	}
	canonical_domain_uv_m = vec2(u_m, v_m);
	uint color_face = (r_bits >> 13u) & 7u;
	terrain_face = float(color_face);
	bool uses_logical_analytic_gradient = world_domain_topology != 0u && uses_logical_chart;
	vec3 logical_direction = vec3(0.0);
	vec3 logical_direction_u = vec3(0.0);
	vec3 logical_direction_v = vec3(0.0);
	vec3 delta_phys = vec3(0.0);
	if (uses_logical_analytic_gradient) {
		vec2 root_delta_m = logical_chart_root_delta_m(coord_u, coord_v, plane_m);
		logical_chart_direction_jacobian(
			root_delta_m,
			uses_bounded_logical_chart,
			logical_direction,
			logical_direction_u,
			logical_direction_v);
		terrain_direction = logical_direction;

		vec3 tangent_u = multinet_bccm_v5_presentation_x_tangent;
		vec3 tangent_v = multinet_bccm_v5_presentation_z_tangent;
		vec3 angular_tangent = root_delta_m.x * tangent_u + root_delta_m.y * tangent_v;
		float q = dot(angular_tangent, angular_tangent);
		float sinc_val;
		float cos_minus_one;
		if (q > 2.4674011) {
			float angle = sqrt(q);
			sinc_val = sin(angle) / angle;
			cos_minus_one = cos(angle) - 1.0;
		} else {
			float q2 = q * q;
			float q3 = q2 * q;
			sinc_val = 1.0 - q / 6.0 + q2 / 120.0 - q3 / 5040.0;
			cos_minus_one = -0.5 * q + q2 / 24.0 - q3 / 720.0;
		}
		delta_phys = (sinc_val * logical_area_radius_m) * angular_tangent +
		             (cos_minus_one * logical_area_radius_m) * multinet_bccm_v5_root_direction;
	} else {
		terrain_direction = world_domain_topology != 0u
			? eval_instance_closed_direction(
				face, coord_u, coord_v, patch_orientation, uses_sample_patch,
				uses_coherent_unfolding, uses_logical_chart, uses_bounded_logical_chart, plane_m)
			: vec3(0.0);
	}

	vec4 logical_height_gradient = uses_logical_analytic_gradient
		? eval_closed_analytic_height_gradient_lattice(delta_phys)
		: vec4(0.0);
	// The analytic gradient already carries the exact centre height. Re-running
	// all noise octaves here doubled the closed-world vertex cost for no result.
	float h_center = uses_logical_analytic_gradient
		? logical_height_gradient.x
		: eval_instance_analytic_height(
			face, coord_u, coord_v, patch_orientation, uses_sample_patch, uses_coherent_unfolding,
			uses_logical_chart, uses_bounded_logical_chart, plane_m);

	// The logical closed chart has an exact derivative. Its former half-metre
	// direction probes fall below FP32 angular precision at Earth radius.
	float h_rt = h_center;
	float h_lf = h_center;
	float h_dn = h_center;
	float h_up = h_center;
	float analytic_slope_u;
	float analytic_slope_v;
	if (uses_logical_analytic_gradient) {
		vec3 physical_gradient = logical_height_gradient.yzw;
		analytic_slope_u = dot(
			physical_gradient, logical_area_radius_m * logical_direction_u);
		analytic_slope_v = dot(
			physical_gradient, logical_area_radius_m * logical_direction_v);
	} else {
		h_rt = eval_instance_analytic_height(
			face, coord_u, coord_v, patch_orientation, uses_sample_patch, uses_coherent_unfolding, uses_logical_chart, uses_bounded_logical_chart,
			plane_m + vec2(analytic_normal_sample_step_m, 0.0));
		h_lf = eval_instance_analytic_height(
			face, coord_u, coord_v, patch_orientation, uses_sample_patch, uses_coherent_unfolding, uses_logical_chart, uses_bounded_logical_chart,
			plane_m - vec2(analytic_normal_sample_step_m, 0.0));
		h_dn = eval_instance_analytic_height(
			face, coord_u, coord_v, patch_orientation, uses_sample_patch, uses_coherent_unfolding, uses_logical_chart, uses_bounded_logical_chart,
			plane_m + vec2(0.0, analytic_normal_sample_step_m));
		h_up = eval_instance_analytic_height(
			face, coord_u, coord_v, patch_orientation, uses_sample_patch, uses_coherent_unfolding, uses_logical_chart, uses_bounded_logical_chart,
			plane_m - vec2(0.0, analytic_normal_sample_step_m));
		analytic_slope_u = world_domain_topology == 0u
			? finite_axis_slope(h_center, h_rt, h_lf, u_m, finite_half_extent_x_m, analytic_normal_sample_step_m)
			: (h_rt - h_lf) / (2.0 * analytic_normal_sample_step_m);
		analytic_slope_v = world_domain_topology == 0u
			? finite_axis_slope(h_center, h_dn, h_up, v_m, finite_half_extent_z_m, analytic_normal_sample_step_m)
			: (h_dn - h_up) / (2.0 * analytic_normal_sample_step_m);
	}

	float final_y = h_center;
	float final_slope_u = analytic_slope_u;
	float final_slope_v = analytic_slope_v;

	if (source_mode == 1u) {
		// AbsoluteHeightPageDebug mode: single-fetch bilinear height and analytic derivatives
		if (gpu_layer > 0u) {
			vec3 page_sample = sample_bilinear_page_with_derivatives(height_pages, vec2(vx, vz), gpu_layer, lod_spacing);
			final_y = page_sample.x;
			final_slope_u = page_sample.y;
			final_slope_v = page_sample.z;
		} else {
			final_y = h_center;
		}
	} else if (source_mode == 2u) {
		// HybridAdditiveDelta mode: analytic base + single-fetch additive delta derivatives
		if (gpu_layer > 0u) {
			vec3 delta_sample = sample_bilinear_page_with_derivatives(height_pages, vec2(vx, vz), gpu_layer, lod_spacing);
			final_y = h_center + delta_sample.x;
			final_slope_u += delta_sample.y;
			final_slope_v += delta_sample.z;
		} else {
			final_y = h_center;
		}
	} else {
		// AnalyticBase mode (Production): immediate canonical analytic height and slopes
		final_y = h_center;
	}
	VERTEX.y = final_y;

	debug_final_y_m = final_y;
	float presented_final_y = final_y;
	if (chp_debug_negative_height_exaggeration && final_y < 0.0) {
		presented_final_y = final_y * 10.0;
	}

	vec3 bx = MODEL_MATRIX[0].xyz;
	vec3 by = MODEL_MATRIX[1].xyz;
	vec3 bz = MODEL_MATRIX[2].xyz;
	float bx2 = dot(bx, bx);
	float by2 = dot(by, by);
	float bz2 = dot(bz, bz);

	vec3 target_normal_cam = normalize(vec3(-final_slope_u, 1.0, -final_slope_v));

	if (chp_gpu_effective && uses_camera_relative_render && chp_debug_reconstruction_mode > 0) {
		vec3 flat_model = vec3(VERTEX.x, presented_final_y, VERTEX.z);
		vec3 flat_camera_relative = (MODEL_MATRIX * vec4(flat_model, 1.0)).xyz;

		vec3 target_camera_relative;
		if (chp_debug_reconstruction_mode == 1) {
			// Mode 1: Identity reconstruction (flat target)
			target_camera_relative = flat_camera_relative;
		} else {
			// Mode 2: CHP curved position and composed curved normal
			vec2 q = flat_camera_relative.xz;
			target_camera_relative = eval_chp_curved_surface_position(q, presented_final_y) - vec3(0.0, chp_camera_altitude_m, 0.0);
			target_normal_cam = eval_chp_curved_surface_normal(q, presented_final_y, final_slope_u, final_slope_v);
		}

		// Exact inverse reconstruction for orthogonal scaled model basis
		if (bx2 > 1e-12 && by2 > 1e-12 && bz2 > 1e-12) {
			vec3 delta = target_camera_relative - MODEL_MATRIX[3].xyz;
			vec3 model_local_target = vec3(
				dot(delta, bx) / bx2,
				dot(delta, by) / by2,
				dot(delta, bz) / bz2
			);
			VERTEX = model_local_target;
		}
	}

	// Map presentation normal back into model-local space for MODEL_NORMAL_MATRIX
	if (bx2 > 1e-12 && by2 > 1e-12 && bz2 > 1e-12) {
		NORMAL = normalize(vec3(
			dot(bx, target_normal_cam),
			dot(by, target_normal_cam),
			dot(bz, target_normal_cam)
		));
	} else {
		NORMAL = normalize(vec3(-final_slope_u * lod_spacing, 1.0, -final_slope_v * lod_spacing));
	}

	if (uses_camera_relative_render) {
		// MODEL_MATRIX contains the small observer-relative MultiMesh transform.
		// Strip the editor camera translation from the view matrix before Godot's
		// normal post-vertex transform so rotation never subtracts huge floats.
		MODELVIEW_MATRIX = mat4(mat3(VIEW_MATRIX)) * MODEL_MATRIX;
		MODELVIEW_NORMAL_MATRIX = mat3(VIEW_MATRIX) * MODEL_NORMAL_MATRIX;
	}
}

void fragment() {
	if (world_domain_topology == 0u && (abs(canonical_domain_uv_m.x) > finite_half_extent_x_m || abs(canonical_domain_uv_m.y) > finite_half_extent_z_m)) {
		discard;
	}
	vec3 base_color = vec3(0.2, 0.6, 0.3);
	if (world_domain_topology != 0u && face_colors_enabled) {
		// Classify the same canonical direction used for height evaluation. This
		// keeps the color seam on the actual cube-face boundary instead of on the
		// canonical owner block that happened to supply the page.
		// Dominant-axis classification is scale invariant. Normalizing this varying
		// per pixel burned a square root across the entire filled viewport.
		uint face_index = face_from_direction(terrain_direction);
		if (face_index == 0u) base_color = face_color_0;
		else if (face_index == 1u) base_color = face_color_1;
		else if (face_index == 2u) base_color = face_color_2;
		else if (face_index == 3u) base_color = face_color_3;
		else if (face_index == 4u) base_color = face_color_4;
		else base_color = face_color_5;
	}
	if (chp_debug_negative_height_color) {
		if (debug_final_y_m < 0.0) {
			base_color = vec3(0.1, 0.2, 0.9); // Deep blue for negative height
		} else {
			base_color = vec3(0.9, 0.8, 0.2); // Golden yellow for nonnegative height
		}
	}
	if (bccm_debug_visual_mode == 1) {
		base_color = NORMAL * 0.5 + 0.5;
	} else if (bccm_debug_visual_mode == 2) {
		base_color = vec3(debug_morph_mu);
	} else if (bccm_debug_visual_mode == 3) {
		if (debug_recursion_depth < 0.5) base_color = vec3(0.1, 0.2, 0.8);
		else if (debug_recursion_depth < 1.5) base_color = vec3(0.2, 0.8, 0.2);
		else if (debug_recursion_depth < 2.5) base_color = vec3(0.9, 0.8, 0.1);
		else base_color = vec3(0.9, 0.1, 0.1);
	}
	ALBEDO = base_color;
}
)";

	return code;
}

static const std::string g_shader_code_storage = build_full_shader_code();
const char *s_bccm_shader_code = g_shader_code_storage.c_str();

std::string get_bccm_shader_code_string() {
	return g_shader_code_storage;
}

BCCMShaderData create_bccm_shader_material() {
#ifndef MULTINET_TEST
	godot::RenderingServer *rs = godot::RenderingServer::get_singleton();

	godot::RID shader_rid = rs->shader_create();
	rs->shader_set_code(shader_rid, godot::String(s_bccm_shader_code));

	godot::RID material_rid = rs->material_create();
	rs->material_set_shader(material_rid, shader_rid);

	return BCCMShaderData{ shader_rid, material_rid };
#else
	return BCCMShaderData{ 0, 0 };
#endif
}

} // namespace multinet::rendering
