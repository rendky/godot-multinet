#include "block_clipmap_shader.h"
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/variant/string.hpp>

namespace multinet::rendering {

static const char *s_bccm_shader_code = R"(
shader_type spatial;
render_mode blend_mix, depth_draw_opaque, cull_back, diffuse_burley, specular_schlick_ggx;

uniform uint terrain_seed = 1337u;
uniform float continental_frequency = 0.005;
uniform float min_elevation = -100.0;
uniform float max_elevation = 2000.0;

uint squirrel_noise5(int position_1d, uint seed) {
	uint mangled = uint(position_1d);
	mangled *= 0xD6E8FEB8u;
	mangled += seed;
	mangled ^= (mangled >> 9u);
	mangled += 0x9656979Du;
	mangled ^= (mangled >> 11u);
	mangled *= 0x5D588B65u;
	mangled ^= (mangled >> 13u);
	mangled += 0xE16B0127u;
	mangled ^= (mangled >> 17u);
	mangled *= 0x2A01A19Cu;
	mangled ^= (mangled >> 19u);
	return mangled;
}

uint squirrel_noise5_2d(int x, int y, uint seed) {
	int PRIME_Y = 198491317;
	return squirrel_noise5(x + (y * PRIME_Y), seed);
}

float squirrel_noise5_2d_zero_to_one(int x, int y, uint seed) {
	return float(squirrel_noise5_2d(x, y, seed)) * (1.0 / 4294967295.0);
}

vec3 eval_value_noise_2d_deriv(vec2 p, uint seed) {
	vec2 i_f = floor(p);
	ivec2 i = ivec2(int(i_f.x), int(i_f.y));
	vec2 f = fract(p);

	// Smoothstep interpolation (Hermite curve) and its derivative
	vec2 u = f * f * (3.0 - 2.0 * f);
	vec2 du = 6.0 * f * (1.0 - f);

	float n00 = squirrel_noise5_2d_zero_to_one(i.x,     i.y,     seed);
	float n10 = squirrel_noise5_2d_zero_to_one(i.x + 1, i.y,     seed);
	float n01 = squirrel_noise5_2d_zero_to_one(i.x,     i.y + 1, seed);
	float n11 = squirrel_noise5_2d_zero_to_one(i.x + 1, i.y + 1, seed);

	float a = n00;
	float b = n10 - n00;
	float c = n01 - n00;
	float d = n11 - n01 - n10 + n00;

	float value = a + b * u.x + (c + d * u.x) * u.y;
	vec2 deriv = vec2(
		du.x * (b + d * u.y),
		du.y * (c + d * u.x)
	);

	return vec3(value, deriv.x, deriv.y);
}

vec3 eval_fbm_deriv(vec2 p, uint seed) {
	float value = 0.0;
	vec2 deriv = vec2(0.0);
	float amplitude = 0.5;
	float frequency = 1.0;

	for (int octave = 0; octave < 4; octave++) {
		vec3 n = eval_value_noise_2d_deriv(p * frequency, seed + uint(octave * 1337));
		value += amplitude * n.x;
		deriv += amplitude * frequency * n.yz;
		
		frequency *= 2.0;
		amplitude *= 0.5;
	}
	return vec3(value, deriv.x, deriv.y);
}

void vertex() {
	// Instance custom data R channel contains the edge_mask
	int edge_mask = int(round(INSTANCE_CUSTOM.r));
	
	// The master block vertices range from 0.0 to 16.0
	ivec2 fine_i = ivec2(int(round(VERTEX.x)), int(round(VERTEX.z)));
	
	bool is_odd_x = (fine_i.x % 2) != 0;
	bool is_odd_z = (fine_i.y % 2) != 0;
	
	// Determine if this vertex is on an edge that borders a coarser LOD
	// +x(1), -x(2), +z(4), -z(8)
	bool on_x_edge = ((edge_mask & 1) != 0 && fine_i.x == 16) || ((edge_mask & 2) != 0 && fine_i.x == 0);
	bool on_z_edge = ((edge_mask & 4) != 0 && fine_i.y == 16) || ((edge_mask & 8) != 0 && fine_i.y == 0);
	
	// World-space parent mapping: fine_i & ~ivec2(1)
	// To prevent cracks, if a vertex is on an outer edge and its position ALONG that edge is odd,
	// we snap it to the even coordinate (which exists in the coarser LOD).
	if (on_x_edge && is_odd_z) {
		VERTEX.z = float(fine_i.y & ~1);
	}
	
	if (on_z_edge && is_odd_x) {
		VERTEX.x = float(fine_i.x & ~1);
	}

	// Calculate world-space position after horizontal snapping
	vec3 world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;

	// Procedural Height Displacement using Analytical Derivatives
	vec2 sample_pos = world_pos.xz * continental_frequency;
	vec3 h_data = eval_fbm_deriv(sample_pos, terrain_seed);
	
	float elevation_range = max_elevation - min_elevation;
	float height = min_elevation + h_data.x * elevation_range;
	VERTEX.y = height;

	// Scale the derivative by the chain rule (elevation scaling * spatial frequency scaling)
	vec2 world_deriv = h_data.yz * elevation_range * continental_frequency;
	vec3 world_normal = normalize(vec3(-world_deriv.x, 1.0, -world_deriv.y));
	
	// Extract model scaling on XZ to convert world normal to local normal
	float spacing_x = length(MODEL_MATRIX[0].xyz);
	float spacing_z = length(MODEL_MATRIX[2].xyz);
	NORMAL = normalize(vec3(world_normal.x * spacing_x, world_normal.y, world_normal.z * spacing_z));
}

void fragment() {
	ALBEDO = vec3(0.2, 0.6, 0.3); // Basic green for now
	
	// Optional wireframe-like grid logic can be added later for diagnostics
}
)";

BCCMShaderData create_bccm_shader_material() {
	godot::RenderingServer *rs = godot::RenderingServer::get_singleton();

	godot::RID shader_rid = rs->shader_create();
	rs->shader_set_code(shader_rid, godot::String(s_bccm_shader_code));

	godot::RID material_rid = rs->material_create();
	rs->material_set_shader(material_rid, shader_rid);

	return BCCMShaderData{ shader_rid, material_rid };
}

} // namespace multinet::rendering
