#include "block_clipmap_shader.h"
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/variant/string.hpp>

namespace multinet::rendering {

const char *s_bccm_shader_code = R"(
shader_type spatial;
render_mode blend_mix, depth_draw_opaque, cull_back, diffuse_burley, specular_schlick_ggx;

uniform sampler2DArray height_pages : hint_default_black, filter_nearest;

void vertex() {
	int edge_mask = int(round(INSTANCE_CUSTOM.r));
	int gpu_layer = int(round(INSTANCE_CUSTOM.g));
	
	ivec2 fine_i = ivec2(int(round(VERTEX.x)), int(round(VERTEX.z)));
	
	bool is_odd_x = (fine_i.x % 2) != 0;
	bool is_odd_z = (fine_i.y % 2) != 0;
	
	bool on_x_edge = ((edge_mask & 1) != 0 && fine_i.x == 16) || ((edge_mask & 2) != 0 && fine_i.x == 0);
	bool on_z_edge = ((edge_mask & 4) != 0 && fine_i.y == 16) || ((edge_mask & 8) != 0 && fine_i.y == 0);
	
	if (on_x_edge && is_odd_z) {
		VERTEX.z = float(fine_i.y & ~1);
	}
	
	if (on_z_edge && is_odd_x) {
		VERTEX.x = float(fine_i.x & ~1);
	}

	ivec2 page_texel = ivec2(int(round(VERTEX.x)) + 1, int(round(VERTEX.z)) + 1);

	float height = texelFetch(height_pages, ivec3(page_texel.x, page_texel.y, gpu_layer), 0).r;
	float h_up = texelFetch(height_pages, ivec3(page_texel.x, page_texel.y - 1, gpu_layer), 0).r;
	float h_dn = texelFetch(height_pages, ivec3(page_texel.x, page_texel.y + 1, gpu_layer), 0).r;
	float h_lf = texelFetch(height_pages, ivec3(page_texel.x - 1, page_texel.y, gpu_layer), 0).r;
	float h_rt = texelFetch(height_pages, ivec3(page_texel.x + 1, page_texel.y, gpu_layer), 0).r;

	VERTEX.y = height;
	
	vec3 du = vec3(
		2.0,
		h_rt - h_lf,
		0.0
	);

	vec3 dv = vec3(
		0.0,
		h_dn - h_up,
		2.0
	);

	vec3 local_normal = normalize(cross(dv, du));

	NORMAL = local_normal;
}

void fragment() {
	ALBEDO = vec3(0.2, 0.6, 0.3); 
}
)";

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
