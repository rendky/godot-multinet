#ifndef MULTINET_BLOCK_CLIPMAP_SHADER_H
#define MULTINET_BLOCK_CLIPMAP_SHADER_H

#include <godot_cpp/classes/shader.hpp>
#include <godot_cpp/classes/shader_material.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/rid.hpp>

namespace multinet::rendering {

struct BCCMShaderData {
	godot::RID shader_rid;
	godot::RID material_rid;
};

// Create and return the RIDs of the BCCM base ShaderMaterial and Shader
BCCMShaderData create_bccm_shader_material();

} // namespace multinet::rendering

#endif // MULTINET_BLOCK_CLIPMAP_SHADER_H
