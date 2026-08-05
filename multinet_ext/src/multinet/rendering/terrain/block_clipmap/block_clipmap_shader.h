#ifndef MULTINET_BLOCK_CLIPMAP_SHADER_H
#define MULTINET_BLOCK_CLIPMAP_SHADER_H

#include <cstdint>

#ifndef MULTINET_TEST
#include <godot_cpp/classes/shader.hpp>
#include <godot_cpp/classes/shader_material.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/rid.hpp>
#endif

#ifdef MULTINET_TEST
using RenderID = uint64_t;
#else
using RenderID = godot::RID;
#endif

namespace multinet::rendering {

struct BCCMShaderData {
	RenderID shader_rid;
	RenderID material_rid;
};

// Raw GLSL shader code string
extern const char* s_bccm_shader_code;

// Create and return the RIDs of the BCCM base ShaderMaterial and Shader
BCCMShaderData create_bccm_shader_material();

} // namespace multinet::rendering

#endif // MULTINET_BLOCK_CLIPMAP_SHADER_H
