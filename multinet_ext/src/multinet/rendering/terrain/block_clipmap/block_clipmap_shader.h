#ifndef MULTINET_BLOCK_CLIPMAP_SHADER_H
#define MULTINET_BLOCK_CLIPMAP_SHADER_H

#include <cstdint>
#include <string>

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

constexpr uint32_t CANONICAL_ANALYTIC_TERRAIN_GPU_VERSION_1 = 1;

enum class TerrainSourceMode : uint8_t {
	AnalyticBase = 0,
	AbsoluteHeightPageDebug = 1,
	HybridAdditiveDelta = 2
};

struct BCCMShaderData {
	RenderID shader_rid;
	RenderID material_rid;
};

// Raw GLSL shader code string
extern const char* s_bccm_shader_code;

// Return complete GLSL shader code string with dynamically generated edge transition table
std::string get_bccm_shader_code_string();

// Create and return the RIDs of the BCCM base ShaderMaterial and Shader
BCCMShaderData create_bccm_shader_material();

// Create and return the RIDs of the BCCM unshaded diagnostic ShaderMaterial and Shader
BCCMShaderData create_bccm_unshaded_shader_material();

} // namespace multinet::rendering

#endif // MULTINET_BLOCK_CLIPMAP_SHADER_H
