#include "modules/multinet/terrain/terrain_adapter.h"

#include "core/object/class_db.h"
#include "modules/multinet/memory/arena_allocator.h"

namespace Multinet {

MultinetTerrainChunk3D::MultinetTerrainChunk3D() {
	mesh_instance = memnew(MeshInstance3D);
	mesh_instance->set_name("TerrainMesh");
	add_child(mesh_instance);

	static_body = memnew(StaticBody3D);
	static_body->set_name("TerrainStaticBody");
	add_child(static_body);

	collision_shape = memnew(CollisionShape3D);
	collision_shape->set_name("TerrainCollisionShape");
	static_body->add_child(collision_shape);
}

void MultinetTerrainChunk3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_cell_x", "cell_x"), &MultinetTerrainChunk3D::set_cell_x);
	ClassDB::bind_method(D_METHOD("get_cell_x"), &MultinetTerrainChunk3D::get_cell_x);
	ClassDB::bind_method(D_METHOD("set_cell_z", "cell_z"), &MultinetTerrainChunk3D::set_cell_z);
	ClassDB::bind_method(D_METHOD("get_cell_z"), &MultinetTerrainChunk3D::get_cell_z);

	ClassDB::bind_method(D_METHOD("set_seed", "seed"), &MultinetTerrainChunk3D::set_seed);
	ClassDB::bind_method(D_METHOD("get_seed"), &MultinetTerrainChunk3D::get_seed);
	ClassDB::bind_method(D_METHOD("set_max_elevation_m", "max_elevation_m"), &MultinetTerrainChunk3D::set_max_elevation_m);
	ClassDB::bind_method(D_METHOD("get_max_elevation_m"), &MultinetTerrainChunk3D::get_max_elevation_m);

	ClassDB::bind_method(D_METHOD("generate_chunk"), &MultinetTerrainChunk3D::generate_chunk);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "cell_x"), "set_cell_x", "get_cell_x");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "cell_z"), "set_cell_z", "get_cell_z");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "seed"), "set_seed", "get_seed");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_elevation_m"), "set_max_elevation_m", "get_max_elevation_m");
}

void MultinetTerrainChunk3D::_notification(int p_what) {
	if (p_what == NOTIFICATION_READY) {
		generate_chunk();
	}
}

void MultinetTerrainChunk3D::set_cell_x(int64_t p_val) {
	cell_x = p_val;
}

int64_t MultinetTerrainChunk3D::get_cell_x() const {
	return cell_x;
}

void MultinetTerrainChunk3D::set_cell_z(int64_t p_val) {
	cell_z = p_val;
}

int64_t MultinetTerrainChunk3D::get_cell_z() const {
	return cell_z;
}

void MultinetTerrainChunk3D::set_seed(uint32_t p_seed) {
	seed = p_seed;
}

uint32_t MultinetTerrainChunk3D::get_seed() const {
	return seed;
}

void MultinetTerrainChunk3D::set_max_elevation_m(float p_elev) {
	max_elevation_m = p_elev;
}

float MultinetTerrainChunk3D::get_max_elevation_m() const {
	return max_elevation_m;
}

void MultinetTerrainChunk3D::generate_chunk() {
	TerrainRecipe recipe{};
	recipe.seed = seed;
	recipe.max_elevation_m = max_elevation_m;
	recipe.continental_frequency = continental_frequency;

	HeightfieldGenerator generator(recipe);

	RegionID r_id{ cell_x, cell_y, cell_z };
	TerrainRegionTile tile(r_id);
	if (!tile.generate(generator)) {
		return;
	}

	ArenaAllocator export_arena(131072);
	Span<const TerrainVertex> vert_span;
	if (!tile.export_render_vertices(export_arena, vert_span) || vert_span.empty()) {
		return;
	}

	constexpr size_t grid_dim = 33;
	PackedVector3Array godot_vertices;
	PackedVector3Array godot_normals;
	PackedInt32Array godot_indices;

	godot_vertices.resize(vert_span.size());
	godot_normals.resize(vert_span.size());

	for (size_t i = 0; i < vert_span.size(); ++i) {
		const auto &v = vert_span[i];
		godot_vertices.set(i, Vector3(v.x, v.y, v.z));

		// Evaluate smooth analytical normal using HeightfieldGenerator
		double world_x = (static_cast<double>(cell_x) * 1024.0) + static_cast<double>(v.x);
		double world_z = (static_cast<double>(cell_z) * 1024.0) + static_cast<double>(v.z);
		SurfaceNormal norm = generator.evaluate_normal(world_x, world_z);
		godot_normals.set(i, Vector3(norm.nx, norm.ny, norm.nz));
	}

	// Generate triangles
	for (size_t z = 0; z < grid_dim - 1; ++z) {
		for (size_t x = 0; x < grid_dim - 1; ++x) {
			int32_t v00 = static_cast<int32_t>(z * grid_dim + x);
			int32_t v10 = static_cast<int32_t>(z * grid_dim + (x + 1));
			int32_t v01 = static_cast<int32_t>((z + 1) * grid_dim + x);
			int32_t v11 = static_cast<int32_t>((z + 1) * grid_dim + (x + 1));

			godot_indices.push_back(v00);
			godot_indices.push_back(v10);
			godot_indices.push_back(v01);

			godot_indices.push_back(v10);
			godot_indices.push_back(v11);
			godot_indices.push_back(v01);
		}
	}

	Array mesh_arrays;
	mesh_arrays.resize(Mesh::ARRAY_MAX);
	mesh_arrays[Mesh::ARRAY_VERTEX] = godot_vertices;
	mesh_arrays[Mesh::ARRAY_NORMAL] = godot_normals;
	mesh_arrays[Mesh::ARRAY_INDEX] = godot_indices;

	Ref<ArrayMesh> array_mesh;
	array_mesh.instantiate();
	array_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, mesh_arrays);

	mesh_instance->set_mesh(array_mesh);

	// Physics Collision Setup via HeightMapShape3D
	Span<const float> jolt_heights;
	if (tile.export_jolt_collision_buffer(export_arena, jolt_heights) && !jolt_heights.empty()) {
		Vector<real_t> shape_heights;
		shape_heights.resize(jolt_heights.size());
		for (size_t i = 0; i < jolt_heights.size(); ++i) {
			shape_heights.set(i, jolt_heights[i]);
		}

		Ref<HeightMapShape3D> height_shape;
		height_shape.instantiate();
		height_shape->set_map_width(static_cast<int>(grid_dim));
		height_shape->set_map_depth(static_cast<int>(grid_dim));
		height_shape->set_map_data(shape_heights);

		collision_shape->set_shape(height_shape);

		// Center the collision shape to align with mesh origin (0..1024 offset)
		Transform3D shape_transform;
		shape_transform.origin = Vector3(512.0f, 0.0f, 512.0f);
		collision_shape->set_transform(shape_transform);
	}

	set_position(Vector3(static_cast<float>(cell_x * 1024), 0.0f, static_cast<float>(cell_z * 1024)));
}

} // namespace Multinet
