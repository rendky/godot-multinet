#include "multinet/world/terrain/terrain_queries.h"
#include "multinet/world/terrain/terrain_recipe.h"
#include "multinet/core/spatial/world_manifests.h"

#include <iostream>
#include <cmath>
#include <cassert>

using namespace Multinet;

void test_terrain_recipe_identity() {
    WorldScaleInput input;
    input.area_equivalent_side_m = 5000000;
    WorldScaleManifest scale = build_world_scale_manifest(input);
    assert(scale.is_valid());

    TerrainRecipe recipe1;
    assert(finalize_terrain_recipe(recipe1, scale));
    assert(validate_terrain_recipe(recipe1, scale));

    TerrainRecipe recipe2 = recipe1;
    assert(compute_terrain_recipe_hash(recipe1) == compute_terrain_recipe_hash(recipe2));

    // one changed parameter -> different hash -> rejection
    TerrainRecipe recipe3 = recipe1;
    recipe3.legacy_signals.continental_frequency = 0.0002f;
    assert(compute_terrain_recipe_hash(recipe1) != compute_terrain_recipe_hash(recipe3));
    assert(!validate_terrain_recipe(recipe3, scale));

    // one changed world size -> different manifest hash -> rejection
    WorldScaleInput input2;
    input2.area_equivalent_side_m = 6000000;
    WorldScaleManifest scale2 = build_world_scale_manifest(input2);
    assert(scale.manifest_hash != scale2.manifest_hash);
    assert(!validate_terrain_recipe(recipe1, scale2));
}

void test_canonical_query_versions() {
    WorldScaleInput input;
    WorldScaleManifest scale = build_world_scale_manifest(input);

    TerrainRecipe recipe;
    finalize_terrain_recipe(recipe, scale);

    CanonicalTerrainSignalV1 signal(recipe, scale);
    TerrainFieldEvaluator evaluator(signal, scale, 1);

    // Valid alias -> accepted
    SurfacePosition64 pos;
    pos.face = SurfaceFace::PositiveX;
    pos.u_m = 0.0;
    pos.v_m = 0.0;
    pos.topology_version = scale.topology_version;
    pos.projection_version = scale.projection_version;
    auto eval1 = evaluator.evaluate(pos);
    assert(eval1.valid);

    // Wrong topology version -> rejected
    SurfacePosition64 pos2 = pos;
    pos2.topology_version = 999;
    auto eval2 = evaluator.evaluate(pos2);
    assert(!eval2.valid);

    // Wrong projection version -> rejected
    SurfacePosition64 pos3 = pos;
    pos3.projection_version = 999;
    auto eval3 = evaluator.evaluate(pos3);
    assert(!eval3.valid);

    // Invalid face -> rejected
    SurfacePosition64 pos4 = pos;
    pos4.face = static_cast<SurfaceFace>(255);
    auto eval4 = evaluator.evaluate(pos4);
    assert(!eval4.valid);

    // Non-finite coordinate -> rejected
    SurfacePosition64 pos5 = pos;
    pos5.u_m = std::numeric_limits<double>::infinity();
    auto eval5 = evaluator.evaluate(pos5);
    assert(!eval5.valid);

    // Overshoot beyond canonicalization contract -> rejected
    SurfacePosition64 pos6 = pos;
    pos6.u_m = scale.chart_half_extent_mm * 10.0; // Very large overshoot
    auto eval6 = evaluator.evaluate(pos6);
    assert(!eval6.valid);
}

void test_world_scale_safety() {
    WorldScaleInput input;
    input.area_equivalent_side_m = 0;
    WorldScaleManifest scale1 = build_world_scale_manifest(input);
    assert(!scale1.is_valid());

    input.area_equivalent_side_m = 0xFFFFFFFFFFULL;
    WorldScaleManifest scale2 = build_world_scale_manifest(input);
    assert(!scale2.is_valid());
}

int main() {
    test_terrain_recipe_identity();
    test_canonical_query_versions();
    test_world_scale_safety();

    std::cout << "IDENTITY-VALIDATION: OK" << std::endl;
    return 0;
}
