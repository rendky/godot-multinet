#include "godot/register_types.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/godot.hpp>

#include "godot/terrain_adapter.h"
#include "godot/water_adapter.h"
#include "godot/structure_adapter.h"
#include "godot/settlement_adapter.h"

using namespace godot;

void initialize_multinet_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}

	ClassDB::register_class<MultinetBCCMNode3D>();
	ClassDB::register_class<MultinetWaterBody3D>();
	ClassDB::register_class<MultinetStructureNode3D>();
	ClassDB::register_class<MultinetSettlementNode3D>();
}

void uninitialize_multinet_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
}

extern "C" {
GDExtensionBool GDE_EXPORT multinet_library_init(GDExtensionInterfaceGetProcAddress p_get_proc_address, const GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization) {
	godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

	init_obj.register_initializer(initialize_multinet_module);
	init_obj.register_terminator(uninitialize_multinet_module);
	init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

	return init_obj.init();
}
}
