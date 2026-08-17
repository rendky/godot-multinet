cmake_minimum_required(VERSION 3.16)
project(multinet_ext LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Full-debug MSVC links can deadlock through the shared symbol service when
# Ninja starts several of them together. Keep compilation parallel and admit
# one linker at a time.
set_property(GLOBAL PROPERTY JOB_POOLS multinet_link_pool=1)
set(CMAKE_JOB_POOL_LINK multinet_link_pool)

# Configure godot-cpp
set(GODOTCPP_CUSTOM_API_FILE "${CMAKE_CURRENT_SOURCE_DIR}/api/extension_api.json" CACHE FILEPATH "Path to custom extension_api.json")
set(GODOTCPP_TARGET "editor" CACHE STRING "Godot build target")
set(GODOTCPP_USE_HOT_RELOAD ON CACHE BOOL "Enable hot reload")

add_subdirectory(godot-cpp)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

file(GLOB_RECURSE ALL_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)
list(FILTER ALL_SOURCES EXCLUDE REGEX "src/tests/.*")

# GDExtension DLL target
add_library(multinet SHARED ${ALL_SOURCES})
target_link_libraries(multinet PRIVATE godot::cpp)

# Destination directory in game project
set(OUTPUT_DIR "D:/Projects/Games/Project-Nations/addons/multinet/bin")

# Godot copies reloadable extensions beside the loaded DLL. The project tree is
# intentionally read-only for the normal editor process, so keep a second
# development copy in the user's writable temp directory for hot reload.
set(HOT_RELOAD_DIR "$ENV{TEMP}/multinet_hotreload")

add_custom_command(TARGET multinet POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${OUTPUT_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:multinet> "${OUTPUT_DIR}/multinet.windows.debug.x86_64.dll"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${HOT_RELOAD_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:multinet> "${HOT_RELOAD_DIR}/multinet.windows.debug.x86_64.dll"
    COMMENT "Copying compiled GDExtension DLL to Project-Nations..."
)

# Standalone CPU test fixture — excludes Godot engine adapter bindings (src/godot/.*)
set(CORE_SOURCES ${ALL_SOURCES})
list(FILTER CORE_SOURCES EXCLUDE REGEX "src/godot/.*")

add_executable(bccm_surface_edge_fixture EXCLUDE_FROM_ALL src/tests/bccm_surface_edge_fixture.cpp ${CORE_SOURCES})
target_compile_definitions(bccm_surface_edge_fixture PRIVATE MULTINET_TEST DEBUG_ENABLED)
target_link_libraries(bccm_surface_edge_fixture PRIVATE godot::cpp)

foreach(WP6_FIXTURE world_domain_fixture editor_domain_navigation_fixture bccm_flat_normal_fixture canonicalization_fixture surface_scale_transition_fixture chp_cpu_reference_fixture chp_r1_position_reconstruction_fixture chp_r1_2p_terrain_precision_fixture bccm_freeze_presentation_fixture)
    add_executable(${WP6_FIXTURE} EXCLUDE_FROM_ALL src/tests/${WP6_FIXTURE}.cpp ${CORE_SOURCES})
    target_compile_definitions(${WP6_FIXTURE} PRIVATE MULTINET_TEST DEBUG_ENABLED)
    target_link_libraries(${WP6_FIXTURE} PRIVATE godot::cpp)
endforeach()
