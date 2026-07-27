cmake_minimum_required(VERSION 3.16)
project(multinet_ext LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Configure godot-cpp
set(GODOTCPP_CUSTOM_API_FILE "${CMAKE_CURRENT_SOURCE_DIR}/api/extension_api.json" CACHE FILEPATH "Path to custom extension_api.json")
set(GODOTCPP_TARGET "editor" CACHE STRING "Godot build target")
set(GODOTCPP_USE_HOT_RELOAD ON CACHE BOOL "Enable hot reload")

add_subdirectory(godot-cpp)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_library(multinet SHARED ${SOURCES})

target_link_libraries(multinet PRIVATE godot::cpp)

# Destination directory in game project
set(OUTPUT_DIR "D:/Projects/Games/Project-Nations/addons/multinet/bin")

add_custom_command(TARGET multinet POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${OUTPUT_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:multinet> "${OUTPUT_DIR}/multinet.windows.debug.x86_64.dll"
    COMMENT "Copying compiled GDExtension DLL to Project-Nations..."
)
