add_executable(ray-tracing "src/main.cpp")
target_compile_features(ray-tracing PRIVATE cxx_std_20)
set_target_properties(ray-tracing PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(ray-tracing PRIVATE src)
target_link_libraries(ray-tracing PRIVATE gcss)
target_link_libraries(ray-tracing PRIVATE glfw)
target_link_libraries(ray-tracing PRIVATE imgui)
target_link_libraries(ray-tracing PRIVATE imgui_glfw_opengl3)

# set cmake source dir macro
target_compile_definitions(ray-tracing PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}" CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")