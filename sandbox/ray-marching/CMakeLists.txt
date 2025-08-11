add_executable(ray-marching "src/main.cpp" "src/bvh.cpp")
target_compile_features(ray-marching PRIVATE cxx_std_20)
set_target_properties(ray-marching PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(ray-marching PRIVATE src)
target_link_libraries(ray-marching PRIVATE gcss)
target_link_libraries(ray-marching PRIVATE glfw)
target_link_libraries(ray-marching PRIVATE imgui)
target_link_libraries(ray-marching PRIVATE imgui_glfw_opengl3)

# set cmake source dir macro
target_compile_definitions(ray-marching PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}" CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")