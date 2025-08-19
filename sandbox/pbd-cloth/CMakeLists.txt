add_executable(pbd-cloth "src/main.cpp")
target_compile_features(pbd-cloth PRIVATE cxx_std_20)
set_target_properties(pbd-cloth PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(pbd-cloth PRIVATE src)
target_link_libraries(pbd-cloth PRIVATE gcss)
target_link_libraries(pbd-cloth PRIVATE glfw)
target_link_libraries(pbd-cloth PRIVATE imgui)
target_link_libraries(pbd-cloth PRIVATE imgui_glfw_opengl3)

# set cmake source dir macro
target_compile_definitions(pbd-cloth PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}" CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")