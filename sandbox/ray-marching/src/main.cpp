#include <cstdio>
#include <iostream>

#include "glad/gl.h"
//
#include "GLFW/glfw3.h"
//
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
//
#include "spdlog/spdlog.h"
//
#include "renderer.h"

Renderer* RENDERER;

const glm::uint windowWidth = 960;
const glm::uint windowHeight = 540;

static void glfwErrorCallback(int error, const char* description) {
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void GLAPIENTRY debugMessageCallback(
    [[maybe_unused]] GLenum source, GLenum type, [[maybe_unused]] GLuint id, GLenum severity,
    [[maybe_unused]] GLsizei length, const GLchar* message,
    [[maybe_unused]] const void* userParam) {
    if (type == GL_DEBUG_TYPE_ERROR) {
        spdlog::error(
            "[GL] type = 0x{:x}, severity = 0x{:x}, message = {}", type, severity, message);
    }
    // else {
    //     spdlog::info(
    //         "[GL] type = 0x{:x}, severity = 0x{:x}, message = {}", type, severity, message);
    // }
}

static void framebufferSizeCallback([[maybe_unused]] GLFWwindow* window, int width, int height) {
    RENDERER->setResolution(glm::uvec2(width, height));
}

void handleInput(GLFWwindow* window, const ImGuiIO& io) {
    // close window
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }

    // move camera
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        RENDERER->move(CameraMovement::FORWARD, io.DeltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        RENDERER->move(CameraMovement::LEFT, io.DeltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        RENDERER->move(CameraMovement::BACKWARD, io.DeltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        RENDERER->move(CameraMovement::RIGHT, io.DeltaTime);
    }

    // camera look around
    if (!io.WantCaptureMouse
        && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS) {
        RENDERER->lookAround(io.MouseDelta.x, io.MouseDelta.y);
    }
}

int main() {
    // init glfw
    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit()) {
        return -1;
    }

    // init window and OpenGL context
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef NDEBUG
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_FALSE);
#else
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
#endif

    GLFWwindow* window =
        glfwCreateWindow(windowWidth, windowHeight, "ray-tracing", nullptr, nullptr);
    if (!window) {
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(0); // enable vsync

    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

    // init glad
    if (!gladLoadGL((GLADloadfunc)glfwGetProcAddress)) {
        std::cerr << "failed to initialize OpenGL context" << std::endl;
        return -1;
    }

    glDebugMessageCallback(debugMessageCallback, 0);

    // init imgui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;

    // set imgui style
    ImGui::StyleColorsDark();

    // init imgui backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 460 core");

    // init renderer
    RENDERER = new Renderer();

    glm::uvec2 resolution = {windowWidth, windowHeight};
    RENDERER->setResolution(resolution);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        handleInput(window, io);

        // start imgui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Ray Tracing Controls");
        {
            ImGui::Text("Framerate %.3f", io.Framerate);

            ImGui::Separator();

            static bool useSpatialHash = RENDERER->getUseSpatialHash();
            if (ImGui::Checkbox("Use Spatial Hash", &useSpatialHash)) {
                RENDERER->setUseSpatialHash(useSpatialHash);
            }

            static bool useBVH = RENDERER->getUseBVH();
            if (ImGui::Checkbox("Use BVH", &useBVH)) {
                RENDERER->setUseBVH(useBVH);
            }

            static bool useDBVH = RENDERER->getUseDBVH();
            if (ImGui::Checkbox("Use DBVH", &useDBVH)) {
                RENDERER->setUseDBVH(useDBVH);
            }

            static glm::vec3 background_color = RENDERER->getBackgroundColor();
            if (ImGui::ColorPicker3("Background Color", glm::value_ptr(background_color))) {
                RENDERER->setBackgroundColor(background_color);
            }

        }
        ImGui::End();

        // render
        RENDERER->render();

        // render imgui
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // cleanup
    delete RENDERER;

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}