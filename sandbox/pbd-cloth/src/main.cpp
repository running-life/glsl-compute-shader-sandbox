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
#include "glm/gtc/type_ptr.hpp"
//
#include "renderer.h"

Renderer* RENDERER;

static void glfwErrorCallback(int error, const char* description) {
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void GLAPIENTRY debugMessageCallback([[maybe_unused]] GLenum source,
                                     GLenum type, [[maybe_unused]] GLuint id,
                                     GLenum severity,
                                     [[maybe_unused]] GLsizei length,
                                     const GLchar* message,
                                     [[maybe_unused]] const void* userParam) {
  if (type == GL_DEBUG_TYPE_ERROR) {
    spdlog::error("[GL] type = 0x{:x}, severity = 0x{:x}, message = {}", type,
                  severity, message);
  } else {
    spdlog::info("[GL] type = 0x{:x}, severity = 0x{:x}, message = {}", type,
                 severity, message);
  }
}

static void framebufferSizeCallback([[maybe_unused]] GLFWwindow* window,
                                    int width, int height) {
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
  if (!io.WantCaptureMouse &&
      glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS) {
    RENDERER->lookAround(io.MouseDelta.x, io.MouseDelta.y);
  }

  // pause
  if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
    RENDERER->setPause(true);
  } else {
    RENDERER->setPause(false);
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
      glfwCreateWindow(800, 600, "PBD Cloth Simulation", nullptr, nullptr);
  if (!window) {
    return -1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(0);  // disable vsync

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

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    handleInput(window, io);

    // start imgui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("PBD Cloth Simulation");
    {
      ImGui::Text("Framerate: %.1f FPS", io.Framerate);
      ImGui::Text("Frame time: %.3f ms", 1000.0f / io.Framerate);

      ImGui::Separator();
      ImGui::Text("Cloth Settings");

      // 布料尺寸
      static int clothWidth = RENDERER->getClothSize().x;
      static int clothHeight = RENDERER->getClothSize().y;
      if (ImGui::InputInt("Cloth Width", &clothWidth)) {
        clothWidth = std::clamp(clothWidth, 2, 128);
      }
      if (ImGui::InputInt("Cloth Height", &clothHeight)) {
        clothHeight = std::clamp(clothHeight, 2, 128);
      }
      if (ImGui::Button("Apply Cloth Size")) {
        RENDERER->setClothSize(glm::uvec2(clothWidth, clothHeight));
      }

      ImGui::Separator();
      ImGui::Text("Physics Settings");

      // 物理参数
      static float gravity = RENDERER->getGravity();
      if (ImGui::SliderFloat("Gravity", &gravity, -20.0f, 0.0f)) {
        RENDERER->setGravity(gravity);
      }

      static float damping = RENDERER->getDamping();
      if (ImGui::SliderFloat("Damping", &damping, 0.9f, 1.0f)) {
        RENDERER->setDamping(damping);
      }

      static float stiffness = RENDERER->getStiffness();
      if (ImGui::SliderFloat("Stiffness", &stiffness, 0.0f, 2.0f)) {
        RENDERER->setStiffness(stiffness);
      }

      static int iterations = RENDERER->getSolverIterations();
      if (ImGui::SliderInt("Solver Iterations", &iterations, 1, 10)) {
        RENDERER->setSolverIterations(iterations);
      }

      static float timeStep = RENDERER->getDt();
      if (ImGui::SliderFloat("Time Step", &timeStep, 0.001f, 0.05f, "%.4f")) {
        RENDERER->setDt(timeStep);
      }

      ImGui::Separator();
      ImGui::Text("Wind Settings");

      static bool windEnabled = RENDERER->isWindEnabled();
      if (ImGui::Checkbox("Enable Wind", &windEnabled)) {
        RENDERER->setWindEnabled(windEnabled);
      }

      static glm::vec3 windForce = RENDERER->getWindForce();
      if (ImGui::SliderFloat3("Wind Force", glm::value_ptr(windForce), -10.0f, 10.0f)) {
        RENDERER->setWindForce(windForce);
      }

      ImGui::Separator();
      ImGui::Text("Rendering Settings");

      static glm::vec3 clothColor = RENDERER->getClothColor();
      if (ImGui::ColorEdit3("Cloth Color", glm::value_ptr(clothColor))) {
        RENDERER->setClothColor(clothColor);
      }

      static bool wireframe = RENDERER->isWireframeMode();
      if (ImGui::Checkbox("Wireframe Mode", &wireframe)) {
        RENDERER->setWireframeMode(wireframe);
      }

      static bool showPoints = RENDERER->isShowPoints();
      if (ImGui::Checkbox("Show Points", &showPoints)) {
        RENDERER->setShowPoints(showPoints);
      }

      ImGui::Separator();
      ImGui::Text("Controls:");
      ImGui::Text("WASD - Move camera");
      ImGui::Text("Middle mouse - Look around");
      ImGui::Text("Space - Pause simulation");
    }
    ImGui::End();

    // render
    RENDERER->render(io.DeltaTime);

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