#include "Core/HessianBasedDeformer/HessianBasedDeformer.h"
#include "Representation/ImplicitBspline.h"
#include "Representation/Sphere.h"
#include "Visualization/Camera.h"
#include "Visualization/Renderer.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl2.h"

#include "imgui.h"

#include <GLFW/glfw3.h>

namespace {

struct AppState {
    Camera camera{Vec3::Zero(), 3.0};
    bool dirty = true;
    bool dragging = false;
    bool panning = false;
    double lastX = 0.0, lastY = 0.0;
};

void OnMouseButton(GLFWwindow* window, int button, int action, int /*mods*/) {
    auto& state = *static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if (ImGui::GetIO().WantCaptureMouse) {
        state.dragging = state.panning = false;
        return;
    }
    const bool pressed = action == GLFW_PRESS;
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        state.dragging = pressed;
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        state.panning = pressed;
    }
    glfwGetCursorPos(window, &state.lastX, &state.lastY);
}

void OnCursorPos(GLFWwindow* window, double x, double y) {
    auto& state = *static_cast<AppState*>(glfwGetWindowUserPointer(window));
    const double dx = x - state.lastX;
    const double dy = y - state.lastY;
    state.lastX = x;
    state.lastY = y;
    if (state.dragging) {
        state.camera.Rotate(dx, dy);
        state.dirty = true;
    } else if (state.panning) {
        state.camera.Pan(dx, dy);
        state.dirty = true;
    }
}

void OnScroll(GLFWwindow* window, double /*dx*/, double dy) {
    auto& state = *static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if (ImGui::GetIO().WantCaptureMouse) {
        return;
    }
    state.camera.Zoom(dy);
    state.dirty = true;
}

constexpr ImVec4 kMinimumColor = {0.2f, 0.9f, 0.2f, 1.0f}; // green
constexpr ImVec4 kOneSaddleColor = {0.2f, 0.4f, 1.0f, 1.0f}; // blue
constexpr ImVec4 kTwoSaddleColor = {1.0f, 0.9f, 0.1f, 1.0f}; // yellow
constexpr ImVec4 kMaximumColor = {1.0f, 0.2f, 0.2f, 1.0f}; // red

void DrawCriticalPoints(const HBDebugData& data, const Camera& camera, int w, int h) {
    glPointSize(8.0f);
    glBegin(GL_POINTS);
    const auto drawAll = [&](const std::vector<Vec3>& points, const ImVec4& color) {
        glColor3f(color.x, color.y, color.z);
        for (const Vec3& p : points) {
            Vec2 ndc;
            if (camera.ProjectToNdc(p, w, h, ndc)) {
                glVertex2d(ndc.x(), ndc.y());
            }
        }
    };
    drawAll(data.minimum, kMinimumColor);
    drawAll(data.oneSaddle, kOneSaddleColor);
    drawAll(data.twoSaddle, kTwoSaddleColor);
    drawAll(data.maximum, kMaximumColor);
    glEnd();
}

void CriticalPointRow(const char* label, const std::vector<Vec3>& points, const ImVec4& color) {
    const float size = ImGui::GetFontSize();
    ImGui::ColorButton(label,
                       color,
                       ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoPicker | ImGuiColorEditFlags_NoDragDrop,
                       ImVec2(size, size));
    ImGui::SameLine();
    ImGui::Text("%s: %d", label, static_cast<int>(points.size()));
}

} // namespace

int main() {
    if (!glfwInit()) {
        return -1;
    }

    GLFWwindow* window = glfwCreateWindow(600, 600, "sdf_manipulation", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable | ImGuiConfigFlags_ViewportsEnable;

    // Install our callbacks before the ImGui backend's, so it chains to them.
    AppState state;
    glfwSetWindowUserPointer(window, &state);
    glfwSetMouseButtonCallback(window, OnMouseButton);
    glfwSetCursorPosCallback(window, OnCursorPos);
    glfwSetScrollCallback(window, OnScroll);

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL2_Init();

    Sphere sphere(Vec3::Zero(), 1.0);
    ImplicitBspline paca = ImplicitBspline::MakePaca();
    ImplicitBspline torus = ImplicitBspline::MakeTorus();
    struct ModelEntry {
        const char* name;
        const Implicit* model;
    };
    const ModelEntry models[] = {{"Sphere", &sphere}, {"Paca", &paca}, {"Torus", &torus}};
    int selectedModel = 1;

    HBDebugData hbDebugData;
    bool showSaddlePoints = false;

    Renderer renderer;
    int lastW = 0, lastH = 0;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        int w, h, fbw, fbh;
        glfwGetWindowSize(window, &w, &h);
        glfwGetFramebufferSize(window, &fbw, &fbh);
        glViewport(0, 0, fbw, fbh);

        if (state.dirty || w != lastW || h != lastH) {
            renderer.ComputeNormalMap(*models[selectedModel].model, state.camera, w, h);
            renderer.ShadeDiffuse();
            state.dirty = false;
            lastW = w;
            lastH = h;
        }

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        renderer.Draw(fbw, fbh);

        const auto* selectedBspline = dynamic_cast<const ImplicitBspline*>(models[selectedModel].model);
        if (showSaddlePoints && selectedBspline) {
            DrawCriticalPoints(hbDebugData, state.camera, w, h);
        }

        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Models");
        for (int i = 0; i < static_cast<int>(std::size(models)); ++i) {
            if (ImGui::Selectable(models[i].name, selectedModel == i) && selectedModel != i) {
                selectedModel = i;
                state.dirty = true;
            }
        }
        ImGui::End();

        ImGui::Begin("Hessian Deformer");
        if (ImGui::Button("Execute") && selectedBspline) {
            hbDebugData = HessianBasedDeformer(*selectedBspline).SearchSaddlePoints();
        }
        ImGui::Checkbox("Show critical points", &showSaddlePoints);
        CriticalPointRow("Minimum", hbDebugData.minimum, kMinimumColor);
        CriticalPointRow("1-saddle", hbDebugData.oneSaddle, kOneSaddleColor);
        CriticalPointRow("2-saddle", hbDebugData.twoSaddle, kTwoSaddleColor);
        CriticalPointRow("Maximum", hbDebugData.maximum, kMaximumColor);
        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

        if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
            GLFWwindow* backup = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup);
        }

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
