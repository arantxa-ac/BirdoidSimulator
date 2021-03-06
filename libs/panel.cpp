#include "panel.h"

#include <array>

namespace panel {

// default values
bool showPanel = false;
ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

// animation
bool playModel = false;
bool resetModel = false;
bool stepModel = false;
float dt = 0.01f;
float ks = 15.f;
float kc = 1.f;
float ka = 1.f;

// reset
bool resetView = false;

void updateMenu() {
  using namespace ImGui;

  giv::io::ImGuiBeginFrame();

  if (showPanel && Begin("panel", &showPanel, ImGuiWindowFlags_MenuBar)) {
    if (BeginMenuBar()) {
      if (BeginMenu("File")) {
        if (MenuItem("Close", "(P)")) {
          showPanel = false;
        }
        // add more if you would like...
        ImGui::EndMenu();
      }
      EndMenuBar();
    }

    Spacing();
    if (CollapsingHeader("Background Color")) { // Clear
      ColorEdit3("Clear color", (float *)&clear_color);
    }

    Spacing();
    Separator();
    if (Button("Play/Pause")) {
      playModel = !playModel;
    }
    resetModel = Button("Reset Model");
    stepModel = Button("Step");
    InputFloat("dt", &dt, 0.00001f, 0.1f, "%.5f");

    Spacing();
    Separator();
    InputFloat("ks", &ks, 0.1f, 1.f, "%.5f");
    InputFloat("kc", &kc, 0.1f, 1.f, "%.5f");
    InputFloat("ka", &ka, 0.1f, 1.f, "%.5f");

    Spacing();
    Separator();
    resetView = Button("Reset view");

    Spacing();
    Separator();
    Text("Application average %.3f ms/frame (%.1f FPS)",
         1000.0f / GetIO().Framerate, GetIO().Framerate);

    End();
  }
  giv::io::ImGuiEndFrame();
}

} // namespace panel
