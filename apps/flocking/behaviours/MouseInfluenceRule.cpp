#include "MouseInfluenceRule.h"
#include "imgui.h"

glm::vec2 MouseInfluenceRule::computeForce(const std::vector<BoidView>& neighborhood, const BoidView& boid) {
  glm::vec2 force(0.f);

  // ImGui::IsMouseDown(ImGuiMouseButton_Left) returns true if the left mouse button is currently pressed.
  // ImGui::GetIO().MousePos returns the current mouse position as an ImVec2.
  // glm::length(vec) returns the length of a vector

  // begin solution
  float length;
  glm::vec2 distance(0, 0);
  if (isRepulsive) //go away from mouse
  {
    if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {

      glm::vec2 mousePos(ImGui::GetIO().MousePos.x, ImGui::GetIO().MousePos.y);
      distance = boid.position - mousePos;
      length = distance.length();
      glm::vec2 repulsion = glm::normalize(distance) * length;
      force += repulsion;
    }
  }

  if (!isRepulsive) //go towards mouse
  {
    if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {

      glm::vec2 mousePos(ImGui::GetIO().MousePos.x, ImGui::GetIO().MousePos.y); //gets mouse position
      distance = boid.position - mousePos; //get distance from mouse and boid
      length = distance.length();
      glm::vec2 repulsion = glm::normalize(distance) * length;
      force -= repulsion;
    }
  }

  force*= weight*100;

  // end solution

  return force;
}

bool MouseInfluenceRule::drawImguiRuleExtra() {
  bool valueHasChanged = false;

  if (ImGui::RadioButton("Attractive", !isRepulsive)) {
    isRepulsive = false;
    valueHasChanged = true;
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("Repulsive", isRepulsive)) {
    isRepulsive = true;
    valueHasChanged = true;
  }

  return valueHasChanged;
}
