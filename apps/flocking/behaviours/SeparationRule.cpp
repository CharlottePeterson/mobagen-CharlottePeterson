#include "SeparationRule.h"
#include "imgui.h"
#include <glm/glm.hpp>

glm::vec2 SeparationRule::computeForce(const std::vector<BoidView>& neighborhood, const BoidView& boid) {
  glm::vec2 separatingForce(0.f); //# Holds the kinematic data for the character

  // the header have the desiredMinimalDistance member variable, which is the distance that the boids should try to maintain from each other.
  // glm::length(vec) returns the length of a vector,
  // glm::normalize(vec) returns the normalized vector (length 1) in the same direction as vec.
  // multiply by (desiredMinimalDistance / distance) is the proportionality factor that makes the force stronger when the boids are closer together, and weaker when they are farther apart.

  // begin solution

  if (neighborhood.empty())
  {
   return separatingForce;
  }

  for (auto neighbor : neighborhood)
  {
    glm::vec2 dist(0, 0);
    dist = boid.position - neighbor.position;
    float length;
    length = dist.length();
    glm::vec2 repulsion = glm::normalize(dist) * length;
    separatingForce += repulsion;
  }
 separatingForce =glm::normalize(separatingForce);
  separatingForce *= weight*100;
  // end solution

  return separatingForce;
}

bool SeparationRule::drawImguiRuleExtra() {
  bool valueHasChanged = false;
  if (ImGui::DragFloat("Desired Separation", &desiredMinimalDistance, 0.05f)) {
    valueHasChanged = true;
  }
  return valueHasChanged;
}

/*
* * 1. Find all neighboring boids within the separation radius (excluding the boid itself)
   * 2. For each neighbor, calculate a repulsion force inversely proportional to distance
   * 3. The force direction should point away from each neighbor (from neighbor to current boid)
   * 4. The force magnitude should be k/distance, where k is the separation strength parameter
   * 5. Sum all individual separation forces to get the total separation force
   * 6. If the total force magnitude exceeds maxForce, clamp it to maxForce while preserving direction
   * 7. Return the final separation force vector
   *
   * Important notes:
   * - Force is inversely proportional to distance: closer boids create stronger repulsion
   * - Avoid division by zero when distance is very small
   * - The maxForce parameter prevents excessive forces that could destabilize the simulation
   *
   * If no neighbors are found within the radius, return a zero force vector.
 */