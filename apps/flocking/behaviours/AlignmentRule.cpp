#include "AlignmentRule.h"
#include <glm/glm.hpp>

glm::vec2 AlignmentRule::computeForce(const std::vector<BoidView>& neighborhood, const BoidView& boid) {
  glm::vec2 averageVelocity(0.f);
  // glm::vec2 can be divided by a float, which will divide each component of the vector by that float.

  // begin solution
  if (neighborhood.empty()) //has no neighbors
  {
    return averageVelocity;
  }

  for (auto neighbor : neighborhood) //find all neighboring boids
  {
    //calculate average velocity of all neighbors

    glm::vec2 neighboidVelocity(0.f,0.f);
    for (auto neighboor : neighborhood)
    {
      neighboidVelocity.x += neighbor.velocity.x;
      neighboidVelocity.y += neighbor.velocity.y;
    }
    neighboidVelocity.x /= (neighborhood.size() + 0.00001);
    neighboidVelocity.y /= (neighborhood.size() + 0.00001);

    averageVelocity = glm::normalize(neighboidVelocity);
    averageVelocity *= weight*100;
    return averageVelocity;
    //Compute a force vector that represents the desired velocity change

  }
  //also include boid's on velocity

  // * 4. Scale this force by the alignment strength parameter 'k'

  //* 5. Return the scaled average velocity as the alignment force
  return averageVelocity;
  // end solution
}

   /**
   * Computes the alignment force for a given boid.
   *
   * Alignment is the tendency of boids to align their velocity with their neighbors.
   * This function should:
   * 1. Find all neighboring boids within the alignment radius (including the boid itself)
   * 2. Calculate the average velocity of these neighbors
   * 3. Compute a force vector that represents the desired velocity change
   * 4. Scale this force by the alignment strength parameter 'k'
   * 5. Return the scaled average velocity as the alignment force
   *
   * Note: Unlike cohesion and separation, alignment typically includes the boid itself
   * in the neighbor calculation, as it contributes to the local velocity average.
   *
   * If no neighbors are found within the radius, return a zero force vector.
   *
   * @param boids The complete list of all boids in the simulation
   * @param boidAgentIndex The index of the boid for which to compute the alignment force
   * @return Vector2 The alignment force to be applied to the boid
   */