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

  glm::vec2 neighboidVelocity(0.f,0.f);

  for (auto neighbor : neighborhood) //find all neighboring boids
  {
    //calculate average velocity of all neighbors


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

  // end solution
}