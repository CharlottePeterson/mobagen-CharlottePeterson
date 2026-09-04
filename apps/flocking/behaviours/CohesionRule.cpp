#include "CohesionRule.h"
#include <glm/glm.hpp>

glm::vec2 CohesionRule::computeForce(const std::vector<BoidView>& neighborhood, const BoidView& boid) {
  glm::vec2 cohesionForce(0.f);

  // glm::length(vec) returns the length of a vector,
  // glm::normalize(vec) returns the normalized vector (length 1) in the same direction as vec.

  // begin solution

  if (neighborhood.empty() ) //if theres no neighbors
  {
      return cohesionForce;
  }

  glm::vec2 CenterOfGroup(0.0,0.0);


  for (auto neighbor : neighborhood) //adds together the position of all of the neighbors
  {
      CenterOfGroup += neighbor.position;
  }

  CenterOfGroup/=neighborhood.size();
  //i think i need to redo this part
  //float Mag=glm::length(glm::vec2(CenterOfGroup+boid.position));

 // cohesionForce += glm::normalize(glm::vec2(CenterOfGroup-boid.position)*(Mag*this->weight));
  cohesionForce += glm::vec2(CenterOfGroup-boid.position)*(this->weight);

  cohesionForce= glm::normalize(cohesionForce);
  cohesionForce*=this->weight*100;
  return cohesionForce;
  // end solution
}

/*
* 1. Find all neighboring boids within the cohesion radius (excluding the boid itself)
* 2. Calculate the center of mass (average position) of these neighbors
* 3. Compute a force vector pointing from the boid's current position toward the center of mass
* 4. Normalize this force vector to get the direction
* 5. Scale the normalized force by the cohesion strength parameter 'k' and return it
*/