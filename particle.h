/* Sample code for physics simulation
 * Written with guidance from Thomas Jakobsen's Advanced Character
 * Physics article. 
 * <http:/www.gamasutra.com/resource_guide/20030121/jacobson_01.html>
 */

#include "cvec.h"
#include <vector>


using namespace std;
using namespace std::tr1;

class Particle {
 public:
  Cvec3    x_;    // Current positions
  Cvec3    oldx_; // Previous positions
  float    invm;     // inverse mass of particle

 Particle(const Cvec3& x, const Cvec3& ox, const float invmass) :
  x_(x[0], x[1], x[2]), oldx_(ox[0], ox[1], ox[2]) {
    invm = invmass;
  }
};


class ParticleSystem {
  vector<Particle> p_;     // Particle vector 
  vector<Cvec3>    m_a;    // Force accumulators
  Cvec3    m_vGravity;            // Gravity
  float      m_fTimeStep;
  int  NUM_PARTICLES;
public:
   void       TimeStep();
   ParticleSystem(vector<Particle> ps, 
                  const Cvec3& g, const float ts) {
     int NUM_PARTICLES = ps.size();
     p_ = ps;
     m_vGravity = g; 
     m_fTimeStep = ts;
   };

private:
   void       Verlet();
   void       SatisfyConstraints();
   void       AccumulateForces();

};

// Verlet integration step
void ParticleSystem::Verlet() {
   for(int i=0; i<NUM_PARTICLES; i++) {
      Cvec3& x = p_[i].x_;
      Cvec3 temp = x;
      Cvec3& oldx = p_[i].oldx_;
      Cvec3& a = m_a[i];
      x += x-oldx+a*m_fTimeStep*m_fTimeStep;
      oldx = temp;
   }
}
// This function should accumulate forces for each particle
void ParticleSystem::AccumulateForces()
{
// All particles are influenced by gravity
   for(int i=0; i<NUM_PARTICLES; i++) 
     m_a[i] = m_vGravity;
}
// Here constraints should be satisfied
//void ParticleSystem::SatisfyConstraints() {
// Ignore this function for now
//}
/*
// Implements cloth simulation
struct Constraint {
   int particleA, particleB;
   float restlength;
};
// Assume that an array of constraints, m_constraints, exists
   void ParticleSystem::SatisfyConstraints() {
      for(int j=0; j<NUM_ITERATIONS; j++) {
         for(int i=0; i<NUM_CONSTRAINTS; i++) {
         Constraint& c = m_constraints[i];
         Cvec3& x1 = p_[c.particleA].x_;
         Cvec3& x2 = p_[c.particleB].x_;
         Cvec3 delta = x2-x1;
         float deltalength = sqrt(delta*delta);
         float diff=(deltalength-c.restlength)/deltalength;
         x1 -= delta*0.5*diff;
         x2 += delta*0.5*diff;
      }
      // Pseudo-code to satisfy (C2) for different mass particles (invmass = 0 is immovable)
      delta = x2-x1;
      deltalength = sqrt(delta*delta);
      diff = (deltalength-restlength)
      /(deltalength*(x1.invm + x2.invm));
      x1 -= x1.invm*delta*diff;
      x2 += x2.invm*delta*diff;

   }
}
*/

// Implements particles in a box, no bouncing
void ParticleSystem::SatisfyConstraints() {
   for(int i=0; i<NUM_PARTICLES; i++) { // For all particles
      Cvec3& x = p_[i].x_;
      x = vmin(vmax(x, Cvec3(-5,-5,-5)),
         Cvec3(5,5,5));
   }
}
/*
// Implements simulation of a stick in a box
void ParticleSystem::SatisfyConstraints() {
   for(int j=0; j<NUM_ITERATIONS; j++) {
         // First satisfy (C1)
         for(int i=0; i<NUM_PARTICLES; i++) { // For all particles
            Cvec3&             x = m_x[i];
               x = vmin(vmax(x, Cvec3(0,0,0)),
                  Cvec3(1000,1000,1000));
         }

         // Then satisfy (C2)
         Cvec3& x1 = m_x[0];
         Cvec3& x2 = m_x[1];
         Cvec3 delta = x2-x1;
         float deltalength = sqrt(delta*delta);
         float diff = (deltalength-restlength)/deltalength;
         x1 -= delta*0.5*diff;
         x2 += delta*0.5*diff;
   }
}
*/
void ParticleSystem::TimeStep() {
   AccumulateForces();
   Verlet();
   SatisfyConstraints();
}
