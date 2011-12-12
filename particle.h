/* Sample code for physics simulation
 * Written with guidance from Thomas Jakobsen's Advanced Character
 * Physics article. 
 * <http:/www.gamasutra.com/resource_guide/20030121/jacobson_01.html>
 */

#include "cvec.h"

// using define or a dynamic vector instead?
#define NUM_PARTICLES 1

class ParticleSystem {
   Cvec3    m_x[NUM_PARTICLES];    // Current positions
   Cvec3    m_oldx[NUM_PARTICLES]; // Previous positions
   Cvec3    m_a[NUM_PARTICLES];    // Force accumulators
   Cvec3    m_vGravity;            // Gravity
   float      m_fTimeStep;
public:
   void       TimeStep();
   ParticleSystem() {};
   ParticleSystem(const Cvec3& x, const Cvec3& ox, const Cvec3& g, 
                  const float ts) {
     for(int i = 0; i < NUM_PARTICLES; i++) {
       m_x[i] = Cvec3(x[0], x[1], x[2]);
       m_oldx[i]= Cvec3(ox[0], ox[1], ox[2]);
       m_a[i] = Cvec3();
     }
     m_vGravity = g; 
     m_fTimestep = ts;
   };

private:
   void       Verlet();
   void       SatisfyConstraints();
   void       AccumulateForces();

};
// Verlet integration step
void ParticleSystem::Verlet() {
   for(int i=0; i<NUM_PARTICLES; i++) {
      Cvec3& x = m_x[i];
      Cvec3 temp = x;
      Cvec3& oldx = m_oldx[i];
      Cvec3& a = m_a[i];
      x += x-oldx+a*fTimeStep*fTimeStep;
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
         Cvec3& x1 = m_x[c.particleA];
         Cvec3& x2 = m_x[c.particleB];
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
      /(deltalength*(invmass1+invmass2));
x1 -= invmass1*delta*diff;
x2 += invmass2*delta*diff;

      // Constrain one particle of the cloth to origo
      m_x[0] = Cvec3(0,0,0);
   }
}
*/

// Implements particles in a box, no bouncing
void ParticleSystem::SatisfyConstraints() {
   for(int i=0; i<NUM_PARTICLES; i++) { // For all particles
      Cvec3& x = m_x[i];
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

