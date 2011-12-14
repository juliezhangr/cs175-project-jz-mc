/* Sample code for physics simulation
 * Written with guidance from Thomas Jakobsen's Advanced Character
 * Physics article. 
 * <http:/www.gamasutra.com/resource_guide/20030121/jacobson_01.html>
 */

#ifndef PARTICLE_H
#define PARTICLE_H

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

// Implements cloth simulation
struct Constraint {
  int particleA, particleB;
  float restLength;
};

class ParticleSystem {
  vector<Particle>    p_;              // Particle vector 
  vector<Cvec3>       m_a;             // Force accumulators
  vector<Constraint>  m_constraints;   // Constraints
  Cvec3               m_vGravity;      // Gravity
  float               m_fTimeStep;     // Duration of timestep
  int                 num_iterations;  // Pre-defined number of iterations in 'relaxation' process.

public:   
  void               constrain(int particleIdA, int particleIdB);
  void               TimeStep();
  void               setFixedParticle(int particleId);
  void               changeGravity(Cvec3 change);
  vector<Particle>&  getParticleVector();

  ParticleSystem(vector<Particle> ps, vector<Constraint> cs, const Cvec3& g, const float ts) {
    p_ = ps;
    m_constraints = cs;
    m_vGravity = g; 
    m_fTimeStep = ts;
    num_iterations = 50;
  };

private:
  void Verlet();
  void SatisfyConstraints();
  void AccumulateForces();

};

// Verlet integration step
void ParticleSystem::Verlet() {
  int num_particles = p_.size();
  for(int i=0; i < num_particles - 1; i++) {
    if(p_[i].invm == 0)
      continue;
    
    Cvec3& x = p_[i].x_;
    Cvec3& oldx = p_[i].oldx_;

    Cvec3 temp = x;
    Cvec3 a = m_a[i];


    x += x - oldx + a * m_fTimeStep * m_fTimeStep;
    oldx = temp;
  }
}

// This function should accumulate forces for each particle
void ParticleSystem::AccumulateForces()
{
  Cvec3 curr_a;
  int num_particles = p_.size();

  m_a.clear();

  // All particles are influenced by gravity
  for(int i=0; i < num_particles; i++) {
    // calculate acceleration of particle given forces
    curr_a = Cvec3(0,0,0);

    // acceleration due to gravity is constant unless particle is immovable
    curr_a += (p_[i].invm == 0 ? Cvec3(0,0,0) : m_vGravity);

    m_a.push_back(curr_a);
  }
}


void ParticleSystem::SatisfyConstraints() {
  int num_particles = p_.size();
  int num_constraints = m_constraints.size();

  // iteratively converge upon state satisfying constraints
  for (int i = 0; i < num_iterations; i++) {

    // ensure contained in box
    for(int j = 0; j < num_particles; j++) {
      Cvec3& x = p_[j].x_;
      x = vmin(vmax(x, Cvec3(-5,-5,-5)),
               Cvec3(5,5,5));
    }
    
    // ensure distance constraints are satisfied
    for(int k = 0; k < num_constraints; k++) {
      Constraint& c = m_constraints[k];
      Cvec3& x1 = p_[c.particleA].x_;
      Cvec3& x2 = p_[c.particleB].x_;
      Cvec3 delta = x1 - x2;
      float deltaLength = sqrt(dot(delta, delta));
      float diff = (deltaLength - c.restLength) / deltaLength;
      
      x1 -= delta * p_[c.particleA].invm * 0.2 * diff;
      x2 += delta * p_[c.particleB].invm * 0.2 * diff;
      }
  }
}

void ParticleSystem::constrain(int particleAId, int particleBId) {
  Constraint constraint;
  constraint.particleA = particleAId;
  constraint.particleB = particleBId;
  constraint.restLength = sqrt(abs(dot(p_[constraint.particleA].x_ - p_[constraint.particleB].x_, p_[constraint.particleA].x_ - p_[constraint.particleB].x_)));
  m_constraints.push_back(constraint);
}

void ParticleSystem::TimeStep() {
  AccumulateForces();
  Verlet();
  SatisfyConstraints();
}

vector<Particle>& ParticleSystem::getParticleVector() {
  return p_;
}

void ParticleSystem::setFixedParticle(int particleId) {
  int num_particles = p_.size();
  for(int i = 0; i < num_particles; i++) {
    if(i == particleId) {
      p_[i].invm = 0;
    }
    else {
      p_[i].invm = 1;
    }
  }
}

void ParticleSystem::changeGravity(Cvec3 change) {
  m_vGravity += change;
  printf("%f, %f, %f\n", m_vGravity[0], m_vGravity[1], m_vGravity[2]);
}

#endif
