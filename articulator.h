#ifndef ARTICULATOR_H
#define ARTICULATOR_H

#include <vector>

#include "scenegraph.h"
#include "asstcommon.h"
#include "particle.h"

class Articulator : public SgNodeVisitor {
protected:
  vector<Particle>& particles_;
  vector<Constraint>& constraints_;
  std::vector<RigTForm> rbtStack_;
  std::vector<int> particleStack_;
  int idCounter_;
public:
  Articulator(const RigTForm& initialRbt, vector<Particle>& particleVec,
                                          vector<Constraint>& constraintVec) 
    : particles_(particleVec),
      constraints_(constraintVec),
      rbtStack_(1, initialRbt),
      particleStack_(),
      idCounter_(0) {}

  virtual bool visit(SgTransformNode& node) {
    RigTForm lastRbt = rbtStack_.back();
    RigTForm currRbt = lastRbt * node.getRbt();

    rbtStack_.push_back(currRbt);
    
    // create new particle
    Cvec3 lastPos = lastRbt.getTranslation();
    Cvec3 currPos = currRbt.getTranslation();

    float invmass = 1;
    particles_.push_back(Particle(currPos, currPos, invmass));
    
    // remember this node's particle
    node.setParticleId(idCounter_);
    
    Constraint newConstraint;

    // if not first particle, create a constraint with this as destination
    //if(particleStack_.size() > 0) {
    //  newConstraint.particleA = particleStack_.back();
    //  newConstraint.particleB = idCounter_;
    //  
    //  // calculate the initial distance between these particles
    //  newConstraint.restLength = sqrt(abs(dot(lastPos - currPos, lastPos - currPos)));

    //  constraints_.push_back(newConstraint);
    //}
    
    particleStack_.push_back(idCounter_);
    idCounter_++;
    return true;
  }

  virtual bool postVisit(SgTransformNode& node) {
    rbtStack_.pop_back();
    particleStack_.pop_back();
    return true;
  }

  virtual bool visit(SgShapeNode& shapeNode) {
    // TODO: Shape vertex particles!
    // for each vertex, should create a particle
    // for each edge, should create a constraint

    RigTForm lastPos = rbtStack_.back().getTranslation();
    Matrix4 T = shapeNode.getAffineMatrix();

    // get vertex data
    std::tr1::shared_ptr<Geometry> g = shapeNode.getGeometry();
    int vbolen, ibolen;
    g->getVboIbolen(vbolen, ibolen);
    
    typedef struct {
      Cvec3 v;
      int particle_id;
    } vertex;
    vertex vertices[vbolen];

    VertexPN *vs = g->getVbo();
    unsigned short *ibo = g->getIbo();

    for (int i = 0; i<vbolen; ++i) {
      vertices[i].v = Cvec3((double)vs[i].p[0], (double)vs[i].p[1], (double)vs[i].p[2]);

      // add vertex as a particle
      //Cvec3 currPos = lastPos + T * relpos;
      vertices[i].particle_id = idCounter_;
      particles_.push_back(Particle(currPos, currPos, 1));

      idCounter_++;
    }

    for (int i=0; i<ibolen-2; i+3) {
      vertex A = vertices[ibo[i]];
      vertex B = vertices[ibo[i+1]];
      vertex C = vertices[ibo[i+2]];

      Constraint C1, C2, C3;
      // if not first particle, create constraint
      // given triangle ABC, constraints are AB, BC, AC.
       if(particleStack_.size() > 0) {
        C1.particleA = A.particle_id;
        C1.particleB = B.particle_id;
        C1.restLength = sqrt(norm2(A.v - B.v));

        C2.particleA = B.particle_id;
        C2.particleB = C.particle_id;
        C3.restLength = sqrt(norm2(B.v - C.v));

        C3.particleA = A.particle_id;
        C3.particleB = C.particle_id;
        C3.restLength = sqrt(norm2(A.v - C.v));
      
        constraints_.push_back(C1);
        constraints_.push_back(C2);
        constraints_.push_back(C3);
      }
    }  

    return true;
  }

  virtual bool postVisit(SgShapeNode& shapeNode) {
    return true;
  }
};

#endif



