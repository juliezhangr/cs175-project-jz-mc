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
    if(particleStack_.size() > 0) {
      newConstraint.particleA = particleStack_.back();
      newConstraint.particleB = idCounter_;
      
      // calculate the initial distance between these particles
      newConstraint.restLength = sqrt(norm2(lastPos - currPos));

      constraints_.push_back(newConstraint);
    }
    
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

    RigTForm lastRbt = rbtStack_.back();
    Matrix4 T = this.getAffineMatrix();

    // get vertex data
    std::tr1::shared_ptr<Geometry> g = this.getGeometry();
    int vbolen, ibolen;
    g.getVboIboLen(vbolen, ibolen);
    Cvec3 vertices[vbolen];
    g.getVbo(vertices);
    unsigned short *ibo = g.getIbo();

    for (int i=0; i<ibolen-2; i+3) {
      float invmass = 1;
      particles_.push_back(Particle(currPos, currPos, invmass));
    
      // remember this node's particle
      node.setParticleId(idCounter_);
    
      Constraint newConstraint;
      // if not first particle, create a constraint with this as destination
      if(particleStack_.size() > 0) {
        newConstraint.particleA = particleStack_.back();
        newConstraint.particleB = idCounter_;
      
        // calculate the initial distance between these particles
        newConstraint.restLength = sqrt(norm2(lastPos - currPos));

        constraints_.push_back(newConstraint);
      }
    
      particleStack_.push_back(idCounter_);
      idCounter_++;
    }  

    return true;
  }

  virtual bool postVisit(SgShapeNode& shapeNode) {
    return true;
  }
};

#endif



