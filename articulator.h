#ifndef ARTICULATOR_H
#define ARTICULATOR_H

#include <vector>

#include "scenegraph.h"
#include "asstcommon.h"
#include "particle.h"

class Articulator : public SgNodeVisitor {
protected:
  vector<Particle>& particles_;
  std::vector<RigTForm> rbtStack_;
  int idCounter_;
public:
  Articulator(const RigTForm& initialRbt, vector<Particle>& particleVec) 
    : particles_(particleVec),
      rbtStack_(1, initialRbt),
      idCounter_(0) {}

  virtual bool visit(SgTransformNode& node) {
    rbtStack_.push_back(rbtStack_.back() * node.getRbt());
    
    // create new particle
    Cvec3 currPos = rbtStack_.back().getTranslation();
    float invmass = 1;

    assert(particles_.size() == idCounter_); // DEBUG
    particles_.push_back(Particle(currPos, currPos, invmass));
    
    // remember this node's particle
    node.setParticleId(idCounter_);
    idCounter_++;

    return true;
  }

  virtual bool postVisit(SgTransformNode& node) {
    rbtStack_.pop_back();
    return true;
  }

  virtual bool visit(SgShapeNode& shapeNode) {
    // TODO: Shape vertex particles!
    // for each vertex, should create a particle
    // for each edge, should create a constraint

    return true;
  }

  virtual bool postVisit(SgShapeNode& shapeNode) {
    return true;
  }
};

#endif



