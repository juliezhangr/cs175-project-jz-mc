#ifndef POSER_H
#define POSER_H

#include <vector>

#include "scenegraph.h"
#include "asstcommon.h"
#include "particle.h"

class Poser : public SgNodeVisitor {
protected:
  vector<Particle> particles_;
  std::vector<RigTForm> rbtStack_;
  int idCounter_;
public:
  Poser(const RigTForm& initialRbt, vector<Particle>& particleVec) 
    : particles_(particleVec),
      rbtStack_(1, initialRbt),
      idCounter_(0) {}

  virtual bool visit(SgTransformNode& node) {

    // get accumulated RBT for node in scene graph
    RigTForm accumRbt = rbtStack_.back() * node.getRbt();
    rbtStack_.push_back(accumRbt);
    
    // get updated particle position
    int particleId = node.getParticleId();
    Cvec3 particlePos = particles_[particleId].x_;
    
    // current position as defined in the scene graph
    Cvec3 accumPos = accumRbt.getTranslation();
    Cvec3 relPos = node.getRbt().getTranslation();

    // update local RBT
    RigTForm newRbt = node.getRbt();
    newRbt.setTranslation(relPos + particlePos - accumPos);
    
    shared_ptr<SgRbtNode> rbtPtr = dynamic_pointer_cast<SgRbtNode>(node.shared_from_this());
    rbtPtr->setRbt(newRbt);

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



