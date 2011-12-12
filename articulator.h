#ifndef ARTICULATOR_H
#define ARTICULATOR_H

#include <vector>

#include "scenegraph.h"
#include "asstcommon.h"

class Articulator : public SgNodeVisitor {
protected:
  vector<Particle>& particles_;
  int idCounter_;
public:
  Articulator(vector<Particle>& particleVec) 
    : particles_(particleVec),
      idCounter_(0) {}

  virtual bool visit(SgTransformNode& node) {
    // create new particle
    // (add to vector of particles)
    //particles_[idCounter_] = Particle(0,0,0);

    // remember this node's particle
    node.setParticleId(idCounter_);
    idCounter_++;

    return true;
  }

  virtual bool postVisit(SgTransformNode& node) {
    return true;
  }

  virtual bool visit(SgShapeNode& shapeNode) {
    return true;
  }

  virtual bool postVisit(SgShapeNode& shapeNode) {
    return true;
  }
};

#endif



