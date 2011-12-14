#ifndef POSER_H
#define POSER_H

#include <vector>

#include "scenegraph.h"
#include "asstcommon.h"
#include "particle.h"

class Poser : public SgNodeVisitor {
protected:
  vector<Particle>& particles_;
  std::vector<RigTForm> rbtStack_;
  int idCounter_;
public:
  Poser(const RigTForm& initialRbt, vector<Particle>& particleVec) 
    : particles_(particleVec),
      rbtStack_(1, initialRbt),
      idCounter_(0) {}

  virtual bool visit(SgTransformNode& node) {

    // get accumulated RBT for parent joint in scene graph
    RigTForm parentAccumRbt = rbtStack_.back();
    rbtStack_.pop_back();

    RigTForm myAccumRbt = parentAccumRbt * node.getRbt();

    // get updated particle position
    int particleId = node.getParticleId();
    Cvec3 newPos = particles_[particleId].x_;
    Cvec3 oldPos = myAccumRbt.getTranslation();
    Cvec3 changePos = newPos - oldPos;

    Cvec3 relPos = node.getRbt().getTranslation();

    Cvec3 newVec = newPos - parentAccumRbt.getTranslation();
    Cvec3 oldVec = oldPos - parentAccumRbt.getTranslation();
    
    Cvec3 normalVec = Cvec3(1,0,0);
    double theta = 0;
    Quat rot = Quat();

    // if this is the torso, fall relative to world
    if(particleId == 0) {

      // update local RBT
      RigTForm newLocalRbt = node.getRbt();
      newLocalRbt.setTranslation(newPos);
      shared_ptr<SgRbtNode> rbtPtr = dynamic_pointer_cast<SgRbtNode>(node.shared_from_this());
      rbtPtr->setRbt(newLocalRbt);

      myAccumRbt = parentAccumRbt * newLocalRbt;
    }
    
    else if(dot(newVec,newVec) > CS175_EPS2 && dot(oldVec,oldVec) > CS175_EPS2){
      //Cvec3 changePos = newPos - oldPos;

      //// update local RBT
      //RigTForm newLocalRbt = node.getRbt();
      //newLocalRbt.setTranslation(newLocalRbt.getTranslation());
      //shared_ptr<SgRbtNode> rbtPtr = dynamic_pointer_cast<SgRbtNode>(node.shared_from_this());
      //rbtPtr->setRbt(newLocalRbt);

      newVec = newVec.normalize();
      oldVec = oldVec.normalize();
      normalVec = cross(oldVec, newVec);
     
      if(dot(normalVec,normalVec) > CS175_EPS2) {
        normalVec.normalize();
        //normalVec = Cvec3(0,1,0);
        double dotProduct = dot(oldVec, newVec);
        theta = acos(dotProduct);
        Quat rot = Quat(cos(theta/2), normalVec * sin(theta/2));
        //Quat rot = Quat(cos(CS175_PI/128.), normalVec * sin(CS175_PI/128.));
        parentAccumRbt = RigTForm(parentAccumRbt.getTranslation(), rot * parentAccumRbt.getRotation());
        myAccumRbt = parentAccumRbt * node.getRbt();
      }
    }

    //assert(abs(dot(myAccumRbt.getTranslation() - newPos, myAccumRbt.getTranslation() - newPos)) < CS175_EPS);
    //particles_[particleId].x_ = myAccumRbt.getTranslation();

    rbtStack_.push_back(parentAccumRbt);
    rbtStack_.push_back(myAccumRbt);

    idCounter_++;
    return true;
  }

  virtual bool postVisit(SgTransformNode& node) {
    RigTForm newAccumRbt = rbtStack_.back();
    rbtStack_.pop_back();

    RigTForm parentAccumRbt = rbtStack_.back();
    RigTForm currLocalRbt = node.getRbt();
    RigTForm currAccumRbt = parentAccumRbt * currLocalRbt;

    RigTForm newLocalRbt = currLocalRbt * newAccumRbt * inv(currAccumRbt);

    shared_ptr<SgRbtNode> rbtPtr = dynamic_pointer_cast<SgRbtNode>(node.shared_from_this());
    //rbtPtr->getRbt().setRotation(newLocalRbt.getRotation());
    rbtPtr->setRbt(RigTForm(currLocalRbt.getTranslation(), newLocalRbt.getRotation()));

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



