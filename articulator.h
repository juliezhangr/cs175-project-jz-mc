#ifndef ARTICULATOR_H
#define ARTICULATOR_H

#include <vector>
#include <iostream>

#include "scenegraph.h"
#include "asstcommon.h"
#include "particle.h"
#include "quat.h"
#include "rigtform.h"

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
      newConstraint.restLength = sqrt(abs(dot(lastPos - currPos, lastPos - currPos)));

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
    shared_ptr<SgGeometryShapeNode> GeoPtr = 
      dynamic_pointer_cast<SgGeometryShapeNode>(shapeNode.shared_from_this());
    // save parent particle id number
    int parentPartId = particleStack_.back();

    // get relative positioning data
    RigTForm lastRbt = rbtStack_.back();
    Cvec3 translation = GeoPtr->getTranslation();
    Cvec3 angles = GeoPtr->getAngles();
    Cvec3 scales = GeoPtr->getScale();
    Quat rot = Quat::makeXRotation(angles[0]) * 
               Quat::makeYRotation(angles[1]) * 
               Quat::makeZRotation(angles[1]);
    RigTForm Q = RigTForm(translation, rot);

    // get vertex data
    std::tr1::shared_ptr<Geometry> g = GeoPtr->getGeometry();
    int vbolen, ibolen;
    g->getVboIbolen(vbolen, ibolen);
    
    typedef struct vertex {
      Cvec3 absPos;
      int particle_id;
    } vertex;

    // create array of vertex data
    vertex vertices[vbolen];

    // get raw VBO and IBO from geometry
    VertexPN *vs = g->getVbo();
    unsigned short *ibo = g->getIbo();

    for (int i = 0; i<vbolen; ++i) {
      // calculate world coordinates of each vertex
      Cvec3 v = Cvec3(lastRbt * Q * Cvec4((double)vs[i].p[0]*scales[0], 
                                          (double)vs[i].p[1]*scales[1], 
                                          (double)vs[i].p[2]*scales[2], 0));
      
      // check for duplicate vertices
      // if found, use its particle id and don't add the particle to the stack
      bool foundDup = false;
      for (int k = 0; k < i; k++) {
        if (vertices[k].absPos == v) {
          vertices[i].absPos = v;
          vertices[i].particle_id = vertices[k].particle_id;
          foundDup = true;
          break;
        }
      }
      if (!foundDup) {
        // else add vertex as a particle
        vertices[i].absPos = v;
        vertices[i].particle_id = idCounter_;
        particles_.push_back(Particle(v, v, 1));
        particleStack_.push_back(idCounter_);

        // add constraint from parent to vertex (only 4 needed)
        if (i < 4) {
          Constraint newConstraint;
          newConstraint.particleA = parentPartId;
          newConstraint.particleB = idCounter_;
          newConstraint.restLength = sqrt(abs(dot(lastRbt.getTranslation() - v,
                                                  lastRbt.getTranslation() - v)));
          constraints_.push_back(newConstraint);
        }
        idCounter_++;
      }
    }


    // create inter-triangle constraints
    for (int j=0; j<ibolen-2; j+=3) {
      vertex A = vertices[(int)ibo[j]];
      vertex B = vertices[(int)ibo[j+1]];
      vertex C = vertices[(int)ibo[j+2]];

      Constraint C1, C2, C3;
      // given triangle ABC, constraints are A->B, B->C, C->A.
      C1.particleA = A.particle_id;
      C1.particleB = B.particle_id;
      C1.restLength = sqrt(abs(dot(A.absPos - B.absPos, A.absPos - B.absPos)));

      C2.particleA = B.particle_id;
      C2.particleB = C.particle_id;
      C2.restLength = sqrt(abs(dot(B.absPos - C.absPos, B.absPos - C.absPos)));

      C3.particleA = C.particle_id;
      C3.particleB = A.particle_id;
      C3.restLength = sqrt(abs(dot(C.absPos - A.absPos, C.absPos - A.absPos)));
      
      constraints_.push_back(C1);
      constraints_.push_back(C2);
      constraints_.push_back(C3);
    }  
    return true;
  }

  virtual bool postVisit(SgShapeNode& shapeNode) {
    particleStack_.pop_back();
    return true;
  }
};

#endif



