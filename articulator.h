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
    shared_ptr<SgGeometryShapeNode> GeoPtr = 
      dynamic_pointer_cast<SgGeometryShapeNode>(shapeNode.shared_from_this());

    // get relative positioning data
    Cvec3 lastPos = rbtStack_.back().getTranslation();
    Cvec3 translation = GeoPtr->getTranslation();
    Cvec3 angles = GeoPtr->getAngles;
    Cvec3 scales = GeoPtr->getScale;
    Quat rot = makeXRotation(angles[0]) * makeYRotation(angles[1]) * makeZRotation(angles[1]);
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
      Cvec3 v = lastPos + Q * Cvec3((double)vs[i].p[0]*scales[0], 
                            (double)vs[i].p[1]*scales[1], 
                            (double)vs[i].p[2]*scales[2]);
      
      // check for duplicate vertices
      // if found, use its particle id and don't add the particle to the stack
      for (int k = 0; k < i; k++) {
        if (vertices[k].absPos == v) {
          vertices[i].absPos = v;
          vertices[i].particle_id = vertices[k].particle_id;
          return;
        }
      }
      // else add vertex as a particle
      vertices[i].absPos = v;
      vertices[i].particle_id = idCounter_;
      particles_.push_back(Particle(vertices[i].absPos, vertices[i].absPos, 1));
      idCounter_++;
      
    }

    // create inter-triangle restraints
    for (int i=0; i<ibolen-2; i+3) {
      vertex A = vertices[ibo[i]];
      vertex B = vertices[ibo[i+1]];
      vertex C = vertices[ibo[i+2]];

      Constraint C1, C2, C3;
      // if not first particle, create constraint
      // given triangle ABC, constraints are AB, BC, AC.
       if (particleStack_.size() > 0) {
        C1.particleA = A.particle_id;
        C1.particleB = B.particle_id;
        C1.restLength = sqrt(norm2(A.absPos - B.absPos));

        C2.particleA = B.particle_id;
        C2.particleB = C.particle_id;
        C3.restLength = sqrt(norm2(B.absPos - C.absPos));

        C3.particleA = A.particle_id;
        C3.particleB = C.particle_id;
        C3.restLength = sqrt(norm2(A.absPos - C.absPos));
      
        constraints_.push_back(C1);
        constraints_.push_back(C2);
        constraints_.push_back(C3);
      }
    }  
    
    return true;
  }

  virtual bool postVisit(SgShapeNode& shapeNode) {
    particleStack.pop_back();
    return true;
  }
};

#endif



