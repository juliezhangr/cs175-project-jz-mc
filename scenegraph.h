#ifndef SCENEGRAPH_H
#define SCENEGRAPH_H

#include <vector>
#include <memory>
#include <stdexcept>
#if __GNUG__
#   include <tr1/memory>
#endif

#include "matrix4.h"
#include "rigtform.h"
#include "glsupport.h" // for Noncopyable

struct ShaderState;
class SgNodeVisitor;

class SgNode : public std::tr1::enable_shared_from_this<SgNode>, Noncopyable {
public:
  virtual bool accept(SgNodeVisitor& vistor) = 0;
  virtual ~SgNode() {}

  // Two nodes are equal if and only if they're the same, i.e.,
  // having the same in memory address
  bool operator == (const SgNode& other) const {
    return this == &other;
  }

  bool operator != (const SgNode& other) const {
    return !(*this == other);
  }

protected:
  SgNode() {}
};

//
// A transform node can have descendents nodes. It uses a
// rigid body transform to represent its frame with respect to
// the parent frame
//
class SgTransformNode : public SgNode {
public:
  virtual bool accept(SgNodeVisitor& visitor);
  virtual RigTForm getRbt() = 0;

  void addChild(std::tr1::shared_ptr<SgNode> child);
  void removeChild(std::tr1::shared_ptr<SgNode> child);

  int getNumChildren() const {
    return children_.size();
  }

  std::tr1::shared_ptr<SgNode> getChild(int i) {
    return children_[i];
  }

private:
  std::vector<std::tr1::shared_ptr<SgNode> > children_;
};

//
// A shape node has no decendents, but can perform drawing. It can also
// provide an affine matrix to be used to modify its geometry.
//
class SgShapeNode : public SgNode {
public:
  virtual bool accept(SgNodeVisitor& visitor);

  virtual Matrix4 getAffineMatrix() = 0;
  virtual void draw(const ShaderState& curSS) = 0;
};


class SgNodeVisitor {
public:
  virtual bool visit(SgTransformNode& node) { return true; }
  virtual bool visit(SgShapeNode& node) { return true; }

  virtual bool postVisit(SgTransformNode& node) { return true; }
  virtual bool postVisit(SgShapeNode& node) { return true; }
};


RigTForm getPathAccumRbt(
  std::tr1::shared_ptr<SgTransformNode> source,
  std::tr1::shared_ptr<SgTransformNode> destination,
  int offsetFromDestination = 0);


//----------------------------------------------------
// Concrete scene graph node implementations follow
//----------------------------------------------------

// A SgRoot node is a Transform node with identity Rbt
class SgRootNode : public SgTransformNode {
public:
  SgRootNode() {}

  virtual RigTForm getRbt() {
    return RigTForm();
  }
};

// A SgRbtNode is a Transform node that wraps a RigTForm
class SgRbtNode : public SgTransformNode {
public:
  SgRbtNode(const RigTForm& rbt = RigTForm())
    : rbt_ (rbt) {}

  virtual RigTForm getRbt() {
    return rbt_;
  }

  void setRbt(const RigTForm& rbt) {
    rbt_ = rbt;
  }

private:
  RigTForm rbt_;
};

#endif