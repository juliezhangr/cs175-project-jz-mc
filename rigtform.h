#ifndef RIGTFORM_H
#define RIGTFORM_H

#include <iostream>
#include <cassert>

#include "matrix4.h"
#include "quat.h"

class RigTForm {
  Cvec3 t_; // translation component
  Quat r_;  // rotation component represented as a quaternion

public:
  RigTForm() : t_(0) {
    assert(norm2(Quat(1,0,0,0) - r_) < CS175_EPS2);
  }

  RigTForm(const Cvec3& t, const Quat& r) : t_(t), r_(r)  {}
  RigTForm(const Cvec3& t) : t_(t), r_()                  {}    // only set translation part (rotation is identity)
  RigTForm(const Quat& r) : t_(0), r_(r)                  {}    // only set rotation part (translation is 0)

  Cvec3 getTranslation() const {
    return t_;
  }

  Quat getRotation() const {
    return r_;
  }

  RigTForm& setTranslation(const Cvec3& t) {
    t_ = t;
    return *this;
  }

  RigTForm& setRotation(const Quat& r) {
    r_ = r;
    return *this;
  }

  Cvec4 operator * (const Cvec4& a) const {
    return Cvec4(t_, 0.0) * a[3] + r_ * a;
  }

  RigTForm operator * (const RigTForm& a) const {
    return RigTForm(t_ + Cvec3(r_ * Cvec4(a.t_, 0)), r_*a.r_);
  }
};

inline RigTForm inv(const RigTForm& tform) {
  Quat invRot = inv(tform.getRotation());
  return RigTForm(Cvec3(invRot * Cvec4(-tform.getTranslation(), 1)), invRot);
}

inline RigTForm transFact(const RigTForm& tform) {
  return RigTForm(tform.getTranslation());
}

inline RigTForm linFact(const RigTForm& tform) {
  return RigTForm(tform.getRotation());
}

inline Matrix4 rigTFormToMatrix(const RigTForm& tform) {
  Matrix4 m = quatToMatrix(tform.getRotation());
  const Cvec3 t = tform.getTranslation();
  for (int i = 0; i < 3; ++i) {
    m(i, 3) = t(i);
  }
  return m;
}

inline RigTForm lerp(const RigTForm& tform0, const RigTForm& tform1, double t) {
  return RigTForm(lerp(tform0.getTranslation(), tform1.getTranslation(), t),
                  slerp(tform0.getRotation(), tform1.getRotation(), t));
}

#endif
