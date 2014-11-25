#ifndef RIGTFORM_H
#define RIGTFORM_H

#include <iostream>
#include <cassert>
#include <cmath>

#include "matrix4.h"
#include "quat.h"

class RigTForm {
  Cvec3 t_; // translation component
  Quat r_;  // rotation component represented as a quaternion

public:
  RigTForm() : t_(0) {
    assert(norm2(Quat(1,0,0,0) - r_) < CS175_EPS2);
  }

  RigTForm(const Cvec3& t, const Quat& r) : t_(t), r_(r) {}      

  explicit RigTForm(const Cvec3& t) : t_(t), r_() {}

  explicit RigTForm(const Quat& r) : t_(0), r_(r) {} 

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
    return r_ * a +Cvec4(t_,0) * a[3];    
  }

  RigTForm operator * (const RigTForm& a) const {
    // ======
	// TODO:
	// ======
    return RigTForm(t_ + Cvec3(r_ * Cvec4(a.getTranslation(), 0)), r_ * a.getRotation()
    );
  }
};

inline RigTForm inv(const RigTForm& tform) {
    // ======
	// TODO:
	// ======
  Quat r_inv = inv(tform.getRotation());
  return RigTForm(Cvec3(r_inv * Cvec4(-tform.getTranslation(),1)),r_inv);
}

inline RigTForm transFact(const RigTForm& tform) {
  return RigTForm(tform.getTranslation());
}

inline RigTForm linFact(const RigTForm& tform) {
  return RigTForm(tform.getRotation());
}

inline Matrix4 rigTFormToMatrix(const RigTForm& tform) {
    // ======
	// TODO:
	// ======
  Matrix4 t = Matrix4::makeTranslation(tform.getTranslation());
  Matrix4 r = quatToMatrix(tform.getRotation());
    return t * r;
}




inline RigTForm interpolate(const RigTForm& r0, const RigTForm& r1, const double alpha) {
  return RigTForm(lerp(r0.getTranslation(),r1.getTranslation(),alpha),slerp(r0.getRotation(),r1.getRotation(),alpha));
}

inline RigTForm interpolateCatmullRom(const RigTForm& r0, const RigTForm& r1, const RigTForm& r2, const RigTForm& r3, const double alpha) {
RigTForm d,e,f,g,h,m,n;

d.setTranslation((r2.getTranslation()-r0.getTranslation())*(1.0/6.0)+r1.getTranslation());
e.setTranslation((r3.getTranslation()-r1.getTranslation())*(-1.0/6.0)+r1.getTranslation());

d.setRotation(power(cn(r2.getRotation()*inv(r0.getRotation())),1.0/6.0)*r1.getRotation());
e.setRotation(power(cn(r3.getRotation()*inv(r1.getRotation())),-1.0/6.0)*r2.getRotation());

f = interpolate(r1,d,alpha);
g = interpolate(d,e,alpha);
h = interpolate(e,r2,alpha);
m = interpolate(f,g,alpha);
n = interpolate(g,h,alpha);

return interpolate(m,n,alpha);
}



#endif
