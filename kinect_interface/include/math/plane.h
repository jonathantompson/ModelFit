//
//  plane.h
//
//  Created by Jonathan Tompson on 3/14/12.
//

#pragma once

#include "math/math_types.h"

namespace jtil {
namespace math {
  template <class T>
  class quaternion;

  template <class T>
  class DATA_ALIGN(ALIGNMENT, Plane {
   public:
    Vec3<T> normal;
    T dist;  // distance below origin - the D from plane equasion Ax+By+Cz+D=0

    // Constructors
    Plane(const Vec3<T>& normal, const double dist);
    Plane();

    // Setters
    void set(const Plane& p);

    // Operations
    void flipPlane();
    //void Transform(const Vec3<T>& position, 
    //  const quaternion<T>& orientation);
    static bool coplanar(const Plane& A, const Plane& B);
  }
  );  // end DATA_ALIGN

  template <class T>
  Plane<T>::Plane(const Vec3<T>& normal, const double dist) {
    this->normal.set(normal);
    this->dist = dist;
  };

  template <class T>
  Plane<T>::Plane() {
    this->normal.zeros();
    this->dist = 0;
  };

  template <class T>
  void Plane<T>::flipPlane() {
    dist *= -1;
    normal.scale(-1);
  };

  template <class T>
  void Plane<T>::set(const Plane& p) {
    this->normal.set(p.normal);
    this->dist = p.dist;
  };

  template <class T>
  bool Plane<T>::coplanar(const Plane& A, const Plane& B) {
    if ((A.normal[0] == B.normal[0]  && A.normal[1] == B.normal[1]  && 
      A.normal[2] == B.normal[2]  && A.dist == B.dist) ||
       (A.normal[0] == -B.normal[0] && A.normal[1] == -B.normal[1] && 
       A.normal[2] == -B.normal[2] && A.dist == -B.dist))
      return true;
    else
      return false;
  };
  
  /*
  template <class T>
  void plane::Transform(const Vec3<T>& position, 
    const quaternion<T>& orientation) {
    //   Transforms the plane to the space defined by the 
    //   given position/orientation.
    static Vec3<T> newnormal;
    static Vec3<T> origin;

    newnormal = Inverse(orientation)*normal;
    origin = Inverse(orientation)*(-normal*dist - position);

    normal = newnormal;
    dist = -dot(newnormal, origin);
  }
  */

};  // namespace math
};  // namespace jtil

