#include <iostream>
#include "math/common_optimization.h"

using std::cout;
using std::endl;

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

namespace jtil {
namespace math {

template <typename T>
  SwarmNode<T>::SwarmNode() {
    vel = NULL;
    pos = NULL;
    best_pos = NULL;
  }

  template <typename T>
  SwarmNode<T>::~SwarmNode() {
    SAFE_DELETE_ARR(vel);
    SAFE_DELETE_ARR(pos);
    SAFE_DELETE_ARR(best_pos);
  }

  template <typename T>
  void SwarmNode<T>::resize(uint32_t size) {
    SAFE_DELETE_ARR(vel);
    SAFE_DELETE_ARR(pos);
    SAFE_DELETE_ARR(best_pos);

    vel = new T[size];
    pos = new T[size];
    best_pos = new T[size];
    residue = std::numeric_limits<T>::infinity();
    best_residue = std::numeric_limits<T>::infinity();
  }

};  // namespace math
};  // namespace jtil

// Explicit template instantiation
template struct jtil::math::SwarmNode<float>;
template struct jtil::math::SwarmNode<double>;