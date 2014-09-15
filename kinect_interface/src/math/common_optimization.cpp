#include <iostream>
#include "math/common_optimization.h"

using std::cout;
using std::endl;

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

namespace jtil {
namespace math {

  SwarmNode::SwarmNode() {
    vel = NULL;
    pos = NULL;
    best_pos = NULL;
  }

  SwarmNode::~SwarmNode() {
    SAFE_DELETE_ARR(vel);
    SAFE_DELETE_ARR(pos);
    SAFE_DELETE_ARR(best_pos);
  }

  void SwarmNode::resize(uint32_t size) {
    SAFE_DELETE_ARR(vel);
    SAFE_DELETE_ARR(pos);
    SAFE_DELETE_ARR(best_pos);

    vel = new float[size];
    pos = new float[size];
    best_pos = new float[size];
    residue = std::numeric_limits<float>::infinity();
    best_residue = std::numeric_limits<float>::infinity();
  }

};  // namespace math
};  // namespace jtil
