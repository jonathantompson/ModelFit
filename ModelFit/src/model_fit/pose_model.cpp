#include "model_fit/pose_model.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

namespace model_fit {
  
  jtil::data_str::Vector<renderer::BoundingSphere*> PoseModel::g_b_spheres;

}  // namespace hand_fit
