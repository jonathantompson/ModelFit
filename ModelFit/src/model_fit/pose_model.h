//
//  pose_shape.h
//
//  Created by Jonathan Tompson on 10/26/12.
//
//  Base container class for all the hand geometry

#pragma once

#include "model_fit/bounding_sphere.h"
#include "data_str/vector.h"
#include "math/common_optimization.h"  // CoeffUpdateFuncPtr

namespace renderer { class Geometry; }

namespace model_fit {

  class PoseModel {
  public:
    // Constructor / Destructor
    PoseModel() { }
    virtual ~PoseModel() { }

    // Call before rendering hand depth maps:
    virtual void updateMatrices(const float* coeff) = 0;
    virtual void updateHeirachyMatrices() = 0;
    virtual void fixBoundingSphereMatrices() = 0;
    virtual inline renderer::Geometry* scene_graph() = 0;

    virtual void renderStackReset() = 0;
    virtual renderer::Geometry* renderStackPop() = 0;
    virtual bool renderStackEmpty() = 0;

    // setRendererAttachement - called when ModelFit wants to detach the model
    // from the global renderer
    virtual void setRendererAttachement(const bool renderer_attachment) = 0;
    virtual const bool getRendererAttachement() = 0;

    // You must define some access functions for the PSO related data
    // angle_coeffs - Which of the coeffs are angles
    virtual const bool* angle_coeffs() = 0;
    // angle_coeffs - PSO search radius in each coeff search direction
    virtual const float* pso_radius_c() = 0;
    // coeff_min_limit - Minimum allowable value for each coeff
    virtual const float* coeff_min_limit() = 0;
    // coeff_max_limit - Maximum allowable value for each coeff
    virtual const float* coeff_max_limit() = 0;
    // coeff_penalty_scale - Percentage (%) of penalty to apply to each coeff
    // as it goes outside it's coeff bounds (default is 100 for each dimension)
    virtual const float* coeff_penalty_scale() = 0;
    // Maximum number of boudning sphere groups to consider
    virtual const uint32_t max_bsphere_groups() = 0;

    static jtil::data_str::Vector<renderer::BoundingSphere*> g_b_spheres;

  protected:
    // Non-copyable, non-assignable.
    PoseModel(PoseModel&);
    PoseModel& operator=(const PoseModel&);
  };
};  // namespace hand_fit
