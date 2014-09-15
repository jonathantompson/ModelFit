#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include "model_fit/model_renderer.h"
#include "model_fit/calibrate_geometry.h"
#include "renderer/open_gl_common.h"
#include "renderer/renderer.h"
#include "renderer/colors.h"
#include "renderer/geometry/geometry_manager.h"
#include "renderer/geometry/geometry.h"
#include "renderer/geometry/geometry_vertices.h"
#include "renderer/geometry/geometry_colored_mesh.h"
#include "renderer/geometry/bone_info.h"
#include "data_str/pair.h"
#include "kinect_interface_primesense/hand_model.h"  // for HandCoeffConvnet
#include "math/bfgs.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

#ifndef HAND_FIT
#error "HAND_FIT is not defined!  You need to declare it in the preprocessor"
#endif

using namespace jtil::math;
using namespace jtil::data_str;
using std::string;
using std::runtime_error;
using std::cout;
using std::endl;
using namespace renderer;
using namespace kinect_interface_primesense::hand_net;

namespace model_fit {
  float CalibrateGeometry::pso_radius_c_[CAL_GEOM_NUM_COEFF];
  jtil::math::BFGS<float>* CalibrateGeometry::solver_ = NULL;
  jtil::math::Float3* CalibrateGeometry::vq_[3] = {NULL, NULL, NULL};
  jtil::math::Float3* CalibrateGeometry::vb_[3] = {NULL, NULL, NULL};
  jtil::math::Float3 CalibrateGeometry::vmodel_[3];
  uint32_t CalibrateGeometry::num_frames_;

  CalibrateGeometry::CalibrateGeometry(const CalibrateGeometryType type) {
    type_ = type;
    scene_graph_ = new Geometry();

    switch (type) {
    case BOX:
      {
      Float3 red(1, 0, 0);
      Float3 green(0, 1, 0);
      Float3 blue(0, 0, 1);
      Float3 white(1, 1, 1);
      Float3 yellow(1, 1, 0);
      Float3 cyan(0, 1, 1);
      box_ = GeometryColoredMesh::makeCube(red, green, blue, white, yellow, 
        cyan);
      scene_graph_->addChild(box_);

      box_size_.set(BOX_SIDEA, BOX_SIDEB, BOX_SIDEC);
      updateSize();
      }
      break;
    case TENNIS:
      {
      box_ = NULL;
      Float3 tennis_ball_yellow(0.776470f, 0.929412f, 0.172549f);
      Float3 wood_brown(0.50588f, 0.48235f, 0.11372f);
      sphere_a_ = GeometryColoredMesh::makeSphere(SPHERE_NSTACKS, SPHERE_NSLICES,
        SPHERE_BASE_RADIUS, tennis_ball_yellow);
      sphere_b_ = GeometryColoredMesh::makeSphere(SPHERE_NSTACKS, SPHERE_NSLICES,
        SPHERE_BASE_RADIUS, tennis_ball_yellow);
      tennis_ball_yellow[0] *= 0.5f;
      tennis_ball_yellow[1] *= 0.5f;
      tennis_ball_yellow[2] *= 0.5f;
      sphere_c_ = GeometryColoredMesh::makeSphere(SPHERE_NSTACKS, SPHERE_NSLICES,
        SPHERE_BASE_RADIUS, tennis_ball_yellow);
#ifdef INCLUDE_STICKS
      cylinder_a_ = GeometryColoredMesh::makeCylinder(CYL_NSLICES,
        CYL_BASE_HEIGHT, CYL_BASE_RADIUS, CYL_BASE_RADIUS, wood_brown);
      cylinder_b_ = GeometryColoredMesh::makeCylinder(CYL_NSLICES,
        CYL_BASE_HEIGHT, CYL_BASE_RADIUS, CYL_BASE_RADIUS, wood_brown);
#endif

      scene_graph_->addChild(sphere_a_);
      scene_graph_->addChild(sphere_b_);
      scene_graph_->addChild(sphere_c_);
#ifdef INCLUDE_STICKS
      scene_graph_->addChild(cylinder_a_);
      scene_graph_->addChild(cylinder_b_);
#endif

      Float3 scale_vec(SPHERE_RADIUS, SPHERE_RADIUS, SPHERE_RADIUS);
      Float4x4::scaleMat(*sphere_a_->mat(), scale_vec);
      Float4x4::scaleMat(*sphere_b_->mat(), scale_vec);
      Float4x4::scaleMat(*sphere_c_->mat(), scale_vec);

      // Calculate the sphere positions in model space
      vmodel_[0].set(0, (SPHERE_A_OFST + SPHERE_RADIUS), 0);
      vmodel_[1].set(-(SPHERE_B_OFST + SPHERE_RADIUS), 0, 0);
      vmodel_[2].set((SPHERE_C_OFST + SPHERE_RADIUS), 0, 0);

      sphere_a_->mat()->leftMultTranslation(vmodel_[0]);
      sphere_b_->mat()->leftMultTranslation(vmodel_[1]);
      sphere_c_->mat()->leftMultTranslation(vmodel_[2]);

#ifdef INCLUDE_STICKS
      scale_vec.set(CYL_RADIUS, SPHERE_A_OFST + SPHERE_RADIUS, CYL_RADIUS);
      Float4x4::scaleMat(*cylinder_b_->mat(), scale_vec);
      cylinder_b_->mat()->leftMultRotateZAxis((float)M_PI_2);
      scale_vec.set(CYL_RADIUS, SPHERE_B_OFST + SPHERE_RADIUS, CYL_RADIUS);
      Float4x4::scaleMat(*cylinder_a_->mat(), scale_vec);
#endif
      }
      break;
    case ICOSAHEDRON:
      icosahedron_ = GeometryManager::g_geom_manager()->loadFromFile("./models/",
        "icosahedron.dae", false);
      scene_graph_->addChild(icosahedron_);

      // Measure a face size
      Geometry* c0 = icosahedron_->getChild(0);
      if (c0->type() != GEOMETRY_COLORED_MESH) {
        throw std::runtime_error("Icosahedron mesh is not the correct type!");
      }
      GeometryColoredMesh* c0_mesh = (GeometryColoredMesh*)c0;
      jtil::data_str::Vector<jtil::math::Float3>& vert = *c0_mesh->vertices();
      jtil::data_str::Vector<uint32_t>& ind = *c0_mesh->indices();
      Float3 vec;
      Float3::sub(vec, vert[ind[0]], vert[ind[1]]);
      float length = vec.length();
      float scale = ICOSAHEDRON_SIDE_LENGTH / length;

      for (uint32_t i = 0; i < icosahedron_->numChildren(); i++) {
        Float4x4::euler2RotMat(*icosahedron_->getChild(i)->mat(),
          -2.97f, 0.26f, -0.625f);
        icosahedron_->getChild(i)->mat()->rightMultScale(scale, scale, scale);
      }

      icosahedron_scale_ = ICOSAHEDRON_DEFAULT_SCALE; 
      updateSize();

      break;
    }

    for (uint32_t i = 0; i < 3; i++) {
      pso_radius_c_[i] = 25.0f;
    }
    for (uint32_t i = 3; i < 6; i++) {
      pso_radius_c_[i] = 0.5f;
    }

    // Adding the geometry to the gobal geometry manager's scene graph will
    // transfer ownership of the memory.
    GeometryManager::scene_graph_root()->addChild(scene_graph_);
    renderer_attachment_ = true;

  }

  CalibrateGeometry::~CalibrateGeometry() {
    // Note, ownership of all geometry is transfered to the renderer class
  }

  void CalibrateGeometry::updateSize() {
    switch (type_) {
    case BOX:
      Float4x4::scaleMat(*box_->mat(), box_size_[0]/2.0f, box_size_[1]/2.0f,
        box_size_[2]/2.0f);
      break;
    case ICOSAHEDRON:
      Float4x4::scaleMat(*icosahedron_->mat(), icosahedron_scale_, 
        icosahedron_scale_, icosahedron_scale_);
      break;
    default:
      throw std::runtime_error("updateSize() not supported for this geometry"
        " type!");
    }
  }

  void CalibrateGeometry::updateMatrices(const float* coeff) {
    coeff2Mat(*scene_graph_->mat(), coeff);
  }
  
  void CalibrateGeometry::renderStackReset() {
    render_stack_.resize(0);  // empty the stack (without deallocating)
    // Seed the render stack with the root node
    render_stack_.pushBack(scene_graph_);
  }

  Geometry* CalibrateGeometry::renderStackPop() {
    Geometry* ret = NULL;
    if (render_stack_.size() > 0) {
      render_stack_.popBackUnsafe(ret);  // Remove the last element

      // Now add the children to the geometry stack
      for (uint32_t i = 0; i < ret->numChildren(); i ++) {
        render_stack_.pushBack(ret->getChild(i));
      }
    }
    return ret;
  }

  bool CalibrateGeometry::renderStackEmpty() {
    return render_stack_.size() == 0;
  }

  void CalibrateGeometry::updateHeirachyMatrices() {
    renderStackReset();
    while (!renderStackEmpty()) {
      Geometry* cur_geom = renderStackPop();
      // Update the render matrix based on our parents position
      if (cur_geom->parent() != NULL) {
        Float4x4::multSIMD(*cur_geom->mat_hierarchy(),
          *cur_geom->parent()->mat_hierarchy(), *cur_geom->mat());
      } else {
        cur_geom->mat_hierarchy()->set(*cur_geom->mat());
      }
    }
  }

  void CalibrateGeometry::fixBoundingSphereMatrices() {
  }

  void CalibrateGeometry::setRendererAttachement(const bool renderer_attachment) {
    if (renderer_attachment_ != renderer_attachment) {
      renderer_attachment_ = renderer_attachment;
      Geometry* g = scene_graph_;
      if (!renderer_attachment_) {
        g->parent()->removeChild(g);
      } else {
        GeometryManager::g_geom_manager()->scene_graph_root()->addChild(g);
      }
    }
  }

  const bool CalibrateGeometry::getRendererAttachement() {
    return renderer_attachment_;
  }

  // These next few methods are to avoid the cos and sin double functions in 
  // the Mat4x4 template
  void CalibrateGeometry::euler2RotMatGM(Float4x4& a, const float x_angle, 
    const float y_angle, const float z_angle) {
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToMatrix/index.htm
    float c1 = cosf(x_angle);
    float s1 = sinf(x_angle);
    float c2 = cosf(y_angle);
    float s2 = sinf(y_angle);
    float c3 = cosf(z_angle);
    float s3 = sinf(z_angle);
#ifdef COLUMN_MAJOR
    a.m[0] = c1*c2;
    a.m[4] = -c1*s2*c3 + s1*s3;
    a.m[8] = c1*s2*s3 + s1*c3;
    a.m[12] = 0;
    a.m[1] = s2;
    a.m[5] = c2*c3;
    a.m[9] = -c2*s3;
    a.m[13] = 0;
    a.m[2] = -s1*c2;
    a.m[6] = s1*s2*c3 + c1*s3;
    a.m[10] = -s1*s2*s3 + c1*c3;
    a.m[14] = 0;
    a.m[3] = 0;
    a.m[7] = 0;
    a.m[11] = 0;
    a.m[15] = 1;
#endif
#ifdef ROW_MAJOR
    a.m[0] = c1*c2;
    a.m[1] = -c1*s2*c3 + s1*s3;
    a.m[2] = c1*s2*s3 + s1*c3;
    a.m[3] = 0;
    a.m[4] = s2;
    a.m[5] = c2*c3;
    a.m[6] = -c2*s3;
    a.m[7] = 0;
    a.m[8] = -s1*c2;
    a.m[9] = s1*s2*c3 + c1*s3;
    a.m[10] = -s1*s2*s3 + c1*c3;
    a.m[11] = 0;
    a.m[12] = 0;
    a.m[13] = 0;
    a.m[14] = 0;
    a.m[15] = 1;
#endif
  }

  // modulu - similar to matlab's mod()
  // result is always possitive. not similar to fmod()
  // Mod(-3,4)= 1   fmod(-3,4)= -3
#if defined(__APPLE__) || defined(_WIN32)
  float inline __fastcall Mod(float x, float y) {
    if (0 == y) {
      return x;
    }
    
    return x - y * floor(x / y);
  }
#else
  float inline Mod(float x, float y) {
    if (0 == y) {
      return x;
    }

    return x - y * floor(x / y);
  }
#endif

  // wrap [rad] angle to [0...2PI)
  inline void WrapTwo2PI(float& angle) {
    angle = Mod(angle, static_cast<float>(2.0 * M_PI));
  }

  // wrap [rad] angle to [-PI...PI)
  inline void WrapTwoPI(float& angle) {
    angle = Mod(angle + static_cast<float>(M_PI), 
      static_cast<float>(2.0 * M_PI)) - static_cast<float>(M_PI);
  }

  FloatQuat tmp_quat_;
  void CalibrateGeometry::renormalizeCoeffs(float* coeff) {
    // Set all angles 0 --> 2pi
    for (uint32_t i = HAND_ORIENT_X; i < CAL_GEOM_NUM_COEFF; i++) {
      WrapTwoPI(coeff[i]);
    }
  }

  // coeff_min_limit_ is the minimum coefficient value before the penalty
  // function heavily penalizes configurations with this value
  const float CalibrateGeometry::coeff_min_limit_[CAL_GEOM_NUM_COEFF] = {
    -std::numeric_limits<float>::infinity(),    // HAND_POS_X
    -std::numeric_limits<float>::infinity(),    // HAND_POS_Y
    -std::numeric_limits<float>::infinity(),    // HAND_POS_Z
    -3.14159f,  // HAND_ORIENT_X
    -3.14159f,  // HAND_ORIENT_Y
    -3.14159f,  // HAND_ORIENT_Z
    //0.25f,       // SCALE
  };
  
  // coeff_max_limit_ is the maximum coefficient value before the penalty
  // function heavily penalizes configurations with this value
  const float CalibrateGeometry::coeff_max_limit_[CAL_GEOM_NUM_COEFF] = {
    std::numeric_limits<float>::infinity(),    // HAND_POS_X
    std::numeric_limits<float>::infinity(),    // HAND_POS_Y
    std::numeric_limits<float>::infinity(),    // HAND_POS_Z
    3.14159f,  // HAND_ORIENT_X
    3.14159f,  // HAND_ORIENT_Y
    3.14159f,  // HAND_ORIENT_Z
    //10.0f,     // SCALE
  };
  
  // coeff_penalty_scale_ is the exponential scale to use when penalizing coeffs
  // outside the min and max values.
  const float CalibrateGeometry::coeff_penalty_scale_[CAL_GEOM_NUM_COEFF] = {
    0,    // HAND_POS_X
    0,    // HAND_POS_Y
    0,    // HAND_POS_Z
    0,    // HAND_ORIENT_X
    0,    // HAND_ORIENT_Y
    0,    // HAND_ORIENT_Z
    //0,    // SCALE
  };

// angle_coeffs are boolean values indicating if the coefficient represents
  // a pure angle (0 --> 2pi)
  const bool CalibrateGeometry::angle_coeffs_[CAL_GEOM_NUM_COEFF] = {
    // Hand 1
    false,  // HAND_POS_X
    false,  // HAND_POS_Y
    false,  // HAND_POS_Z
    true,   // HAND_ORIENT_X
    true,   // HAND_ORIENT_Y
    true,   // HAND_ORIENT_Z
    //false,  // SCALE
  };

  void CalibrateGeometry::coeff2Mat(jtil::math::Float4x4& mat, 
    const float* coeff) {
    // Set the root matrix:
    euler2RotMatGM(mat, coeff[CALIB_ORIENT_X], coeff[CALIB_ORIENT_Y],
      coeff[CALIB_ORIENT_Z]);
    mat.leftMultTranslation(coeff[CALIB_POS_X], coeff[CALIB_POS_Y],
      coeff[CALIB_POS_Z]);
  }

  void CalibrateGeometry::coeffMeters2Mat(jtil::math::Float4x4& mat, 
    const float* coeff) {
    // Set the root matrix:
    euler2RotMatGM(mat, coeff[CALIB_ORIENT_X], coeff[CALIB_ORIENT_Y],
      coeff[CALIB_ORIENT_Z]);
    mat.leftMultTranslation(1000.0f * coeff[CALIB_POS_X], 
      1000.0f * coeff[CALIB_POS_Y], 1000.0f * coeff[CALIB_POS_Z]);
  }

  float CalibrateGeometry::calcAveCameraViewObjFunc(const float* coeff) {
    Float4x4 mat;
    coeffMeters2Mat(mat, coeff);
    float ret_val = 0.0f;
    Float3 v_trans;
    Float3 v_delta;
    for (uint32_t f = 0; f < num_frames_; f++) {
      for (uint32_t i = 0; i < 3; i++) {
        Float3::affineTransformPos(v_trans, mat, vq_[i][f]);
        Float3::sub(v_delta, v_trans, vb_[i][f]);
        ret_val += sqrtf(Float3::dot(v_delta, v_delta));
      }
    }
    return ret_val / ((float)num_frames_ * 3.0f);  // Normalize
  }

  void CalibrateGeometry::calcAveCameraViewJacobFunc(float* jacob, 
    const float* coeff) {
    float coeff_tmp[NUM_PARAMETERS];
    // Estimate using central diff.
    // http://math.fullerton.edu/mathews/n2003/differentiation/NumericalDiffProof.pdf
    memcpy(coeff_tmp, coeff, sizeof(coeff_tmp[0]) * NUM_PARAMETERS);
    const float h = 0.001f;
    for (uint32_t i = 0; i < NUM_PARAMETERS; i++) {
      coeff_tmp[i] = coeff[i] - h;
      float f0 = calcAveCameraViewObjFunc(coeff_tmp);
      coeff_tmp[i] = coeff[i] + h;
      float f1 = calcAveCameraViewObjFunc(coeff_tmp);
      coeff_tmp[i] = coeff[i];
      jacob[i] = (f1 - f0) / (2.0f * h);
    }
  }

  //void CalibrateGeometry::calcAveCameraView(jtil::math::Float4x4& ret, 
  //  const uint32_t i_base_cam, const uint32_t i_query_cam, 
  //  const float*** coeffs, const uint32_t num_frames) {
  //  std::cout << "calculating ave camera view" << std::endl;
  //  // Following Murphy's suggestion.  We're going to use BFGS and solve the
  //  // average transformation in a least sqs sense, that is we want to minimize
  //  // sum_f=1:n ( sum_i=1,2,3 (||A vb_i,f - vq_i,f||_2) )
  //  // - A is the affine transformation that goes from base coord to query
  //  // - vq_i,f is one of the 3 correspondance points in query's coord frame
  //  // - vb_i,f is the same correspondance point in base's coord frame
  //  solver_ = new BFGS(CalibrateCoeff::NUM_PARAMETERS);
  //  solver_->verbose = true;
  //  solver_->eta_s = 1e-8;  // Aggressive sufficient descent condition
  //  num_frames_ = num_frames;
  //  for (uint32_t i = 0; i < 3; i++) {
  //    vq_[i] = new Float3[num_frames_];
  //    vb_[i] = new Float3[num_frames_];
  //  }

  //  // Calculate vq and vb for each camera
  //  Float4x4 model_base;
  //  Float4x4 model_query;
  //  for (uint32_t f = 0; f < num_frames_; f++) {
  //    coeff2Mat(model_base, coeffs[i_base_cam][f]);
  //    coeff2Mat(model_query, coeffs[i_query_cam][f]);
  //    for (uint32_t i = 0; i < 3; i++) {
  //      Float3::affineTransformPos(vb_[i][f], model_base, vmodel_[i]);
  //      Float3::affineTransformPos(vq_[i][f], model_query, vmodel_[i]);
  //    }
  //  }
  //  
  //  // Approximate a starting matrix by using the 0th frame's data
  //  float c0_base[NUM_PARAMETERS];
  //  float c0_query[NUM_PARAMETERS];
  //  memcpy(c0_base, coeffs[i_base_cam][0], sizeof(c0_base[0])*NUM_PARAMETERS);
  //  memcpy(c0_query, coeffs[i_query_cam][0], sizeof(c0_base[0])*NUM_PARAMETERS);
  //  coeff2Mat(model_base, c0_base);
  //  coeff2Mat(model_query, c0_query);
  //  Float4x4 model_query_inv, mat0;
  //  Float4x4::inverse(model_query_inv, model_query);
  //  Float4x4::mult(mat0, model_base, model_query_inv);

  //  // MAT0 is just rotations and translations --> easy to decompose (and is
  //  // pretty close to the answer we want):
  //  float c0[NUM_PARAMETERS];
  //  Float4x4 rot0;
  //  Float3 trans0, euler0;
  //  Float4x4::getTranslation(trans0, mat0);
  //  // Float4x4::extractRotation(rot0, mat0);  // Actually does polar decomp
  //  Float4x4::rotMat2Euler(euler0[0], euler0[1], euler0[2], mat0);
  //  c0[CALIB_POS_X] = trans0[0] / 1000.0f;
  //  c0[CALIB_POS_Y] = trans0[1] / 1000.0f;
  //  c0[CALIB_POS_Z] = trans0[2] / 1000.0f;
  //  c0[CALIB_ORIENT_X] = euler0[0];
  //  c0[CALIB_ORIENT_Y] = euler0[1];
  //  c0[CALIB_ORIENT_Z] = euler0[2];

  //  std::cout << "Starting obj func value = " << calcAveCameraViewObjFunc(c0);
  //  std::cout << std::endl;

  //  float J0[NUM_PARAMETERS];
  //  calcAveCameraViewJacobFunc(J0, c0);
  //  std::cout << "Starting jacobian value = ";
  //  for (uint32_t i = 0; i < NUM_PARAMETERS; i++) {
  //    std::cout << J0[i] << " ";
  //  }
  //  std::cout << std::endl;

  //  float cfit[NUM_PARAMETERS];
  //  solver_->minimize(cfit, c0, angle_coeffs_, calcAveCameraViewObjFunc,
  //    calcAveCameraViewJacobFunc, renormalizeCoeffs);

  //  coeffMeters2Mat(ret, cfit);

  //  // Clean up
  //  SAFE_DELETE(solver_);
  //  for (uint32_t i = 0; i < 3; i++) {
  //    SAFE_DELETE_ARR(vq_[i]);
  //    SAFE_DELETE_ARR(vb_[i]);
  //  }
  //}

  void CalibrateGeometry::calcCameraView(const jtil::math::Float4x4& mat_base,
    jtil::math::Float4x4& mat_src, const uint32_t i_base_cam,
    const uint32_t i_query_cam, const jtil::data_str::VectorManaged<float*>* coeffs, 
    const uint32_t cur_frame) {
    std::cout << "calculating ave camera view" << std::endl;

    // Approximate a starting matrix by using the 0th frame's data
    float c0_base[NUM_PARAMETERS];
    float c0_query[NUM_PARAMETERS];
    memcpy(c0_base, coeffs[i_base_cam][cur_frame], 
      sizeof(c0_base[0])*NUM_PARAMETERS);
    memcpy(c0_query, coeffs[i_query_cam][cur_frame], 
      sizeof(c0_base[0])*NUM_PARAMETERS);
    Float4x4 model_base;
    Float4x4 model_query;
    coeff2Mat(model_base, c0_base);
    coeff2Mat(model_query, c0_query);
    Float4x4 model_query_inv;
    Float4x4::inverse(model_query_inv, model_query);
    Float4x4 model_query_to_base;
    Float4x4::mult(model_query_to_base, model_base, model_query_inv);
    Float4x4::mult(mat_src, mat_base, model_query_to_base);
  }

  void CalibrateGeometry::findPointsCloseToModel(Vector<float>& vert_ret, 
      Vector<float>& norm_ret, const float* xyz_src, 
      const float* norm_src, const float* coeff, const float dist_thresh) {
    float dist_thresh_sq = dist_thresh * dist_thresh;
    switch (type_) {
    case BOX:
      {
        Float4x4 model_mat, model_mat_inv;
        coeff2Mat(model_mat, coeff);
        Float4x4::inverse(model_mat_inv, model_mat);

        vert_ret.resize(0);
        norm_ret.resize(0);

        Float3 pt, pt_tranformed, pt_box, delta;
        Float3 box_half_lengths(BOX_SIDEA/2.0f, BOX_SIDEB/2.0f, BOX_SIDEC/2.0f);
        for (uint32_t i = 0; i < src_dim; i++) {
          // Transform the point into the box's coordinate frame
          pt.set(xyz_src[i * 3], xyz_src[i * 3 + 1], xyz_src[i * 3 + 2]);
          Float3::affineTransformPos(pt_tranformed, model_mat_inv, pt);
          // Force the tranformed point into the postive octant (flipping the
          // axis wont change the distance computation, but makes our life easier)
          pt_tranformed[0] = fabsf(pt_tranformed[0]);
          pt_tranformed[1] = fabsf(pt_tranformed[1]);
          pt_tranformed[2] = fabsf(pt_tranformed[2]);
          // Find the closest point on the box to the transformed point in the 
          // positive octant, we don't care about negative coeffs
          pt_box[0] = std::min<float>(box_half_lengths[0], pt_tranformed[0]);
          pt_box[1] = std::min<float>(box_half_lengths[1], pt_tranformed[1]);
          pt_box[2] = std::min<float>(box_half_lengths[2], pt_tranformed[2]);
          // Now calculate the squared distance:
          Float3::sub(delta, pt_box, pt_tranformed);
          float dist_sq = Float3::dot(delta, delta);
          if (dist_sq <= dist_thresh_sq) {
            vert_ret.pushBack(xyz_src[i * 3]);
            vert_ret.pushBack(xyz_src[i * 3 + 1]);
            vert_ret.pushBack(xyz_src[i * 3 + 2]);
            norm_ret.pushBack(norm_src[i * 3]);
            norm_ret.pushBack(norm_src[i * 3 + 1]);
            norm_ret.pushBack(norm_src[i * 3 + 2]);

          }
        }
      }
      break;
    case ICOSAHEDRON:
      {
        coeff2Mat(*scene_graph_->mat(), coeff);
        updateSize();
        updateHeirachyMatrices();
        Float4x4 model_mat_inv;
        Float4x4::inverse(model_mat_inv, *icosahedron_->mat_hierarchy());

        // Just be lazy and do it by radius
        // http://en.wikipedia.org/wiki/Icosahedron :
        float radius = ICOSAHEDRON_SIDE_LENGTH * sinf(2.0f * (float)M_PI / 5.0f);
        radius *= 0.9f;  // Shrink it in a bit to avoid noise.

        vert_ret.resize(0);
        norm_ret.resize(0);

        Float3 pt, pt_tranformed, pt_box, delta;
        for (uint32_t i = 0; i < src_dim; i++) {
          // Transform the point into the box's coordinate frame
          pt.set(xyz_src[i * 3], xyz_src[i * 3 + 1], xyz_src[i * 3 + 2]);
          Float3::affineTransformPos(pt_tranformed, model_mat_inv, pt);
          float dist_from_origion = sqrtf(Float3::dot(pt_tranformed, pt_tranformed));
          if ((dist_from_origion - dist_thresh) <= radius) {
            vert_ret.pushBack(xyz_src[i * 3]);
            vert_ret.pushBack(xyz_src[i * 3 + 1]);
            vert_ret.pushBack(xyz_src[i * 3 + 2]);
            norm_ret.pushBack(norm_src[i * 3]);
            norm_ret.pushBack(norm_src[i * 3 + 1]);
            norm_ret.pushBack(norm_src[i * 3 + 2]);
          }
        }
      }
      break;
    default:
      throw std::runtime_error("findPointsCloseToModel() - Not supported for"
        " this model type");
    }
  }

}  // namespace hand_fit
