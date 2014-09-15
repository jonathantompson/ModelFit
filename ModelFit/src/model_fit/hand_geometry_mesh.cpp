#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include "model_fit/model_renderer.h"
#include "model_fit/hand_geometry_mesh.h"
#include "model_fit/bounding_sphere.h"
#include "renderer/open_gl_common.h"
#include "renderer/renderer.h"
#include "renderer/colors.h"
#include "renderer/geometry/geometry_manager.h"
#include "renderer/geometry/geometry.h"
#include "renderer/geometry/geometry_textured_boned_mesh.h"
#include "renderer/geometry/geometry_vertices.h"
#include "renderer/geometry/geometry_colored_mesh.h"
#include "renderer/geometry/bone_info.h"
#include "data_str/pair.h"
#include "kinect_interface_primesense/hand_model.h"  // for HandCoeffConvnet

#define SAFE_DELETE(target) \
  if (target != NULL) { \
    delete target; \
    target = NULL; \
  }

#ifndef HAND_FIT
#error "HAND_FIT is not defined!  You need to declare it in the preprocessor"
#endif

using namespace jtil::math;
using std::string;
using std::runtime_error;
using std::cout;
using std::endl;
using namespace renderer;
using namespace kinect_interface_primesense::hand_net;

namespace model_fit {
  float HandGeometryMesh::pso_radius_c_[HAND_NUM_COEFF];
  float HandGeometryMesh::cur_scale_ = 1.0f;
  float HandGeometryMesh::cur_lengths_[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  
  HandGeometryMesh::HandGeometryMesh(HandType hand_type) {
    // Set all the bones to undefined:
    bone_wrist_index_ = MAX_UINT32;
    bone_palm_index_ = MAX_UINT32;
    bone_thumb_1_index_ = MAX_UINT32;
    bone_thumb_2_index_ = MAX_UINT32;
    bone_thumb_3_index_ = MAX_UINT32;
    for (uint32_t i = 0; i < 4; i++) {
      bone_finger_1_index_[i] = MAX_UINT32;
      bone_finger_2_index_[i] = MAX_UINT32;
      bone_finger_3_index_[i] = MAX_UINT32;
      bone_finger_4_index_[i] = MAX_UINT32;
    }

    loadHandGeometry(hand_type);

    // HACK: The model's normals are inside out --> Fix them
    if (hand_type == HandType::RIGHT) {
      if (mesh_->type() == GeometryType::GEOMETRY_TEXTURED_BONED_MESH) {
        GeometryTexturedBonedMesh* m = (GeometryTexturedBonedMesh*)mesh_;
        m->unsyncVAO();
        jtil::data_str::Vector<jtil::math::Float3>* norms = m->normals();
        for (uint32_t i = 0; i < norms->size(); i++) {
          Float3::scale((*norms)[i], -1.0f);
        }
        m->syncVAO();
      }
    }

    // Adding the geometry to the gobal geometry manager's scene graph will
    // transfer ownership of the memory.
    GeometryManager::scene_graph_root()->addChild(scene_graph_);
    renderer_attachment_ = true;

    const float* cmax = coeff_max_limit();
    const float* cmin = coeff_min_limit();
     
    // Set the PSO static radius
    for (uint32_t i = HAND_POS_X; i <= HAND_POS_Z; i++) {
      pso_radius_c_[i] = PSO_RAD_POSITION;
    }
    for (uint32_t i = HAND_ORIENT_X; i <= HAND_ORIENT_Z; i++) {
      pso_radius_c_[i] = PSO_RAD_EULER;
    }
    for (uint32_t i = WRIST_THETA; i <= WRIST_PHI; i++) {  // Wrist
      pso_radius_c_[i] = (cmax[i] - cmin[i]) * PSO_RAD_WRIST;
    }
    for (uint32_t i = THUMB_THETA; i <= THUMB_K2_PHI; i++) {  // thumb
      pso_radius_c_[i] = (cmax[i] - cmin[i]) * PSO_RAD_THUMB;
    }
    for (uint32_t i = 0; i < 4; i++) {  // All fingers
      pso_radius_c_[F0_ROOT_THETA+i*FINGER_NUM_COEFF] = 
        (cmax[F0_ROOT_THETA+i*FINGER_NUM_COEFF] - 
        cmin[F0_ROOT_THETA+i*FINGER_NUM_COEFF]) * PSO_RAD_FINGERS;
      pso_radius_c_[F0_ROOT_PHI+i*FINGER_NUM_COEFF] = 
        (cmax[F0_ROOT_PHI+i*FINGER_NUM_COEFF] - 
        cmin[F0_ROOT_PHI+i*FINGER_NUM_COEFF]) * PSO_RAD_FINGERS;
      pso_radius_c_[F0_THETA+i*FINGER_NUM_COEFF] = 
        (cmax[F0_THETA+i*FINGER_NUM_COEFF] - 
        cmin[F0_THETA+i*FINGER_NUM_COEFF]) * PSO_RAD_FINGERS;
      pso_radius_c_[F0_PHI+i*FINGER_NUM_COEFF] = 
        (cmax[F0_PHI+i*FINGER_NUM_COEFF] - 
        cmin[F0_PHI+i*FINGER_NUM_COEFF]) * PSO_RAD_FINGERS;
      pso_radius_c_[F0_KNUCKLE_MID+i*FINGER_NUM_COEFF] = 
        (cmax[F0_KNUCKLE_MID+i*FINGER_NUM_COEFF] - 
        cmin[F0_KNUCKLE_MID+i*FINGER_NUM_COEFF]) * PSO_RAD_FINGERS;
      pso_radius_c_[F0_KNUCKLE_END+i*FINGER_NUM_COEFF] = 
        (cmax[F0_KNUCKLE_END+i*FINGER_NUM_COEFF] - 
        cmin[F0_KNUCKLE_END+i*FINGER_NUM_COEFF]) * PSO_RAD_FINGERS;
    }
    for (uint32_t i = F0_TWIST; i <= THUMB_TWIST; i++) {  // thumb
      pso_radius_c_[i] = (cmax[i] - cmin[i]) * PSO_RAD_FINGERS;
    }
  }

  HandGeometryMesh::~HandGeometryMesh() {
    // Note, ownership of all geometry is transfered to the renderer class
  }

  uint32_t HandGeometryMesh::sph_bone_ind_[NUM_BOUNDING_SPHERES];

  void HandGeometryMesh::loadHandGeometry(HandType type) {
    Renderer* g_renderer = Renderer::g_renderer();
    GeometryManager* gm = GeometryManager::g_geom_manager();
#ifndef LOAD_HAND_MESH_JFILE
    if (type == HandType::LEFT) {
      scene_graph_ = gm->loadFromFile(HAND_MODEL_PATH, LHAND_MODEL_FILE);
      gm->saveToJFile(scene_graph_, HAND_MODEL_PATH, LHAND_MODEL_JFILE);
    } else {
      scene_graph_ = gm->loadFromFile(HAND_MODEL_PATH, RHAND_MODEL_FILE);
      gm->saveToJFile(scene_graph_, HAND_MODEL_PATH, RHAND_MODEL_JFILE);
    }
#else

    if (type == HandType::LEFT) {
      scene_graph_ = gm->loadFromJFile(HAND_MODEL_PATH, LHAND_MODEL_JFILE);
    } else {
      scene_graph_ = gm->loadFromJFile(HAND_MODEL_PATH, RHAND_MODEL_JFILE);
    }
#endif

    // Find all the bone nodes so we can quickly access them later:
    if (type == HandType::LEFT) {
      bones_in_file_ = gm->findBoneFileInfo(LHAND_MODEL_FILE);
    } else {
      bones_in_file_ = gm->findBoneFileInfo(RHAND_MODEL_FILE);
    }
    if (bones_in_file_ == NULL) {
      throw runtime_error(string("HandGeometryMesh::loadHandGeometry()")+
        string(" - ERROR: Could not find hand model bone data!"));
    }

    // 2. Now find the bone indices that we care about:
    for (uint32_t i = 0; i < bones_in_file_->bones.size(); i++) {
      Bone* cur_bone = bones_in_file_->bones[i];
      if (cur_bone->name == "carpals") {
        bone_wrist_index_ = i;
      } else if (cur_bone->name == "metacarpals") {
        bone_palm_index_ = i;
      } else if (cur_bone->name == "finger5joint1") {
        bone_thumb_1_index_ = i;
      } else if (cur_bone->name == "finger5joint2") {
        bone_thumb_2_index_ = i;
      } else if (cur_bone->name == "finger5joint3") {
        bone_thumb_3_index_ = i;
      } else if (cur_bone->name == "Bone") {
        bone_finger_1_index_[0] = i;
      } else if (cur_bone->name == "finger1joint1") {
        bone_finger_2_index_[0] = i;
      } else if (cur_bone->name == "finger1joint2") {
        bone_finger_3_index_[0] = i;
      } else if (cur_bone->name == "finger1joint3") {
        bone_finger_4_index_[0] = i;
      } else if (cur_bone->name == "Bone.001") {
        bone_finger_1_index_[1] = i;
      } else if (cur_bone->name == "finger2joint1") {
        bone_finger_2_index_[1] = i;
      } else if (cur_bone->name == "finger2joint2") {
        bone_finger_3_index_[1] = i;
      } else if (cur_bone->name == "finger2joint3") {
        bone_finger_4_index_[1] = i;
      } else if (cur_bone->name == "Bone.002") {
        bone_finger_1_index_[2] = i;
      } else if (cur_bone->name == "finger3joint1") {
        bone_finger_2_index_[2] = i;
      } else if (cur_bone->name == "finger3joint2") {
        bone_finger_3_index_[2] = i;
      } else if (cur_bone->name == "finger3joint3") {
        bone_finger_4_index_[2] = i;
      } else if (cur_bone->name == "Bone.003") {
        bone_finger_1_index_[3] = i;
      } else if (cur_bone->name == "finger4joint1") {
        bone_finger_2_index_[3] = i;
      } else if (cur_bone->name == "finger4joint2") {
        bone_finger_3_index_[3] = i;
      } else if (cur_bone->name == "finger4joint3") {
        bone_finger_4_index_[3] = i;
      } else {
        cout << "WARNING: couldn't associate bone: " << cur_bone->name;
      }
    }

    // Check that we found all the bones we were looking for.
    bool bones_ok = true;
    for (uint32_t i = 0; i < 4; i++) {
      bones_ok = bones_ok && (bone_finger_1_index_[i] != MAX_UINT32);
      bones_ok = bones_ok && (bone_finger_2_index_[i] != MAX_UINT32);
      bones_ok = bones_ok && (bone_finger_3_index_[i] != MAX_UINT32);
      bones_ok = bones_ok && (bone_finger_4_index_[i] != MAX_UINT32);
    }
    if (bone_wrist_index_ == MAX_UINT32 || bone_palm_index_ == MAX_UINT32 ||
       bone_thumb_1_index_ == MAX_UINT32 || bone_thumb_2_index_ == MAX_UINT32 ||
       bone_thumb_3_index_ == MAX_UINT32 || bone_palm_index_ == MAX_UINT32 ||
       !bones_ok) {
      throw runtime_error(string("HandGeometryMesh::loadHandGeometry()") +
        string(" - ERROR: couldn't find one of the bones!"));
    }

    // Record the association between bounding spheres and bone indices
    for (uint32_t i = 0; i < 4; i++) {
      sph_bone_ind_[F1_KNU3_A + i * NSPH_PER_GROUP] = bone_finger_4_index_[i];
      sph_bone_ind_[F1_KNU3_B + i * NSPH_PER_GROUP] = bone_finger_4_index_[i];
      sph_bone_ind_[F1_KNU2_A + i * NSPH_PER_GROUP] = bone_finger_3_index_[i];
      sph_bone_ind_[F1_KNU2_B + i * NSPH_PER_GROUP] = bone_finger_3_index_[i];
      sph_bone_ind_[F1_KNU1_A + i * NSPH_PER_GROUP] = bone_finger_2_index_[i];
      sph_bone_ind_[F1_KNU1_B + i * NSPH_PER_GROUP] = bone_finger_2_index_[i];
    }
    sph_bone_ind_[TH_KNU3_A] = bone_thumb_3_index_;
    sph_bone_ind_[TH_KNU3_B] = bone_thumb_3_index_;
    sph_bone_ind_[TH_KNU2_A] = bone_thumb_2_index_;
    sph_bone_ind_[TH_KNU2_B] = bone_thumb_2_index_;
    sph_bone_ind_[TH_KNU1_A] = bone_thumb_1_index_;
    sph_bone_ind_[TH_KNU1_B] = bone_thumb_1_index_;

    sph_bone_ind_[PALM_1] = bone_palm_index_;
    sph_bone_ind_[PALM_2] = bone_palm_index_;
    sph_bone_ind_[PALM_3] = bone_palm_index_; 
    sph_bone_ind_[PALM_4] = bone_palm_index_;
    sph_bone_ind_[PALM_5] = bone_palm_index_;
    sph_bone_ind_[PALM_6] = bone_palm_index_;

    // For all meshes in the heirachy set the specular intensity low:
    renderStackReset(); 
    while (!renderStackEmpty()) {
      Geometry* geom = renderStackPop();
      geom->mtrl()->specular_intensity = 0.05f;
    }

    // Now save the rest transforms for each bone:
    rest_transforms_.capacity(bones_in_file_->bones.size());
    rest_transforms_.resize(bones_in_file_->bones.size());
    for (uint32_t i = 0; i < bones_in_file_->bones.size(); i++) {
      rest_transforms_[i].set(*bones_in_file_->bones[i]->getNode()->mat());
    }

    string mesh_name = string("hand_mesh");
    mesh_ = gm->findGeometryByName(scene_graph_, mesh_name);
    if (mesh_ == NULL) {
      throw std::runtime_error("ERROR: Couldn't find hand_mesh geometry!");
    }

    // Now place static bounding spheres to bone transforms:
    Float4x4 bone_offset_mat;
    Float3 center;
    Float3 origin(0, 0, 0);
    BoundingSphere* cur_sphere;
    Float4x4 temp;
    for (uint32_t i = 0; i < NUM_BOUNDING_SPHERES; i++) {
      Bone* cur_bone = bones_in_file_->bones[sph_bone_ind_[i]];
      Geometry* geom = cur_bone->getNode();
      bone_offset_mat.set(cur_bone->bone_offset); 
      Float4x4::inverse(temp, bone_offset_mat);
      Float3::affineTransformPos(center, temp, origin);
      center.accum(&HandModelCoeff::sph_off_[i*3]);
      cur_sphere = new BoundingSphere(HandModelCoeff::sph_size_[i], center, mesh_, 
        scene_graph_, cur_bone->bone_offset);
      geom->addChild(cur_sphere);
      PoseModel::g_b_spheres.pushBack(cur_sphere);
      bspheres_.pushBack(cur_sphere);
    }
  }

  void HandGeometryMesh::setCurrentStaticHandProperties(
    const float* coeff) {
    cur_scale_ = coeff[SCALE];
    cur_lengths_[0] = coeff[F0_LENGTH];
    cur_lengths_[1] = coeff[F1_LENGTH];
    cur_lengths_[2] = coeff[F2_LENGTH];
    cur_lengths_[3] = coeff[F3_LENGTH];
    cur_lengths_[4] = coeff[THUMB_LENGTH];
  }

  void HandGeometryMesh::updateMatrices(const float* coeff) {
    FloatQuat cur_rot_quat;
    Float4x4* mat;

    // Set the root matrix:
    mat = scene_graph_->mat();
    euler2RotMatGM(*mat, coeff[HAND_ORIENT_X], coeff[HAND_ORIENT_Y],
      coeff[HAND_ORIENT_Z]);
    mat->leftMultTranslation(coeff[HAND_POS_X],
                             coeff[HAND_POS_Y],
                             coeff[HAND_POS_Z]);
    mat->rightMultScale(cur_scale_, cur_scale_, cur_scale_); 

    // Set the palm bone (depending on wrist angle)
    mat = bones_in_file_->bones[bone_wrist_index_]->getNode()->mat();
    rotateMatXAxisGM(mat_tmp1, coeff[WRIST_PHI]);
    rotateMatZAxisGM(mat_tmp2, coeff[WRIST_THETA]);
    Float4x4::multSIMD(mat_tmp3, mat_tmp1, mat_tmp2);
    // rotateMatXAxis(mat_tmp1, HandModelCoeff::wrist_twist);
    // Float4x4::mult(mat_tmp2, mat_tmp1, mat_tmp3);
    Float4x4::multSIMD(*mat, rest_transforms_[bone_wrist_index_], mat_tmp3);

    // Set the finger bones
//#pragma omp parallel for num_threads(4)
    for (int32_t i = 0; i < 4; i++) {
      float theta, phi, psi;
      Float4x4* mat;
      jtil::math::Float4x4 mat_tmp1;
      jtil::math::Float4x4 mat_tmp2;

      // Root
      theta = coeff[F0_ROOT_THETA + i * FINGER_NUM_COEFF];
      phi = coeff[F0_ROOT_PHI + i * FINGER_NUM_COEFF];
      psi = 0;
      mat = bones_in_file_->bones[bone_finger_1_index_[i]]->getNode()->mat();
      euler2RotMatGM(mat_tmp1, psi, theta, phi);
      Float4x4::multSIMD(*mat, rest_transforms_[bone_finger_1_index_[i]], mat_tmp1);

      // K1 base
      theta = coeff[F0_THETA + i * FINGER_NUM_COEFF];
      phi = coeff[F0_PHI + i * FINGER_NUM_COEFF];
      psi = coeff[F0_TWIST + i];
      mat = bones_in_file_->bones[bone_finger_2_index_[i]]->getNode()->mat();
      euler2RotMatGM(mat_tmp1, psi, theta, phi);
      Float4x4::multSIMD(*mat, rest_transforms_[bone_finger_2_index_[i]], mat_tmp1);
      mat->rightMultScale(1.0f, 1.0f + cur_lengths_[i], 1.0f);  // Scale this node

      mat = bones_in_file_->bones[bone_finger_3_index_[i]]->getNode()->mat();
      float k2_theta = coeff[F0_KNUCKLE_MID + i * FINGER_NUM_COEFF];
      rotateMatXAxisGM(mat_tmp1, k2_theta);
      const Float4x4& bone_mid = rest_transforms_[bone_finger_3_index_[i]];
      Float3 bone_mid_pos;
      Float4x4::getTranslation(bone_mid_pos, bone_mid);
      float bone_base_length = bone_mid_pos.length();
      mat_tmp2.set(bone_mid);
      // Move bone by fraction of the bone length:
      mat_tmp2.leftMultTranslation(0, bone_base_length * cur_lengths_[i], 0);  
      Float4x4::multSIMD(*mat, mat_tmp2, mat_tmp1);
      mat->leftMultScale(1.0f, 1.0f / (1.0f + cur_lengths_[i]), 1.0f);  // Undo parent scale
      mat->rightMultScale(1.0f, 1.0f + cur_lengths_[i], 1.0f);  // Scale this node

      mat = bones_in_file_->bones[bone_finger_4_index_[i]]->getNode()->mat();
      float k3_theta = coeff[F0_KNUCKLE_END + i * FINGER_NUM_COEFF];
      rotateMatXAxisGM(mat_tmp1, k3_theta);
      const Float4x4& bone_tip = rest_transforms_[bone_finger_4_index_[i]];
      Float3 bone_tip_pos;
      Float4x4::getTranslation(bone_tip_pos, bone_tip);
      float bone_mid_length = bone_tip_pos.length();
      mat_tmp2.set(bone_tip);
      // Move bone by fraction of the bone length:
      mat_tmp2.leftMultTranslation(0, bone_mid_length * cur_lengths_[i], 0);
      Float4x4::multSIMD(*mat, mat_tmp2, mat_tmp1);

      mat->leftMultScale(1.0f, 1.0f / (1.0f + cur_lengths_[i]), 1.0f);  // Undo parent scale
      mat->rightMultScale(1.0f, 1.0f + cur_lengths_[i], 1.0f);  // Scale this node
    }

    // Set the thumb bones
    float theta = coeff[THUMB_THETA];
    float phi = coeff[THUMB_PHI];
    float psi = coeff[THUMB_TWIST];
    mat = bones_in_file_->bones[bone_thumb_1_index_]->getNode()->mat();
    euler2RotMatGM(mat_tmp1, psi, theta, phi);
    Float4x4::multSIMD(*mat, rest_transforms_[bone_thumb_1_index_], mat_tmp1);

    theta = coeff[THUMB_K1_THETA];
    phi = coeff[THUMB_K1_PHI];
    mat = bones_in_file_->bones[bone_thumb_2_index_]->getNode()->mat();
    rotateMatZAxisGM(mat_tmp1, theta);
    rotateMatXAxisGM(mat_tmp2, phi);
    Float4x4::multSIMD(mat_tmp3, mat_tmp1, mat_tmp2);

    const Float4x4& bone_mid = rest_transforms_[bone_thumb_2_index_];
    mat_tmp2.set(bone_mid);
    Float4x4::multSIMD(*mat, mat_tmp2, mat_tmp3);
    mat->rightMultScale(1.0f, 1.0f + cur_lengths_[4], 1.0f);  // Scale this node

    phi = coeff[THUMB_K2_PHI];
    mat = bones_in_file_->bones[bone_thumb_3_index_]->getNode()->mat();
    rotateMatXAxisGM(mat_tmp1, phi);
    const Float4x4& bone_tip = rest_transforms_[bone_thumb_3_index_];
    Float3 bone_tip_pos;
    Float4x4::getTranslation(bone_tip_pos, bone_tip);
    float bone_mid_length = bone_tip_pos.length();
    mat_tmp2.set(bone_tip);
    // Move bone by fraction of the bone length:
    mat_tmp2.leftMultTranslation(0, bone_mid_length * cur_lengths_[4], 0);
    Float4x4::multSIMD(*mat, mat_tmp2, mat_tmp1);
    mat->leftMultScale(1.0f, 1.0f / (1.0f + cur_lengths_[4]), 1.0f);  // Undo parent scale
    mat->rightMultScale(1.0f, 1.0f + cur_lengths_[4], 1.0f);  // Scale this node

  }
  
  void HandGeometryMesh::renderStackReset() {
    render_stack_.resize(0);  // empty the stack (without deallocating)
    // Seed the render stack with the root node
    render_stack_.pushBack(scene_graph_);
  }

  // TO DO: CREATE A FLATTENED HEIRACHY VECTOR (LIKE IN HAND_MODEL.CPP) TO
  // AVOID THE RENDER OVERHEAD
  Geometry* HandGeometryMesh::renderStackPop() {
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

  bool HandGeometryMesh::renderStackEmpty() {
    return render_stack_.size() == 0;
  }

  void HandGeometryMesh::updateHeirachyMatrices() {
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
    // Now update bone matrices
    GeometryManager::g_geom_manager()->updateBoneMatrices(bones_in_file_);
  }

  void HandGeometryMesh::fixBoundingSphereMatrices() {
    
  }

  Geometry* root = NULL;
  void HandGeometryMesh::handCoeff2CoeffConvnet(HandModelCoeff* hand,
    float* coeff_convnet, const Int4& hand_pos_wh, const Float3& uvd_com,
    const Float4x4& proj_mat, const Float4x4& view_mat) {
    // Thumb and fingers are actually learned as salient points -->
    // Luckily we have a good way to get these.  Use the positions of some of
    // the key bounding sphere positions --> Then project these into UV.
    root = NULL;
    setRendererAttachement(false);
    setCurrentStaticHandProperties(hand->coeff());
    updateMatrices(hand->coeff());
    updateHeirachyMatrices();
    fixBoundingSphereMatrices();
 
    // Project the XYZ position into UV space
    // Use the bounding sphere centers since they are already in good positions
    for (uint32_t i = 0; i < num_convnet_feats; i++) {
      extractPositionForConvnet(hand, coeff_convnet, hand_pos_wh, uvd_com,
        convnet_sphere_indices[i], i * FEATURE_SIZE, proj_mat, view_mat);
    }
  }

  void HandGeometryMesh::handCoeff2UVD(HandModelCoeff* hand,
    float* coeff_uvd, const Int4& hand_pos_wh, const Float3& uvd_com,
    const Float4x4& proj_mat, const Float4x4& view_mat) {
    // Thumb and fingers are actually learned as salient points -->
    // Luckily we have a good way to get these.  Use the positions of some of
    // the key bounding sphere positions --> Then project these into UV.
    root = NULL;
    setRendererAttachement(false);
    setCurrentStaticHandProperties(hand->coeff());
    updateMatrices(hand->coeff());
    updateHeirachyMatrices();
    fixBoundingSphereMatrices();
 
    // Project the XYZ position into UV space
    // Use the bounding sphere centers since they are already in good positions
    for (uint32_t i = 0; i < num_uvd_feats; i++) {
      extractPositionForUVD(hand, coeff_uvd, hand_pos_wh, uvd_com,
        i, i * FEATURE_SIZE, proj_mat, view_mat);
    }
  }

  Float4x4 root_inverse, tmp;
  void HandGeometryMesh::extractPositionForConvnet(HandModelCoeff* hand, 
    float* coeff_convnet, const Int4& hand_pos_wh, const Float3& uvd_com,
    const uint32_t b_sphere_index, const uint32_t convnet_U_index,
    const Float4x4& proj_mat, const Float4x4& view_mat) {
    const float* coeff = hand->coeff();
    float dmin = uvd_com[2] - (HN_HAND_SIZE * 0.5f);
    BoundingSphere* sphere = PoseModel::g_b_spheres[b_sphere_index];
    //BoundingSphere* sphere = bspheres_[b_sphere_index];

    // This is a bit of a hack, but we have to undo the parent's root
    // transform (by left multiplying by its inverse).
    // Then we have to left multiply by the mesh node's transform
    Geometry* cur_root = sphere->hand_root();
    if (cur_root != root) {
      Float4x4::inverse(root_inverse, *sphere->hand_root()->mat());
      root = cur_root;
    }
    Float4x4::mult(tmp, root_inverse, *sphere->mat_hierarchy());
    Float4x4::mult(*sphere->mat_hierarchy(), *sphere->mesh_node()->mat_hierarchy(), tmp);

    // NOTE, WE COULD PROBABLY JUST MULTIPLY THE ALREADY CALCULATED BONE 
    // TRANSFORM WITH THE MESH_NODE'S TRANSFORM TO AVOID THE INVERSE TWICE (AND
    // SAVE ON A FEW CALCULATIONS):
    // sphere->mat_hierarchy() = sphere->mesh_node()->mat_hierarchy() * BONE_TRANSFORM
    // But I don't want to touch it in case I break it.
    
    sphere->transform();

    Float2 pos_uv;
    calcHandImageUVFromXYZ(*sphere->transformed_center(), pos_uv, hand_pos_wh,
      proj_mat, view_mat);
    coeff_convnet[convnet_U_index] = pos_uv[0];
    coeff_convnet[convnet_U_index+1] = pos_uv[1];
    if (FEATURE_SIZE >= 3) {
      coeff_convnet[convnet_U_index+2] = ((*sphere->transformed_center())[2] - 
        dmin) / HN_HAND_SIZE;
    }
  }

  void HandGeometryMesh::extractPositionForUVD(HandModelCoeff* hand, 
    float* coeff_convnet, const Int4& hand_pos_wh, const Float3& uvd_com,
    const uint32_t b_sphere_index, const uint32_t convnet_U_index,
    const Float4x4& proj_mat, const Float4x4& view_mat) {
    const float* coeff = hand->coeff();
    float dmin = uvd_com[2] - (HN_HAND_SIZE * 0.5f);
    BoundingSphere* sphere = PoseModel::g_b_spheres[b_sphere_index];
    //BoundingSphere* sphere = bspheres_[b_sphere_index];

    // This is a bit of a hack, but we have to undo the parent's root
    // transform (by left multiplying by its inverse).
    // Then we have to left multiply by the mesh node's transform
    Geometry* cur_root = sphere->hand_root();
    if (cur_root != root) {
      Float4x4::inverse(root_inverse, *sphere->hand_root()->mat());
      root = cur_root;
    }
    Float4x4::mult(tmp, root_inverse, *sphere->mat_hierarchy());
    Float4x4::mult(*sphere->mat_hierarchy(), *sphere->mesh_node()->mat_hierarchy(), tmp);

    // NOTE, WE COULD PROBABLY JUST MULTIPLY THE ALREADY CALCULATED BONE 
    // TRANSFORM WITH THE MESH_NODE'S TRANSFORM TO AVOID THE INVERSE TWICE (AND
    // SAVE ON A FEW CALCULATIONS):
    // sphere->mat_hierarchy() = sphere->mesh_node()->mat_hierarchy() * BONE_TRANSFORM
    // But I don't want to touch it in case I break it.
    
    sphere->transform();

    Float2 pos_uv;
    calcHandImageUVFromXYZ(*sphere->transformed_center(), pos_uv, hand_pos_wh,
      proj_mat, view_mat);
    coeff_convnet[convnet_U_index] = pos_uv[0]  * 
      (float)hand_pos_wh[2] + (float)hand_pos_wh[0];
    coeff_convnet[convnet_U_index+1] = pos_uv[1]  * 
      (float)hand_pos_wh[3] + (float)hand_pos_wh[1];
    if (FEATURE_SIZE >= 3) {
      coeff_convnet[convnet_U_index+2] = (*sphere->transformed_center())[2];
    }
  }

  void HandGeometryMesh::calcHandImageUVFromXYZ(Float3& xyz_pos, 
    Float2& uv_pos, const Int4& hand_pos_wh, const Float4x4& proj_mat, 
    const Float4x4& view_mat) {
    Float4 pos(xyz_pos[0], xyz_pos[1], xyz_pos[2], 1.0f);
    Float4 eye_pos;
    Float4::mult(eye_pos, view_mat, pos);
    Float4 homog_pos;
    Float4::mult(homog_pos, proj_mat, eye_pos);
    uv_pos[0] = (homog_pos[0] / homog_pos[3]);  // NDC X: -1 --> 1
    uv_pos[1] = (homog_pos[1] / homog_pos[3]);  // NDC Y: -1 --> 1
    // http://www.songho.ca/opengl/gl_transform.html
    uv_pos[0] = (float)src_width * 0.5f * (uv_pos[0] + 1);  // Window X: 0 --> W
    uv_pos[1] = (float)src_height * 0.5f * (-uv_pos[1] + 1);  // Window Y: 0 --> H
    // Now figure out the fractional position in the hand sub-image 
    uv_pos[0] = (uv_pos[0] - (float)hand_pos_wh[0]) / (float)hand_pos_wh[2];
    uv_pos[1] = (uv_pos[1] - (float)hand_pos_wh[1]) / (float)hand_pos_wh[3];
  }

  void HandGeometryMesh::setRendererAttachement(const bool renderer_attachment) {
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

  const bool HandGeometryMesh::getRendererAttachement() {
    return renderer_attachment_;
  }

  // These next few methods are to avoid the cos and sin double functions in 
  // the Mat4x4 template
  void HandGeometryMesh::euler2RotMatGM(Float4x4& a, const float x_angle, 
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

  void HandGeometryMesh::rotateMatZAxisGM(Float4x4& ret, const float angle) {
    float cos_angle = cosf(angle);
    float sin_angle = sinf(angle);
#ifdef ROW_MAJOR
    ret.m[0] = cos_angle;
    ret.m[1] = -sin_angle;
    ret.m[2] = 0;
    ret.m[3] = 0;
    ret.m[4] = sin_angle;
    ret.m[5] = cos_angle;
    ret.m[6] = 0;
    ret.m[7] = 0;
    ret.m[8] = 0;
    ret.m[9] = 0;
    ret.m[10] = 1;
    ret.m[11] = 0;
    ret.m[12] = 0;
    ret.m[13] = 0;
    ret.m[14] = 0;
    ret.m[15] = 1;
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = cos_angle;
    ret.m[1] = sin_angle;
    ret.m[2] = 0;
    ret.m[3] = 0;
    ret.m[4] = -sin_angle;
    ret.m[5] = cos_angle;
    ret.m[6] = 0;
    ret.m[7] = 0;
    ret.m[8] = 0;
    ret.m[9] = 0;
    ret.m[10] = 1;
    ret.m[11] = 0;
    ret.m[12] = 0;
    ret.m[13] = 0;
    ret.m[14] = 0;
    ret.m[15] = 1;
#endif
  };

  void HandGeometryMesh::rotateMatYAxisGM(Float4x4& ret, const float angle) {
    float cos_angle = cosf(angle);
    float sin_angle = sinf(angle);
#ifdef ROW_MAJOR
    ret.m[0] = cos_angle;
    ret.m[1] = 0;
    ret.m[2] = sin_angle;
    ret.m[3] = 0;
    ret.m[4] = 0;
    ret.m[5] = 1;
    ret.m[6] = 0;
    ret.m[7] = 0;
    ret.m[8] = -sin_angle;
    ret.m[9] = 0;
    ret.m[10] = cos_angle;
    ret.m[11] = 0;
    ret.m[12] = 0;
    ret.m[13] = 0;
    ret.m[14] = 0;
    ret.m[15] = 1;
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = cos_angle;
    ret.m[1] = 0;
    ret.m[2] = -sin_angle;
    ret.m[3] = 0;
    ret.m[4] = 0;
    ret.m[5] = 1;
    ret.m[6] = 0;
    ret.m[7] = 0;
    ret.m[8] = sin_angle;
    ret.m[9] = 0;
    ret.m[10] = cos_angle;
    ret.m[11] = 0;
    ret.m[12] = 0;
    ret.m[13] = 0;
    ret.m[14] = 0;
    ret.m[15] = 1;
#endif
  };

  void HandGeometryMesh::rotateMatXAxisGM(Float4x4& ret, const float angle) {
    float cos_angle = cosf(angle);
    float sin_angle = sinf(angle);
#ifdef ROW_MAJOR
    ret.m[0] = 1;
    ret.m[1] = 0;
    ret.m[2] = 0;
    ret.m[3] = 0;
    ret.m[4] = 0;
    ret.m[5] = cos_angle;
    ret.m[6] = -sin_angle;
    ret.m[7] = 0;
    ret.m[8] = 0;
    ret.m[9] = sin_angle;
    ret.m[10] = cos_angle;
    ret.m[11] = 0;
    ret.m[12] = 0;
    ret.m[13] = 0;
    ret.m[14] = 0;
    ret.m[15] = 1;
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = 1;
    ret.m[1] = 0;
    ret.m[2] = 0;
    ret.m[3] = 0;
    ret.m[4] = 0;
    ret.m[5] = cos_angle;
    ret.m[6] = sin_angle;
    ret.m[7] = 0;
    ret.m[8] = 0;
    ret.m[9] = -sin_angle;
    ret.m[10] = cos_angle;
    ret.m[11] = 0;
    ret.m[12] = 0;
    ret.m[13] = 0;
    ret.m[14] = 0;
    ret.m[15] = 1;
#endif
  };

}  // namespace hand_fit
