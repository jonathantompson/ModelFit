#include <sstream>
#include <string>
#include "image_util/image_util.h"
#include "math/math_types.h"  // for uint
#include "data_str/vector.h"

using namespace jtil::data_str;
using namespace jtil::math;

namespace jtil {
namespace image_util {

  int32_t cur_pixel[MEDIAN_FILTER_MAX_LABELS];

  void createIndices(Vector<Int3>& indices, const float* xyz, const uint32_t w,
    const uint32_t h, const float dist_cutoff) {
    indices.resize(0);  // Set the size to zero without deallocation
    const float dist_cutoff_sq = dist_cutoff * dist_cutoff;

    // Populate the mesh data with faces
    // Step through the UV map examining each quad of 4 neighbouring vertices.
    // If 3 or more of those vertices are close enough, then add the
    // corresponding face.

    for (uint32_t v = 0; v < (h-1); v++) {
      for (uint32_t u = 0; u < (w-1); u++) {
        uint32_t p0 = v*w + u;          // top left
        uint32_t p1 = v*w + (u+1);      // top right
        uint32_t p2 = (v+1)*w + u;      // bottom left
        uint32_t p3 = (v+1)*w + (u+1);  // bottom right

        Float3 pt0(&xyz[p0 * 3]);
        Float3 pt1(&xyz[p1 * 3]);
        Float3 pt2(&xyz[p2 * 3]);
        Float3 pt3(&xyz[p3 * 3]);

        uint32_t face_cnt = 0;

        // Check if p0-p2-p1 make a face
        Float3 delta;
        Float3::sub(delta, pt0, pt2);
        float dist_sq_p0_p2 = Float3::dot(delta, delta);
        Float3::sub(delta, pt0, pt1);
        float dist_sq_p0_p1 = Float3::dot(delta, delta);
        if (dist_sq_p0_p2 <= dist_cutoff_sq && dist_sq_p0_p1 <= dist_cutoff_sq) {
          face_cnt++;
          indices.pushBack(Int3(p0, p2, p1)); 
        }

        // Check if p3-p1-p2 make a face
        Float3::sub(delta, pt3, pt1);
        float dist_sq_p3_p1 = Float3::dot(delta, delta);
        Float3::sub(delta, pt3, pt2);
        float dist_sq_p3_p2 = Float3::dot(delta, delta);
        if (dist_sq_p3_p1 <= dist_cutoff_sq && dist_sq_p3_p2 <= dist_cutoff_sq) {
          face_cnt++;
          indices.pushBack(Int3(p3, p1, p2)); 
        } 


        if (face_cnt == 0) {
          // There might be a face on the other diagonal
          // Check if p0-p3-p1 make a face
          Float3::sub(delta, pt0, pt3);
          float dist_sq_p0_p3 = Float3::dot(delta, delta);
          // dist_sq_p0_p1 already calculated
          if (dist_sq_p0_p3 <= dist_cutoff_sq && dist_sq_p0_p1 <= dist_cutoff_sq) {
            face_cnt++;
            indices.pushBack(Int3(p0, p3, p1)); 
          }

          // Check if p0-p2-p3 make a face
          // dist_sq_p0_p2 already calculated
          // dist_sq_p0_p3 already calculated
          if (dist_sq_p0_p2 <= dist_cutoff_sq && dist_sq_p0_p3 <= dist_cutoff_sq) {
            face_cnt++;
            indices.pushBack(Int3(p0, p2, p3)); 
          } 
        }
      }
    }
  }

  void calcNormal(Float3& normal, const float* pt0, const float* pt1, 
    const float* pt2) {
    Float3 tmp1_, tmp2_;
    tmp1_[0] = pt0[0] - pt1[0];
    tmp1_[1] = pt0[1] - pt1[1];
    tmp1_[2] = pt0[2] - pt1[2];

    tmp2_[0] = pt2[0] - pt1[0];
    tmp2_[1] = pt2[1] - pt1[1];
    tmp2_[2] = pt2[2] - pt1[2];
    Float3::cross(normal, tmp1_, tmp2_);
    normal.normalize();
  }

  void calcNormalUnNormalized(Float3& normal, const float* pt0, 
    const float* pt1, const float* pt2) {
    Float3 tmp1_, tmp2_;
    tmp1_[0] = pt0[0] - pt1[0];
    tmp1_[1] = pt0[1] - pt1[1];
    tmp1_[2] = pt0[2] - pt1[2];

    tmp2_[0] = pt2[0] - pt1[0];
    tmp2_[1] = pt2[1] - pt1[1];
    tmp2_[2] = pt2[2] - pt1[2];
    Float3::cross(normal, tmp1_, tmp2_);
  }

  float calcAngleSafe(const float* pt0, const float* pt1, const float* pt2)  {
    Float3 tmp1_, tmp2_;
    tmp1_[0] = pt0[0] - pt1[0];
    tmp1_[1] = pt0[1] - pt1[1];
    tmp1_[2] = pt0[2] - pt1[2];

    tmp2_[0] = pt2[0] - pt1[0];
    tmp2_[1] = pt2[1] - pt1[1];
    tmp2_[2] = pt2[2] - pt1[2];
    tmp1_.normalize();
    tmp2_.normalize();
    float dot = Float3::dot(tmp1_, tmp2_);
    dot = dot > 1 ? 1 : dot;
    dot = dot < -1 ? -1 : dot;
    return acosf(dot);
  }

  void createNormals(float* norm_xyz, const float* xyz, Vector<Int3>& indices, 
    const uint32_t w, const uint32_t h, 
    const NormalApproximationMethod method) {
    // Set all the normals as 0
    for (uint32_t i = 0; i < w * h; i++) {
      norm_xyz[i * 3] = 0;
      norm_xyz[i * 3 + 1] = 0;
      norm_xyz[i * 3 + 2] = 0;
    }
    
    // For each face in the index array, calculate its normal and add it to the
    // normal accumulation
    Float3 cur_normal;
    for (uint32_t i = 0; i < indices.size(); i++) {
      uint32_t v1 = indices[i][0];
      uint32_t v2 = indices[i][1];
      uint32_t v3 = indices[i][2];
      
      switch (method) {
      case DumbNormalApproximation:
        // METHOD 1 --> STUPID AND NOT FAST
        // If you sum the normal weighted by the tri area, then you get a better
        // average normal.  More importantly, if you calculate the normal by 
        // cross product then you get a normal whos lenght is 2 x the triangle 
        // area, so summing these gives us the correct weights.
        calcNormal(cur_normal, &xyz[v1 * 3], &xyz[v2 * 3], &xyz[v3 * 3]);

        norm_xyz[v1 * 3] += cur_normal[0];
        norm_xyz[v1 * 3 + 1] += cur_normal[1];
        norm_xyz[v1 * 3 + 2] += cur_normal[2];

        norm_xyz[v2 * 3] += cur_normal[0];
        norm_xyz[v2 * 3 + 1] += cur_normal[1];
        norm_xyz[v2 * 3 + 2] += cur_normal[2];

        norm_xyz[v3 * 3] += cur_normal[0];
        norm_xyz[v3 * 3 + 1] += cur_normal[1];
        norm_xyz[v3 * 3 + 2] += cur_normal[2];
        break;
      case SimpleNormalApproximation:
        // METHOD 2 --> FAST
        // If you sum the normal weighted by the tri area, then you get a better
        // average normal.  More importantly, if you calculate the normal by 
        // cross product then you get a normal whos lenght is 2 x the triangle 
        // area, so summing these gives us the correct weights.
        calcNormalUnNormalized(cur_normal, &xyz[v1 * 3], &xyz[v2 * 3], 
          &xyz[v3 * 3]);

        norm_xyz[v1 * 3] += cur_normal[0];
        norm_xyz[v1 * 3 + 1] += cur_normal[1];
        norm_xyz[v1 * 3 + 2] += cur_normal[2];

        norm_xyz[v2 * 3] += cur_normal[0];
        norm_xyz[v2 * 3 + 1] += cur_normal[1];
        norm_xyz[v2 * 3 + 2] += cur_normal[2];

        norm_xyz[v3 * 3] += cur_normal[0];
        norm_xyz[v3 * 3 + 1] += cur_normal[1];
        norm_xyz[v3 * 3 + 2] += cur_normal[2];
        break;
      case RobustNormalApproximation:
        // METHOD 3 --> EXPENSIVE
        // Weight the normals by the angle they form at the vertex.
        calcNormalUnNormalized(cur_normal, &xyz[v1 * 3], &xyz[v2 * 3], 
          &xyz[v3 * 3]);
        // V2 Angle
        float angle = calcAngleSafe(&xyz[v1 * 3], &xyz[v2 * 3], &xyz[v3 * 3]);
        norm_xyz[v1 * 3] += (cur_normal[0] * angle);
        norm_xyz[v1 * 3 + 1] += (cur_normal[1] * angle);
        norm_xyz[v1 * 3 + 2] += (cur_normal[2] * angle);
        // V1 Angle
        angle = calcAngleSafe(&xyz[v3 * 3], &xyz[v1 * 3], &xyz[v2 * 3]);
        norm_xyz[v2 * 3] += (cur_normal[0] * angle);
        norm_xyz[v2 * 3 + 1] += (cur_normal[1] * angle);
        norm_xyz[v2 * 3 + 2] += (cur_normal[2] * angle);
        // V3 Angle
        angle = calcAngleSafe(&xyz[v2 * 3], &xyz[v3 * 3], &xyz[v1 * 3]);
        norm_xyz[v3 * 3] += (cur_normal[0] * angle);
        norm_xyz[v3 * 3 + 1] += (cur_normal[1] * angle);
        norm_xyz[v3 * 3 + 2] += (cur_normal[2] * angle);
        break;
      }
    }

    for (uint32_t i = 0; i < w * h; i++) { 
      // No need to divide by the number of faces...  Just normalize
      float* norm = &norm_xyz[i * 3];
      float len_sq = norm[0] * norm[0] + norm[1] * norm[1] + norm[2] * norm[2];
      if (len_sq > EPSILON) {
        float len = sqrtf(len_sq);
        norm[0] /= len;
        norm[1] /= len;
        norm[2] /= len;
      }
    }
  }

  void CalcNormalImage(float* normals_xyz, const float* xyz, 
    const uint32_t w, const uint32_t h, const float dist_cutoff, 
    const NormalApproximationMethod method) {
    jtil::data_str::Vector<jtil::math::Int3> indices_;

    createIndices(indices_, xyz, w, h, dist_cutoff);
    createNormals(normals_xyz, xyz, indices_, w, h, method);
  }

}  // namespace image_util
}  // namespace jtil
