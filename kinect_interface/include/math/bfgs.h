//
//  bfgs.h
//
//  Created by Jonathan Tompson on 11/20/12.
//
//  A black box implementation of the BFGS with linear-search
//  backtracking non-linear optimization algorithm.  Requires Jacobian function
//  (which you can estimate using central differencing).
//
//  http://en.wikipedia.org/wiki/BFGS_method
//
//  Note, for numerical stability you should always try and use doubles!
// 

#pragma once

#include <iostream>
#include <random>
#include "math/math_types.h"
#include "math/common_optimization.h"

#if defined(WIN32) || defined(_WIN32)
#pragma warning( push )
#pragma warning( disable: 4244 )
#endif

#include <Eigen/Eigen>

#if defined(WIN32) || defined(_WIN32)
  #define constexpr 
#endif

// C++11 alias declaration --> Wont compile in Visual Studio (not yet supported)
//template <class T>
//using MatDynamic = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

namespace jtil {
namespace math {

  // BFGS with backtracking optimization
  template <class T>
  class BFGS {
  public:
    BFGS(uint32_t num_coeffs);
    ~BFGS();

    // minimize():
    // - end_c: The output value.
    // - start_c: The starting inital guess.
    // - angle_coeff: Since the mid-point of two angles needs to be calc'ed
    //   differently, the user can supply an array of boolians indicating that 
    //   the i-th coefficient is an angle.
    // - obj_func: Function to minimize.
    // - jac_func: Derivative function of obj_func.
    // - coeff_norm_func: After coefficient update, some coefficients may need
    //   normalization.  Set to NULL if not needed.
    void minimize(T* end_c,
                  const T* start_c,
                  const bool* angle_coeff,  // can be NULL
                  T (*obj_func)(const T* coeff),
                  void (*jac_func)(T* jacob, const T* coeff),
                  void (*coeff_update_func)(T* coeff));  // can be NULL

    // Termination and Optimization settings:
    T c1;  // Default: 1e-4: Armijo sufficient descent parameter (lower
           // is more aggressive) page 39, 3.6a of Nocedal and Wright
    T c2;  // Default: 0.9: Wolfe sufficient descent parameter
           // page 39, 3.6b of Nocedal and Wright
    T gamma;  // Default: 0.5:  Backtracking search contraction parameter
    uint64_t max_iterations;
    T jac_2norm_term;  // Default 1e-4;
    T delta_x_2norm_term;  // Default 1e-5;
    T delta_f_term;  // Default 1e-5;
    bool verbose;
    SufficientDescentCondition descent_cond;

    // http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:
    uint32_t num_coeffs_;
    T alpha_k_;
   
    T f_k_;
    T f_k_p1_;

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> x_k_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> x_k_p1_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> B_inv_k_;  // Approximate Inverse Hessian
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> B_inv_k_p1_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> J_k_;  // Jacobian
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> J_k_p1_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> y_k_;  // J_k_p1_ - J_k_
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> p_k_;  // Search direction
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> s_k_;  // alpha_k_ * p_k_

    // Temp vectors
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> J_k_tran_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> J_k_p1_tran_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> J_k_tran_p_k_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> J_k_p1_tran_p_k_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> s_k_tran_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> y_k_tran_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> s_k_tran_y_k_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> s_k_y_k_tran_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> s_k_s_k_tran_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> y_k_s_k_tran_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> y_k_tran_B_inv_k_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> tmp_scalar1_;
  };

  // Some helper functions...
  namespace bfgs {
    // ret = a + interp_val * (b - c)
    template <class T>
    void interpolateCoeff(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& ret, 
      const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& a,
      const T interp_val, 
      const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& b, 
      const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& c,
      const bool* angle_coeff) {
      for (uint32_t i = 0; i < a.size(); i++) {
        if (angle_coeff != NULL && angle_coeff[i]) {
          T real_a = (T)cos((double)a(i));
          T imag_a = (T)sin((double)a(i));
          T real_b = (T)cos((double)b(i));
          T imag_b = (T)sin((double)b(i));
          T real_c = (T)cos((double)c(i));
          T imag_c = (T)sin((double)c(i));
          T real_interp = real_a + interp_val * (real_b - real_c);
          T imag_interp = imag_a + interp_val * (imag_b - imag_c);
          T interp_angle = (T)atan2((double)imag_interp, (double)real_interp);
          ret(i) = interp_angle;
        } else {
          ret(i) = a(i) + interp_val * (b(i) - c(i));
        }
      }
    }

    // ret = a + interp_val * b
    template <class T>
    void interpolateCoeff(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& ret, 
      const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& a,
      const T interp_val, 
      const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& b, 
      const bool* angle_coeff) {
      for (uint32_t i = 0; i < a.size(); i++) {
        if (angle_coeff != NULL && angle_coeff[i]) {
          T real_a = (T)cos((double)a(i));
          T imag_a = (T)sin((double)a(i));
          T real_b = (T)cos((double)b(i));
          T imag_b = (T)sin((double)b(i));
          T real_interp = real_a + interp_val * real_b;
          T imag_interp = imag_a + interp_val * imag_b;
          T interp_angle = (T)atan2((double)imag_interp, (double)real_interp);
          ret(i) = interp_angle;
        } else {
          ret(i) = a(i) + interp_val * b(i);
        }
      }
    }
  }

  template <class T>
  BFGS<T>::BFGS(uint32_t num_coeffs) {
    num_coeffs_ = num_coeffs;

    x_k_.resize(num_coeffs, 1);
    x_k_p1_.resize(num_coeffs, 1);
    B_inv_k_.resize(num_coeffs, num_coeffs);
    B_inv_k_p1_.resize(num_coeffs, num_coeffs);
    J_k_.resize(num_coeffs, 1);
    J_k_p1_.resize(num_coeffs, 1);
    y_k_.resize(num_coeffs, 1);
    p_k_.resize(num_coeffs, 1);
    s_k_.resize(num_coeffs, 1);

    J_k_tran_.resize(1, num_coeffs);
    J_k_p1_tran_.resize(1, num_coeffs);
    J_k_tran_p_k_.resize(1, 1);
    J_k_p1_tran_p_k_.resize(1,1);
    s_k_tran_.resize(1, num_coeffs);
    y_k_tran_.resize(1, num_coeffs);
    s_k_tran_y_k_.resize(1, 1);
    s_k_y_k_tran_.resize(num_coeffs, num_coeffs);
    s_k_s_k_tran_.resize(num_coeffs, num_coeffs);
    y_k_s_k_tran_.resize(num_coeffs, num_coeffs);
    y_k_tran_B_inv_k_.resize(1, num_coeffs);

    // Some default parameters
    max_iterations = 1000;
    c1 = (T)1e-4;  // page 200 of Nocedal and Wright
    c2 = (T)0.9;
    jac_2norm_term = (T)1e-4;
    gamma = (T)0.5;
    delta_x_2norm_term = (T)1e-5;
    delta_f_term = (T)1e-5;
    verbose = false;
    descent_cond = ARMIJO;
  }

  template <class T>
  BFGS<T>::~BFGS() {
    // Nothing to do
  }

  template <class T>
  void BFGS<T>::minimize(T* end_c, const T* start_c, const bool* angle_coeff,
    T (*obj_func)(const T* coeff), void (*jac_func)(T* jacob, const T* coeff),
    void (*coeff_update_func)(T* coeff)) {
    if (verbose) {
      std::cout << "Starting BFGS with backtracking optimization..." << std::endl;
    }

    for (uint32_t i = 0; i < num_coeffs_; i++) {
      x_k_(i) = start_c[i];
    }
    f_k_ = obj_func(x_k_.data());
    jac_func(J_k_.data(), x_k_.data());
    B_inv_k_.setZero();
    for (uint32_t i = 0; i < num_coeffs_; i++) {
      B_inv_k_(i,i) = 1.0;
    }

    uint32_t no_iterations = 0;
    while (no_iterations < max_iterations) {
      if (verbose) {
        std::cout << "****************************************************" << std::endl;
        std::cout << "Iteration: " << no_iterations << std::endl;
        std::cout << "   cur_f_ = " << f_k_ << std::endl;
      }

      J_k_tran_ = J_k_.transpose();
      T jac_2normsq_ = J_k_.squaredNorm();
      if (verbose) {
        std::cout << "   (||J||_2)^2 = " << jac_2normsq_ << std::endl;
      }
      if (jac_2normsq_ < (jac_2norm_term * jac_2norm_term)) {
        if (verbose) {
          std::cout << "   jac_2norm_ < jac_2norm_term" << std::endl;
        }
        break;
      }

      // Search direction satisfies H_k_ * p_k_ = -J_k_
      p_k_ = -B_inv_k_ * J_k_;  

      // Otherwise perform a backtracking line search:
      alpha_k_ = 1;  // page 200 of Nocedal and Wright says always start with 1
      uint32_t num_alpha_iterations = 0;
      bfgs::interpolateCoeff(x_k_p1_, x_k_, alpha_k_, 
        p_k_, angle_coeff);
      if (coeff_update_func) { 
        coeff_update_func(x_k_p1_.data());
      }
      f_k_p1_ = obj_func(x_k_p1_.data());

      if (descent_cond == STRONG_WOLFE) {
        jac_func(J_k_p1_.data(), x_k_p1_.data());
        J_k_p1_tran_ = J_k_p1_.transpose();
        J_k_p1_tran_p_k_ = J_k_p1_tran_ * p_k_;
      }

      J_k_tran_p_k_ = J_k_tran_ * p_k_;  // avoid redundant calc
      while ((f_k_p1_ > (f_k_ + c1 * alpha_k_ * J_k_tran_p_k_(0)) ||  // 3.6a, page 39 N&W
        (descent_cond == STRONG_WOLFE && fabs(J_k_p1_tran_p_k_(0)) < fabs(c2 * J_k_tran_p_k_(0)))) &&   // 3.6b, page 39 N&W
        num_alpha_iterations < 64) {
          // sufficient decrease condition not met: contract the step
          alpha_k_ *= gamma;
          bfgs::interpolateCoeff(x_k_p1_, x_k_, alpha_k_, 
            p_k_, angle_coeff);
          if (coeff_update_func) {
            coeff_update_func(x_k_p1_.data());
          }
          f_k_p1_ = obj_func(x_k_p1_.data());
          if (descent_cond == STRONG_WOLFE) {
            jac_func(J_k_p1_.data(), x_k_p1_.data());
            J_k_p1_tran_ = J_k_p1_.transpose();
            J_k_p1_tran_p_k_ = J_k_p1_tran_ * p_k_;
          }
          num_alpha_iterations++;
      }

      if (num_alpha_iterations >= 64) {
        if (verbose) {
          std::cout << "   num_alpha_iterations >= 64 " << std::endl;
        }
        break;
      }

      if (verbose) {
        std::cout << "   alpha = " << alpha_k_ << std::endl;
      }

      // Take the step:
      s_k_ = x_k_p1_ - x_k_; 
      s_k_tran_ = s_k_.transpose();
      T delta_x_2normsq = s_k_.squaredNorm();
      T delta_f = f_k_ - f_k_p1_;  // Guaranteed positive

      if (verbose) {
        std::cout << "   (||DeltaX||_2)^2 = " << delta_x_2normsq << std::endl;
        std::cout << "   |delta_f| = " << delta_f << std::endl;
      }
      if (delta_x_2normsq < (delta_x_2norm_term * delta_x_2norm_term)) {
        if (verbose) {
          std::cout << "   delta_x_2norm < delta_x_2norm_term" << std::endl;
        }
        break;
      }
      if (delta_f < delta_f_term) {
        if (verbose) {
          std::cout << "   delta_f < delta_f_term" << std::endl;
        }
        break;
      }

      // Update the new Jacobian
      jac_func(J_k_p1_.data(), x_k_p1_.data());
      y_k_ = J_k_p1_ - J_k_;
      y_k_tran_ = y_k_.transpose();

      // Update the Hessian inverse
      //http://en.wikipedia.org/wiki/BFGS_method
      s_k_tran_y_k_ = s_k_tran_ * y_k_;
      s_k_y_k_tran_ = s_k_ * y_k_tran_;
      s_k_s_k_tran_ = s_k_ * s_k_tran_;
      y_k_s_k_tran_ = y_k_ * s_k_tran_;
      y_k_tran_B_inv_k_ = y_k_tran_ * B_inv_k_;

      tmp_scalar1_ = (s_k_tran_y_k_ + y_k_tran_B_inv_k_ * y_k_) / (s_k_tran_y_k_(0)*s_k_tran_y_k_(0));
      B_inv_k_p1_ = B_inv_k_ + (tmp_scalar1_(0) * s_k_s_k_tran_)
        - (B_inv_k_ * y_k_s_k_tran_ + s_k_y_k_tran_ * B_inv_k_) / (s_k_tran_y_k_(0));
      
      // Get ready for the next iteration
      x_k_ = x_k_p1_;
      f_k_ = f_k_p1_;
      J_k_ = J_k_p1_;
      B_inv_k_ = B_inv_k_p1_;

      no_iterations++;
    }

    // TO DO: try a restart with identity inverse hessian

    if (verbose) {
      std::cout << "BFGS with backtracking finished with f = " << f_k_;
      std::cout << " (" << no_iterations << " iterations)" << std::endl;
    }

    for (uint32_t i = 0; i < num_coeffs_; i++) {
      end_c[i] = x_k_(i);
    }
  }

};  // namespace math
};  // namespace jtil

#if defined(WIN32) || defined(_WIN32)
#pragma warning( pop )
#endif
