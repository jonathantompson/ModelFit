//
//  lm_fit.h
//
//  Created by Jonathan Tompson on 8/23/12.
//
//  A black box implementation of the Levenberg-Marquardt non-linear
//  least squares fitting.
//
//  This class and its supporting functions takes in a set of data points y,
//  evaluated at points x and will perform a non-linear fit to the function 
//  f(x,c) (x in R^n, c in R^m, f(x,c) in R^1), by modifying c.  The residual 
//  function is used to evaluate the "quality" of the fit (otherwise 
//  the L2 norm of the error is used if no residual function is supplied).
//
//  Note, for numerical stability you should always try and use doubles!
// 

#pragma once

#include <iostream>
#include "math/math_types.h"
#include "math/common_optimization.h"

#if defined(WIN32) || defined(_WIN32)
#pragma warning( push )
#pragma warning( disable: 4244 )
#endif

#include <Eigen/Eigen>

namespace jtil {
namespace math {

  // Levenberg-Marquardt optimization
  template <class T>
  class LMFit {
  public:
    LMFit(uint32_t c_dim, uint32_t x_dim, uint32_t num_pts);
    ~LMFit();

    // fitModel = Top level function:
    // - end_c: Return value for coefficient c
    // - start_c: Starting value for coefficient c
    // - y: y data to fit to
    // - x: x sample points
    // - eval_func: Function pointer to function to fit: takes in a set of 
    //   coefficients and a set of x evaluation points and returns f(x,c).
    // - jacobian_func: Function pointer to jacobian calculation: should return
    //   J_ji = df(x_j,c) / dc_i.   That is for each element x_j it needs to 
    //   calculate the parital deriviative wrt c_i.
    // - obj_func: Function to calculate residue (error) per iteration.
    //   set to NULL to let LMFitting use the L2 norm as residue.
    // - coeff_norm_func: After coefficient update, some coefficients may need
    //   normalization.  Set to NULL if not needed.
    void fitModel(T* end_c,
                  const T* start_c,
                  const T* y,  // (y_1, y_2, ...)
                  const T* x,  // ((x1_1, ... x1_xdim), (x2_1, ... x2_xdim), ...)
                  T (*fit_func)(const T* x, const T* c),
                  void (*jacob_func)(T* jacob, const T* x, const T* c),
                  T (*residue_func)(const T* y, const T* y_fit) = NULL,
                  void (*coeff_update_func)(T* coeff) = NULL);  // can be NULL

    // Termination and Optimization settings:
    T delta_c_termination;  // L2_norm of delta C termination criterion
    T delta_residue_termination;  // delta residue termination criterion
    T lambda_start;  // Starting lambda value
    T lambda_min;  // minimum lambda value
    T lambda_max;  // maximum lambda value
    uint64_t max_iterations;  // Maximum number of LM iterations
    bool verbose;

    // http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:
    uint32_t c_dim_;
    uint32_t x_dim_;
    uint32_t num_pts_;

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> jacobian_k_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> jacobian_k_tran_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> delta_c_k_p1_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> delta_c_k_p2_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> normal_mat_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> cur_normal_mat_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> coeff_k_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> best_coeff_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> coeff_tmp_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> coeff_k_p1_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> coeff_k_p2_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> f_k_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> f_k_p1_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> f_k_p2_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> f_k_partial_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> delta_y_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> delta_y_prime_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> delta_y_p12_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> residual_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> y_;
  };

  template <class T>
  LMFit<T>::LMFit(uint32_t c_dim, uint32_t x_dim, uint32_t num_pts) {
    c_dim_ = c_dim;
    x_dim_ = x_dim;
    num_pts_ = num_pts;

    // Some default parameters
    delta_c_termination = (T)1e-10;
    delta_residue_termination = (T)1e-10;
    lambda_start = (T)1e-6;
    lambda_min = (T)1e-12;
    lambda_max = (T)1e12;
    max_iterations = 1000;
    verbose = false;

    jacobian_k_.resize(num_pts, c_dim);
    jacobian_k_tran_.resize(c_dim, num_pts);
    delta_c_k_p1_.resize(1, c_dim);
    delta_c_k_p2_.resize(1, c_dim);
    normal_mat_.resize(c_dim, c_dim);
    cur_normal_mat_.resize(c_dim, c_dim);
    coeff_k_.resize(1, c_dim);
    coeff_k_p1_.resize(1, c_dim);
    coeff_k_p2_.resize(1, c_dim);
    f_k_.resize(num_pts, 1);
    y_.resize(num_pts, 1);
    f_k_p1_.resize(num_pts, 1);
    f_k_p2_.resize(num_pts, 1);
    delta_y_.resize(num_pts, 1);
    delta_y_p12_.resize(num_pts, 1);
    delta_y_prime_.resize(c_dim, 1);
    residual_.resize(num_pts, 1);
  }

  template <class T>
  LMFit<T>::~LMFit() {
    // Empty
  }

  template <class T>
  void LMFit<T>::fitModel(T* end_c, const T* start_c, const T* y, const T* x,
    T (*fit_func)(const T* x, const T* c), 
    void (*jacob_func)(T* jacob, const T* x, const T* c),
    T (*residue_func)(const T* y, const T* y_fit), 
    void (*coeff_update_func)(T* coeff)) {

    for (uint32_t i = 0; i < c_dim_; i++) {
      coeff_k_(i) = start_c[i];
    }
    best_coeff_ = coeff_k_;
    T r_k;
    T best_r_k;

    // Convert y values to eigen
    for (uint32_t i = 0; i < num_pts_; i++) {
      y_(i) = y[i];
    }

    if (coeff_update_func != NULL) {  // Just in case make sure coeff_k_ is valid
      coeff_update_func(coeff_k_.data());
    }
    uint64_t iteration_num = 1;
    T norm_delta_c_k = std::numeric_limits<T>::infinity();
    T delta_r_k = std::numeric_limits<T>::infinity();
    T lambda_k = lambda_start;

    // Calculate the starting residual and Jacobian
    for (uint32_t i = 0; i < num_pts_; i++) {
      f_k_(i) = fit_func(&x[i * x_dim_], coeff_k_.data());
      // NOTE: jacobian_k_ IS ROW MAJOR
      jacob_func(&jacobian_k_(i, 0), &x[i * x_dim_], coeff_k_.data());
    }

    jacobian_k_tran_ = jacobian_k_.transpose();
    normal_mat_ = jacobian_k_tran_ * jacobian_k_;
    delta_y_ = y_ - f_k_;
    delta_y_prime_ = jacobian_k_tran_ * delta_y_;  // RHS 'b' of 'Ax = b'

    if (residue_func != NULL) {
      r_k = residue_func(y, f_k_.data());
    } else {
      Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> r_k_mat;
      r_k_mat = delta_y_.transpose() * delta_y_;   // the dot product
      r_k = r_k_mat(0); 
    }
     
    best_r_k = r_k;
    best_coeff_ = coeff_k_;

    // Main optimization loop
    do {
      if (verbose) {
        if (iteration_num != 1) {
          // Print the results of the last iteration
	  std::cout << "Iteration " << iteration_num - 1 << std::endl;
	  std::cout << "  --> 2-norm of delta_c is " << norm_delta_c_k << std::endl;
	  std::cout << "  --> lambda_k is " << lambda_k << std::endl;
	  std::cout << "  --> r_k is " << r_k << std::endl;
	  std::cout << "  --> delta_r_k is " << delta_r_k << std::endl;
        }
      }

      // Solve the 1st linear system (for the current lambda)
      cur_normal_mat_ = normal_mat_;
      for (uint32_t i = 0; i < c_dim_; i++) {
        // Add the Levenberg Marquardt dapening factor (down the diagonals)
        cur_normal_mat_(i, i) = cur_normal_mat_(i, i) + 
          (lambda_k * cur_normal_mat_(i, i));
      }
      delta_c_k_p1_ = 
        cur_normal_mat_.householderQr().solve(delta_y_prime_);
      if(!delta_y_prime_.isApprox(cur_normal_mat_ * delta_c_k_p1_)) {
        // Matrix might not be positive definite 
        // --> Either way, cannot perform cholesky factorization.
        lambda_k = lambda_k * 2;
        iteration_num++;
        continue;  // Quit this iteration and start again
      }

      // Compute the new coefficients and it's residual
      coeff_k_p1_ = coeff_k_ + delta_c_k_p1_.transpose();
      if (coeff_update_func != NULL) {  // Just in case make sure coeff_k_ is valid
        coeff_update_func(coeff_k_p1_.data());
      }
      for (uint32_t i = 0; i < num_pts_; i++) {
        f_k_p1_(i) = fit_func(&x[i * x_dim_], coeff_k_p1_.data());
      }

      T r_k_p1;
      if (residue_func != NULL) {
        r_k_p1 = residue_func(y, f_k_p1_.data());
      } else {
        delta_y_p12_ = y_ - f_k_p1_;
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> r_k_p1_mat;
        r_k_p1_mat = delta_y_p12_.transpose() * delta_y_p12_;
        r_k_p1 = r_k_p1_mat(0);
      }

      // Solve the 2nd linear system (for decreased current lambda)
      cur_normal_mat_ = normal_mat_;
      for (uint32_t i = 0; i < c_dim_; i++) {
        // Add the Levenberg Marquardt dapening factor (down the diagonals)
        cur_normal_mat_(i, i) = cur_normal_mat_(i, i) + 
          ((T)0.5 * lambda_k * cur_normal_mat_(i, i));
      }
      delta_c_k_p2_ = cur_normal_mat_.fullPivHouseholderQr().solve(delta_y_prime_);
      if(!delta_y_prime_.isApprox(cur_normal_mat_ * delta_c_k_p2_)) {
        // Matrix might not be positive definite 
        // --> Either way, cannot perform cholesky factorization.
        lambda_k = lambda_k * (T)2;
        iteration_num++;
        continue;  // Quit this iteration and start again
      }

      // Compute the new coefficients and it's residual
      coeff_k_p2_ = coeff_k_ + delta_c_k_p2_.transpose();
      if (coeff_update_func != NULL) {
        coeff_update_func(coeff_k_p2_.data());
      }
      for (uint32_t i = 0; i < num_pts_; i++) {
        f_k_p2_(i) = fit_func(&x[i * x_dim_], coeff_k_p2_.data());
      }
      T r_k_p2;
      if (residue_func != NULL) {
        r_k_p2 = residue_func(y, f_k_p2_.data());
      } else {
        delta_y_p12_ = y_ - f_k_p2_;
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> r_k_p2_mat;
        r_k_p2_mat = delta_y_p12_.transpose() * delta_y_p12_;
        r_k_p2 = r_k_p2_mat(0);
      } 

      // NOW WE HAVE 3 OPTIONS WITH 3 RESIDUALS:
      // 1. We couldn't make progress (r_k get's worse, so we need a higher
      //    dampening factor).
      // 2. The current dampening factor is perfect
      // 3. We made progress and we can reduce the dampening factor (for faster
      //    convergence on the next iteration).
      int residual_case;
      if (r_k <= r_k_p1) {
        if (r_k <= r_k_p2) {
          residual_case = 0;
        } else {
          residual_case = 2;
        }
      } else {
        if (r_k_p1 < r_k_p2) {
          residual_case = 1;
        } else {
          residual_case = 2;
        }
      }

      switch (residual_case) {
      case 0:
        // Stepping increased the residual, double the dampening factor and
        // try again without updating coefficients
        lambda_k = lambda_k * (T)2;
        break;
      case 1:
        lambda_k = lambda_k;
        delta_r_k = r_k - r_k_p1;
        r_k = r_k_p1;
        coeff_k_ = coeff_k_p1_;
        f_k_ = f_k_p1_;
#if defined(WIN32) || defined(_WIN32)
        norm_delta_c_k = delta_c_k_p1_.lpNorm<2>();
#else
        norm_delta_c_k = delta_c_k_p1_.template lpNorm<2>();
#endif
        break;
      case 2:
        lambda_k = lambda_k * (T)0.5;
        delta_r_k = r_k - r_k_p2;
        r_k = r_k_p2;
        coeff_k_ = coeff_k_p2_;
        f_k_ = f_k_p2_;
#if defined(WIN32) || defined(_WIN32)
        norm_delta_c_k = delta_c_k_p2_.lpNorm<2>();
#else
	norm_delta_c_k = delta_c_k_p2_.template lpNorm<2>();
#endif
        break;
      }

      if (residual_case != 0) {  // We made some progress --> Update the jacobian
        for (uint32_t i = 0; i < num_pts_; i++) {
          jacob_func(&jacobian_k_(i, 0), &x[i * x_dim_], coeff_k_.data());
        }
        jacobian_k_tran_ = jacobian_k_.transpose();
        normal_mat_ = jacobian_k_tran_ * jacobian_k_;
        delta_y_ = y_ - f_k_;
        delta_y_prime_ = jacobian_k_tran_ * delta_y_;  // RHS 'b' of 'Ax = b'
      }

      if (lambda_k < lambda_min) {
        lambda_k = lambda_min;
      }

      iteration_num++;
    } while (norm_delta_c_k > delta_c_termination &&
      delta_r_k > delta_residue_termination && 
      iteration_num < max_iterations && 
      lambda_k < lambda_max);

    if (r_k < best_r_k) {
      best_coeff_ = coeff_k_;
      best_r_k = r_k;
    }

    for (uint32_t i = 0; i < c_dim_; i++) {
      end_c[i] = best_coeff_(i);
    }
  }

};  // namespace math
};  // namespace jtil

#if defined(WIN32) || defined(_WIN32)
#pragma warning( pop )
#endif
