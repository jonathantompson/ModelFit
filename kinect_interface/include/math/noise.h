//
//  noise.h
//  ModelVis
//
//  Created by Murphy M Stein on 10/23/12.
//  Edited by Jonathan Tompson on 11/8/2012
//  Copyright (c) 2012 Murphy M Stein. All rights reserved.
//
//  An easy interface to generate random keyframes and then use cubic
//  interpolation in order to animate between them.
//  

#pragma once

#if defined(WIN32) || defined(_WIN32)
#include <functional> 
#include <stdint.h>
#endif
#include <random>
#include <math.h>
#include <iostream>

namespace jtil {
namespace math {

  template <class T>
  class Noise {
  public:
    Noise(uint32_t num_samples);
    ~Noise();

    // Sample noise function at f(x)
    T sample(T _x);

    void setKeyframe(uint32_t i, T y);

  private:
    uint32_t n;
    T* g;

    T lerp(T t, T a, T b);
    T cubic_interpolate(T mu, T y0, T y1, T y2, T y3);  // interp between y1 and y2
  };

  template <class T>
  Noise<T>::~Noise() {
    delete[] g;
  }

  template <class T>
  Noise<T>::Noise(uint32_t num_samples) {
    g = new T[num_samples];
    n = num_samples;
    std::random_device rd;
    std::mt19937 gen(rd());
    auto uf = std::bind(std::uniform_real_distribution<float>(-1,1), gen);
    g[0] = 0;

    for (uint32_t i = 1; i < n; i++) {
      g[i] = uf();
    }
    // g[n-1] = g[0];
  }

  template <class T>
  void Noise<T>::setKeyframe(uint32_t i, T y) {
    if (i >= n) {
      throw std::runtime_error("setKeyframe() ERROR: - index overflow!"); 
    }
    g[i] = y;
  }

  template <class T>
  T Noise<T>::lerp(T t, T a, T b) {
    return (1-t) * a + t * b; 
  }

  // http://paulbourke.net/miscellaneous/interpolation/
  // This is Catmull-Rom splines
  template <class T>
  T Noise<T>::cubic_interpolate(T mu, T y0, T y1, T y2, T y3) {
    T mu2 = mu * mu;
    T a0 = ((T)-0.5)*y0 + ((T)1.5)*y1 - ((T)1.5)*y2 + ((T)0.5)*y3;
    T a1 = y0 - ((T)2.5)*y1 + ((T)2)*y2 - ((T)0.5)*y3;
    T a2 = ((T)-0.5)*y0 + ((T)0.5)*y2;
    T a3 = y1;

   return (a0*mu*mu2+a1*mu2+a2*mu+a3);
  }

  template <class T>
  T Noise<T>::sample(T _x) {
    T x = (T)fmod(_x,n);
    T t = x - (T)floor(x);
    uint32_t x1 = static_cast<uint32_t>(floor(x)) % n;
    uint32_t x2 = (x1 + 1) % n;
    uint32_t x3 = (x1 + 2) % n;
    uint32_t x0 = x1 > 0 ? (x1 - 1) : (n - 1);
    return cubic_interpolate(t, g[x0], g[x1], g[x2], g[x3]);
    // return lerp(t, g[x1], g[x2]);
  }

};  // namespace math
};  // namespace jtil
