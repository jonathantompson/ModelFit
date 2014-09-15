//
//  colors.h
//
//  Created by Jonathan Tompson on 9/12/2012.
//

#pragma once

#include "math/math_types.h"

namespace renderer {

  extern const jtil::math::Float3 white;
  extern const jtil::math::Float3 black;
  extern const jtil::math::Float3 red;
  extern const jtil::math::Float3 lred;
  extern const jtil::math::Float3 green;
  extern const jtil::math::Float3 lgreen;
  extern const jtil::math::Float3 blue;
  extern const jtil::math::Float3 lblue;
  extern const jtil::math::Float3 yellow;
  extern const jtil::math::Float3 pink;
  extern const jtil::math::Float3 cyan;
  extern const jtil::math::Float3 grey;
  extern const jtil::math::Float3 gold;

  const uint32_t n_colors = 13;
  extern const jtil::math::Float3 colors[n_colors];

};  // renderer namespace
