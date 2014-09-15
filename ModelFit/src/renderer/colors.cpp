#include "renderer/colors.h"

using jtil::math::Float3;

namespace renderer {

  // Some from here: http://www.w3schools.com/Html/html_colors.asp
  const Float3 white(1.0f, 1.0f, 1.0f);
  const Float3 black(0.0f, 0.0f, 0.0f);
  const Float3 red(1.0f, 0.0f, 0.0f);
  const Float3 lred(1.0f, 0.5f, 0.5f);
  const Float3 green(0.0f, 1.0f, 0.0f);
  const Float3 lgreen(0.5f, 1.0f, 0.5f);
  const Float3 blue(0.0f, 0.0f, 1.0f);
  const Float3 lblue(0.5f, 0.5f, 1.0f);
  const Float3 yellow(1.0f, 1.0f, 0.0f);
  const Float3 pink(1.0f, 0.0f, 1.0f);
  const Float3 cyan(0.0f, 1.0f, 1.0f);
  const Float3 grey(0.75294117f, 0.75294117f, 0.75294117f);
  const Float3 gold(1.0f, 0.84313725f, 0.0f);

  const Float3 colors[n_colors] = {white, red, green, blue, 
    yellow, pink, cyan, grey, gold, black, lred, lgreen, lblue};

};  // renderer namespace
