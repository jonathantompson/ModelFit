//
//  window_cb.h
//
//  Created by Jonathan Tompson on 2/25/13.
//

#pragma once

struct GLFWwindow;

namespace jtil {
namespace windowing {
  
  typedef void (*KeyboardCBFuncPtr)(int key, int scancode, int action, 
    int mods);
  typedef void (*MousePosCBFuncPtr)(double x, double y);
  typedef void (*MouseButCBFuncPtr)(int button, int action, int mods);
  typedef void (*MouseWheelCBFuncPtr)(double xoffset, double yoffset);
  typedef int (*CloseWndCBFuncPtr)();

};  // namespace windowing
};  // namespace jtil
