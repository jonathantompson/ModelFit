//
//  window_interface.h
//
//  Created by Jonathan Tompson on 2/25/12.
//
//  Pure virtual interface declaration.
// 

#pragma once

#include "math/math_types.h"  // for Int2
#include "windowing/keys_and_buttons.h"  // for NUM_KEYS
#include "windowing/window_cb.h"

namespace jtil {
namespace windowing {

  class WindowInterface {
  public:
    // Poll the mouse and keyboard state on demand
    virtual const bool getKeyState(const int key) const = 0;
    virtual const bool getMousePosition(math::Double2& pos) const = 0;
    virtual const bool getMouseButtonStateRight() const = 0;
    virtual const bool getMouseButtonStateLeft() const = 0;
    virtual const bool getMouseButtonStateMiddle() const = 0;
    
    // Some getter methods
    virtual const int width() const = 0;
    virtual const int height() const = 0;
    virtual const bool fullscreen() const = 0;
    virtual const bool isOpen() const = 0;
    virtual const bool getDoubleBuffering() const = 0;
    virtual void setDoubleBuffering(const bool double_buffer) = 0;

    // Because on Mac OS X the actual viewport wh may differ from the true
    // wh that we asked for, we need to store the actual value after opening.
    // This should only be of concern to the renderer when setting the last
    // viewport (when it is time to render to the screen).
    virtual const int viewport_width() const = 0;
    virtual const int viewport_height() const = 0;
    
    // Add callback functions to get imediate updates on a mouse or key event
    virtual void registerKeyboardCB(KeyboardCBFuncPtr callback) = 0;
    virtual void registerMousePosCB(MousePosCBFuncPtr callback) = 0;
    virtual void registerMouseButCB(MouseButCBFuncPtr callback) = 0;
    virtual void registerMouseWheelCB(MouseWheelCBFuncPtr callback) = 0;
  };

};  // namespace windowing
};  // namespace jtil
