//
//  window.h
//
//  Created by Jonathan Tompson on 4/26/12.
//
//  A wrapper for windowing using GLFW (2.7.7 on mac and 2.7.6 on windows).
//  Even though GLFW is cross platform, this layer is added as an abstraction 
//  in case I want to switch windowing libraries and it also keeps App free of
//  implementation specific API calls.  Dynamic window resizing is not 
//  supported since it's not that useful anyway in most rendering contexts.
//
//  Note, a limitation of this class is that only one window can be open at any
//  one time.  TODO: Change g_window to a list of windows and then have
//  static callback methods scan the list for the correct window's callback.
//
//  Typical Useage:
//
//  void keyboardFunc(int key, int action);
//  void mousePosFunc(int x, int y);
//  void mouseButtonFunc(int button, int action);
//  void mouseWheelFunc(int pos);
//
//  void main {
//    Window::initWindowSystem();
//    WindowSettings settings;
//    settings.width = xxx;
//    ... etc ...
//    Window* wnd = new Window(settings);
//    wnd->registerKeyboardCB(&keyboardFunc);
//    wnd->registerMousePosCB(&mousePosFunc);
//    ... do stuff ...
//    delete wnd;
//    Window::killWindowSystem();
//  }
// 

#pragma once

#include <mutex>
#include <string>
#include "math/math_types.h"  // for Int4
#include "windowing/keys_and_buttons.h"  // for NUM_KEYS
#include "windowing/window_settings.h"
#include "windowing/window_cb.h"
#include "windowing/window_interface.h"

#define MESSAGE_LOOP_POLL_TIME_MS 50

struct GLFWwindow;

namespace jtil {
namespace windowing {

  typedef enum {
    RES_640_480     = ((640 << 16) | 480),  // 41943520
    RES_800_600     = ((800 << 16) | 600),  // 52429400
    RES_1280_1024   = ((1280 << 16) | 1024),
    RES_1920_1200   = ((1920 << 16) | 1200),
    RES_1280_720    = ((1280 << 16) | 720),
    RES_1920_1080   = ((1920 << 16) | 1080),  // 125830200
    RES_1000_1000   = ((1000 << 16) | 1000),
    RES_960_720    = ((960 << 16) | 720),  // Kinect Resolution * 2
    RES_1280_960    = ((1280 << 16) | 960),  // Kinect Resolution * 2
    RES_1920_1440   = ((1920 << 16) | 1440),  // Kinect Resolution * 3
    NUM_RES = 10,
  } WINDOW_RES;

  class Window : public WindowInterface {
  public:
    Window(const WindowSettings& settings);
    ~Window();

    static void initWindowSystem();
    static void killWindowSystem();
    void swapBackBuffer() const ;  // Makes any API calls necessary

    // Poll the mouse and keyboard state on demand
    virtual const bool getKeyState(const int key) const;
    virtual const bool getMousePosition(math::Double2& pos) const;
    virtual const bool getMouseButtonStateRight() const;
    virtual const bool getMouseButtonStateLeft() const;
    virtual const bool getMouseButtonStateMiddle() const;
    
    // Some getter methods
    virtual const int width() const;
    virtual const int height() const;
    virtual const bool fullscreen() const;
    virtual const bool isOpen() const;
    virtual const bool getDoubleBuffering() const;
    virtual void setDoubleBuffering(const bool double_buffer);

    // Because on Mac OS X the actual viewport wh may differ from the true
    // wh that we asked for, we need to store the actual value after opening.
    // This should only be of concern to the renderer when setting the last
    // viewport (when it is time to render to the screen).
    virtual const int viewport_width() const;
    virtual const int viewport_height() const;

    // A helper function to turn enumerated resolutions into width and height
    // NOTE: This should only be called ONCE on startup (or on screen resize) 
    // to get the desired w and h from the settings enum.  You should call
    // width() and height() methods to get the actual resolution as it may
    // differ.
    static void windowResEnumToInt(const int res, int& w, int& h);
    
    // Add callback functions to get imediate updates on a mouse or key event
    virtual void registerKeyboardCB(KeyboardCBFuncPtr callback);
    virtual void registerMousePosCB(MousePosCBFuncPtr callback);
    virtual void registerMouseButCB(MouseButCBFuncPtr callback);
    virtual void registerMouseWheelCB(MouseWheelCBFuncPtr callback);
    // virtual void registerCharInputCB(CharInputCBFuncPtr callback);
    virtual void registerCloseWndCB(CloseWndCBFuncPtr callback);

    static const Window* g_window() { return g_window_; }

  private:
    WindowSettings settings_;

    KeyboardCBFuncPtr keyboard_cb_;
    MousePosCBFuncPtr mouse_pos_cb_;
    MouseButCBFuncPtr mouse_button_cb_;
    MouseWheelCBFuncPtr mouse_wheel_cb_;
    // CharInputCBFuncPtr character_input_cb_;
    CloseWndCBFuncPtr close_cb_;
    
    static math::Double2 mouse_wheel_pos_;
    static bool key_down_[NUM_KEYS];
    static bool mouse_button_down_[NUM_MOUSE_BUTTONS];
    static math::Double2 mouse_pos_;
    static bool init_glew_;
    static std::mutex window_init_lock_;
    static bool init_window_system_;
    static uint32_t num_windows_;
    GLFWwindow* glfw_window_;  // For internal use only.
    static Window* g_window_;  // SINGLETON CLASS
    
    // Callbacks for GLFW --> Will also send keyboard events to the app class
    static void keyboardInputCB(GLFWwindow* wnd, int key, int scancode, 
      int action, int mods);
    static void mousePosCB(GLFWwindow* wnd, double x, double y);
    static void mouseButtonCB(GLFWwindow* wnd, int button, int action,
      int mods);
    static void mouseWheelCB(GLFWwindow* wnd, double xoffset, double yoffset);
    static void closeCB(GLFWwindow* wnd);
    static void errorCallback(int error, const char* description);
    
    // Non-copyable, non-assignable.
    Window(Window&);
    Window& operator=(const Window&);
  };

  void NativeErrorBox(const wchar_t* str);

};  // namespace windowing
};  // namespace jtil
