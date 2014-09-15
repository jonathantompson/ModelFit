#include <cstring>
#include <string>
#include <sstream>
#include <iostream>
#include "windowing/glfw.h"
#include "windowing/window.h"

using std::wstring;

namespace jtil {

using math::Int2;

namespace windowing {

  // Internal static member variables.
  math::Double2 Window::mouse_wheel_pos_;
  bool Window::key_down_[NUM_KEYS];
  bool Window::mouse_button_down_[NUM_MOUSE_BUTTONS];
  math::Double2 Window::mouse_pos_;
  bool Window::init_glew_ = true;
  bool Window::init_window_system_ = true;
  uint32_t Window::num_windows_ = 0;
  std::mutex Window::window_init_lock_;
  Window* Window::g_window_;

  Window::~Window() {
    std::lock_guard<std::mutex> lk(window_init_lock_);
    glfwSetWindowSizeCallback(glfw_window_, NULL);
    glfwSetKeyCallback(glfw_window_, NULL);
    glfwSetCharCallback(glfw_window_, NULL);
    glfwSetCursorPosCallback(glfw_window_, NULL);
    glfwSetMouseButtonCallback(glfw_window_, NULL);
    glfwSetScrollCallback(glfw_window_, NULL);
    if (isOpen()) {
      glfwDestroyWindow(glfw_window_);
    }
    glfw_window_ = NULL;
    g_window_ = NULL;
    num_windows_--;
  } 

  Window::Window(const WindowSettings& settings) {
    std::lock_guard<std::mutex> lk(window_init_lock_);
    if (num_windows_ != 0) {
      throw std::runtime_error("Window::Window() - ERROR: A window is open.");
    }
    g_window_ = this;

    settings_ = settings;
    keyboard_cb_ = NULL;
    mouse_pos_cb_ = NULL;
    mouse_button_cb_ = NULL;
    mouse_wheel_cb_ = NULL;

    // Set the open hits for GLFW
    if (settings.samples > 1) {
      glfwWindowHint(GLFW_SAMPLES, settings.samples);
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, settings.gl_major_version);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, settings.gl_minor_version);
#if defined(DEBUG) || defined(_DEBUG)
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
#else
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_FALSE);
#endif
    if (settings.gl_core_profile) {
      glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
      glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    }
    glfwWindowHint(GLFW_SAMPLES, 0);
    glfwWindowHint(GLFW_AUX_BUFFERS, 0);
    glfwWindowHint(GLFW_STEREO, 0);
    
    // Prevent resize events
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

    GLFWmonitor* monitor = glfwGetPrimaryMonitor();

    // Open the GLFW window
    if (settings.fullscreen) {
	    glfw_window_ =  glfwCreateWindow(settings.width, settings.height,
        settings.title.c_str(), monitor, NULL);
    } else {
      const GLFWvidmode* mode = glfwGetVideoMode(monitor);
      glfwWindowHint(GLFW_BLUE_BITS, mode->blueBits);
      glfwWindowHint(GLFW_GREEN_BITS, mode->greenBits);
      glfwWindowHint(GLFW_RED_BITS, mode->redBits);
      glfwWindowHint(GLFW_DEPTH_BITS, settings.num_depth_bits);
      glfwWindowHint(GLFW_STENCIL_BITS, settings.num_stencil_bits);
      glfwWindowHint(GLFW_REFRESH_RATE, mode->refreshRate);
	    glfw_window_ =  glfwCreateWindow(settings.width, settings.height,
        settings.title.c_str(), NULL, NULL);
    }
    if (glfw_window_ == NULL) {
      std::stringstream ss;
      ss << "ERROR: glfwOpenWindow returned an error: ";
      ss << std::endl;
      throw std::runtime_error(ss.str());
    }

    glfwMakeContextCurrent(glfw_window_);

    if (settings.fullscreen) {
      glfwSetInputMode(glfw_window_, GLFW_CURSOR, GLFW_CURSOR_NORMAL );
    }

    // Callbacks must be set after the window is created
    glfwSetWindowSizeCallback(glfw_window_, NULL);
    glfwSetKeyCallback(glfw_window_, &keyboardInputCB);
    glfwSetCharCallback(glfw_window_, NULL);
    glfwSetCursorPosCallback(glfw_window_, &mousePosCB);
    glfwSetMouseButtonCallback(glfw_window_, &mouseButtonCB);
    glfwSetScrollCallback(glfw_window_, &mouseWheelCB);
    glfwSetWindowCloseCallback(glfw_window_, &closeCB);

    // Double check to make sure we were successful in creating a new OpenGL
    // context of the requested version
    if (settings.gl_major_version >= 3) {
      int major_window;
      int minor_window;
      int rev_window;
      major_window = glfwGetWindowAttrib(glfw_window_, GLFW_CONTEXT_VERSION_MAJOR);
      minor_window = glfwGetWindowAttrib(glfw_window_, GLFW_CONTEXT_VERSION_MINOR);
      rev_window = glfwGetWindowAttrib(glfw_window_, GLFW_CONTEXT_REVISION);
      if ( major_window != settings.gl_major_version ||
           minor_window != settings.gl_minor_version ) {
	std::cout << "ERROR: OpenGL context isnt the requested Vers." << std::endl;
	std::cout << "major vers: " << major_window << ", minor vers: " << minor_window << std::endl;
        throw std::runtime_error("ERROR: OpenGL context isnt the requested Vers.");
      }
    }


    // Now we need to initialize all the GL3 function pointers now that a valid
    // context has been built.
    // Note: there is a bug in GLEW 1.7, and to get around it we need to set
    // glewExperimental: http://www.opengl.org/wiki/OpenGL_Loading_Library
    if (init_glew_) {  // Only init glew once!
#ifdef _WIN32
      glewExperimental = GL_TRUE;
#else
      glewExperimental = true;
#endif
      GLenum ret = glewInit();
      if (ret != GLEW_OK) {
        std::stringstream ss;
        ss << "ERROR: glewInit() returned an error" << glewGetErrorString(ret);
        throw std::runtime_error(ss.str());
      }
      // Unfortuantely, as mentioned above GLEW causes some spurious gl errors on
      // startup, we should flush those so that error checking later is valid.
      while (glGetError() != GL_NO_ERROR) { }

      init_glew_ = false;
    }

    setDoubleBuffering(settings.double_buffering);

		// To fix a bug on Mac OS X with high DPI displays...  The actual 
    // resolution Might not match what we asked for.  So we must always 
    // reference this width and height from now on.
    glfwGetFramebufferSize(glfw_window_, &settings_.viewport_width, 
      &settings_.viewport_height);

    num_windows_++;
  }

  void Window::errorCallback(int error, const char* description) {
    throw std::runtime_error("GLFW Error: " + std::string(description));
  }

  void Window::setDoubleBuffering(const bool double_buffer) {
    settings_.double_buffering = double_buffer;
    if (double_buffer) {
      glfwSwapInterval(1);
    } else {
      glfwSwapInterval(0);
    }
  }

  void Window::swapBackBuffer() const {
    // swap back and front buffers
    glfwSwapBuffers(glfw_window_);
    glfwPollEvents();
  }

  const bool Window::getKeyState(const int key) const {
    return key_down_[key];
  }

  // Return true if the mouse is within the window
  const bool Window::getMousePosition(math::Double2& pos) const {
    pos.set(mouse_pos_);
    return pos.m[0] >= 0 && pos.m[0] < settings_.width &&
           pos.m[1] >= 0 && pos.m[1] < settings_.height;
  }

  const bool Window::getMouseButtonStateRight() const {
    return mouse_button_down_[static_cast<int>(GLFW_MOUSE_BUTTON_RIGHT)];
  }  
  
  const bool Window::getMouseButtonStateLeft() const {
    return mouse_button_down_[static_cast<int>(GLFW_MOUSE_BUTTON_LEFT)];
  }    
  
  const bool Window::getMouseButtonStateMiddle() const {
    return mouse_button_down_[static_cast<int>(GLFW_MOUSE_BUTTON_MIDDLE)];
  }   

  void Window::initWindowSystem() {
    std::lock_guard<std::mutex> lk(window_init_lock_);
    if (init_window_system_) {
      memset(key_down_, 0, NUM_KEYS*sizeof(key_down_[0]));
      memset(mouse_button_down_, 0, 
             NUM_MOUSE_BUTTONS*sizeof(mouse_button_down_[0]));

      if (!glfwInit()) {
        throw std::runtime_error("ERROR: glfwInit() returned an error");
      }
      init_window_system_ = false;

      glfwSetErrorCallback(errorCallback);
    }
  }

  void Window::keyboardInputCB(GLFWwindow* wnd, int key, int scancode, 
      int action, int mods) {
#if defined(DEBUG) || defined(_DEBUG)
    std::cout << "Window::keyboardInputCB(), key " << (char)key << ", action ";
    std::cout << action << std::endl;
    if (g_window_ && g_window_->glfw_window_ != wnd) {
      throw std::runtime_error("Window::keyboardInputCB() - ERROR: global "
        "window does not match GLFW cb pointer!");
    }
#endif
    // API Change: GLFW 3.0 defines a REPEAT action, which skews up existing
    // logic, for now ignore repeat events
    if (action != PRESSED_REPEAT) {
      key_down_[key] = (action == PRESSED) ? true : false;
      if (g_window_ && g_window_->keyboard_cb_) {
        g_window_->keyboard_cb_(key, scancode, action, mods);
      }
    }
  }

  void Window::mousePosCB(GLFWwindow* wnd, double x, double y) {
#if defined(DEBUG) || defined(_DEBUG)
    if (g_window_ && g_window_->glfw_window_ != wnd) {
      throw std::runtime_error("Window::keyboardInputCB() - ERROR: global "
        "window does not match GLFW cb pointer!");
    }
#endif
    mouse_pos_[0] = x;
    mouse_pos_[1] = y;
    
    if (g_window_ && g_window_->mouse_pos_cb_) {
      g_window_->mouse_pos_cb_(x, y);
    }
  }

  void Window::mouseButtonCB(GLFWwindow* wnd, int button, int action,
      int mods) {
#if defined(DEBUG) || defined(_DEBUG)
    std::cout << "Window::mouseButtonCB(), key " << button << ", action ";
    std::cout << action << std::endl;
    if (g_window_ && g_window_->glfw_window_ != wnd) {
      throw std::runtime_error("Window::keyboardInputCB() - ERROR: global "
        "window does not match GLFW cb pointer!");
    }
#endif
    mouse_button_down_[button] = (action == PRESSED) ? true : false;
    if (g_window_ && g_window_->mouse_button_cb_) {
      g_window_->mouse_button_cb_(button, action, mods);
    }
  }

  void Window::mouseWheelCB(GLFWwindow* wnd, double xoffset, double yoffset) {
#if defined(DEBUG) || defined(_DEBUG)
    if (g_window_ && g_window_->glfw_window_ != wnd) {
      throw std::runtime_error("Window::keyboardInputCB() - ERROR: global "
        "window does not match GLFW cb pointer!");
    }
#endif
    mouse_wheel_pos_[0] += xoffset;  // NOTE: It might overflow, but this is
    mouse_wheel_pos_[1] += yoffset;  //       very unlikely!
    if (g_window_ && g_window_->mouse_wheel_cb_) {
      g_window_->mouse_wheel_cb_(xoffset, yoffset);
    }
  }

  void Window::closeCB(GLFWwindow* wnd) {
#if defined(DEBUG) || defined(_DEBUG)
    if (g_window_ && g_window_->glfw_window_ != wnd) {
      throw std::runtime_error("Window::keyboardInputCB() - ERROR: global "
        "window does not match GLFW cb pointer!");
    }
#endif
    if (g_window_ && g_window_->close_cb_) {
      g_window_->close_cb_();
    }
  }

  void Window::killWindowSystem() {
    glfwTerminate();
  }

  const int Window::width() const { 
    return settings_.width; 
  }

  const int Window::height() const { 
    return settings_.height; 
  }

  const int Window::viewport_width() const { 
    return settings_.viewport_width; 
  }

  const int Window::viewport_height() const { 
    return settings_.viewport_height; 
  }

  const bool Window::fullscreen() const { 
    return settings_.fullscreen; 
  }

  const bool Window::isOpen() const {
    return !glfwWindowShouldClose(glfw_window_);
  }

  const bool Window::getDoubleBuffering() const { 
    return settings_.double_buffering; 
  }

  void Window::windowResEnumToInt(const int res, int& w, int& h) {
    // NOTE: This should only be called ONCE on startup (or on screen resize) 
    // to get the desired w and h from the settings enum.  You should call
    // width() and height() methods to get the actual resolution as it may
    // differ.
    w = (res >> 16) & 0x0000ffff;
    h = (res & 0x0000ffff);
  }
  
  void Window::registerKeyboardCB(KeyboardCBFuncPtr callback) {
    keyboard_cb_ = callback;
  }
  
  void Window::registerMousePosCB(MousePosCBFuncPtr callback) {
    mouse_pos_cb_ = callback;
  }
  
  void Window::registerMouseButCB(MouseButCBFuncPtr callback) {
    mouse_button_cb_ = callback;
  }
  
  void Window::registerMouseWheelCB(MouseWheelCBFuncPtr callback) {
    mouse_wheel_cb_ = callback;
  }

  void Window::registerCloseWndCB(CloseWndCBFuncPtr callback) {
    close_cb_ = callback;
  }

}  // namespace windowing
}  // namespace jtil
