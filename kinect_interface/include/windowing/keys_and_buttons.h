//
//  keys.h
//
//  Created by Jonathan Tompson on 6/13/12.
//

// Just grab the GLFW input related definitions
#include "windowing/glfw_keys.h"

#pragma once

#define PRESSED GLFW_PRESS
#define PRESSED_REPEAT GLFW_REPEAT
#define RELEASED GLFW_RELEASE

// Define some key values (so app.h doesn't need GLFW specific bindings)
// Note that the character keys are all UPPER_CASE char values, ie the key for
// 'a' or 'A' is static_cast<int>('A')
#define KEY_UNKNOWN      GLFW_KEY_UNKNOWN
#define KEY_APOSTROPHE   GLFW_KEY_APOSTROPHE
#define KEY_COMMA        GLFW_KEY_COMMA
#define KEY_MINUS        GLFW_KEY_MINUS
#define KEY_ESC          GLFW_KEY_ESCAPE
#define KEY_SPACE        GLFW_KEY_SPACE
#define KEY_F1           GLFW_KEY_F1
#define KEY_F2           GLFW_KEY_F2
#define KEY_F3           GLFW_KEY_F3
#define KEY_F4           GLFW_KEY_F4
#define KEY_F5           GLFW_KEY_F5
#define KEY_F6           GLFW_KEY_F6
#define KEY_F7           GLFW_KEY_F7
#define KEY_F8           GLFW_KEY_F8
#define KEY_F9           GLFW_KEY_F9
#define KEY_F10          GLFW_KEY_F10
#define KEY_F11          GLFW_KEY_F11
#define KEY_F12          GLFW_KEY_F12
#define KEY_F13          GLFW_KEY_F13
#define KEY_F14          GLFW_KEY_F14
#define KEY_F15          GLFW_KEY_F15
#define KEY_F16          GLFW_KEY_F16
#define KEY_F17          GLFW_KEY_F17
#define KEY_F18          GLFW_KEY_F18
#define KEY_F19          GLFW_KEY_F19
#define KEY_F20          GLFW_KEY_F20
#define KEY_F21          GLFW_KEY_F21
#define KEY_F22          GLFW_KEY_F22
#define KEY_F23          GLFW_KEY_F23
#define KEY_F24          GLFW_KEY_F24
#define KEY_F25          GLFW_KEY_F25
#define KEY_UP           GLFW_KEY_UP
#define KEY_DOWN         GLFW_KEY_DOWN
#define KEY_LEFT         GLFW_KEY_LEFT
#define KEY_RIGHT        GLFW_KEY_RIGHT
#define KEY_LSHIFT       GLFW_KEY_LEFT_SHIFT
#define KEY_RSHIFT       GLFW_KEY_RIGHT_SHIFT
#define KEY_LCTRL        GLFW_KEY_LEFT_CONTROL
#define KEY_RCTRL        GLFW_KEY_RIGHT_CONTROL
#define KEY_LALT         GLFW_KEY_LEFT_ALT
#define KEY_RALT         GLFW_KEY_RIGHT_ALT
#define KEY_TAB          GLFW_KEY_TAB
#define KEY_ENTER        GLFW_KEY_ENTER
#define KEY_BACKSPACE    GLFW_KEY_BACKSPACE
#define KEY_INSERT       GLFW_KEY_INSERT
#define KEY_DEL          GLFW_KEY_DELETE
#define KEY_PAGEUP       GLFW_KEY_PAGE_UP
#define KEY_PAGEDOWN     GLFW_KEY_PAGE_DOWN
#define KEY_HOME         GLFW_KEY_HOME
#define KEY_END          GLFW_KEY_END
#define KEY_KP_0         GLFW_KEY_KP_0
#define KEY_KP_1         GLFW_KEY_KP_1
#define KEY_KP_2         GLFW_KEY_KP_2
#define KEY_KP_3         GLFW_KEY_KP_3
#define KEY_KP_4         GLFW_KEY_KP_4
#define KEY_KP_5         GLFW_KEY_KP_5
#define KEY_KP_6         GLFW_KEY_KP_6
#define KEY_KP_7         GLFW_KEY_KP_7
#define KEY_KP_8         GLFW_KEY_KP_8
#define KEY_KP_9         GLFW_KEY_KP_9
#define KEY_KP_DIVIDE    GLFW_KEY_KP_DIVIDE
#define KEY_KP_MULTIPLY  GLFW_KEY_KP_MULTIPLY
#define KEY_KP_SUBTRACT  GLFW_KEY_KP_SUBTRACT
#define KEY_KP_ADD       GLFW_KEY_KP_ADD
#define KEY_KP_DECIMAL   GLFW_KEY_KP_DECIMAL
#define KEY_KP_EQUAL     GLFW_KEY_KP_EQUAL
#define KEY_KP_ENTER     GLFW_KEY_KP_ENTER
#define KEY_KP_NUM_LOCK  GLFW_KEY_NUM_LOCK
#define KEY_CAPS_LOCK    GLFW_KEY_CAPS_LOCK
#define KEY_SCROLL_LOCK  GLFW_KEY_SCROLL_LOCK
#define KEY_PAUSE        GLFW_KEY_PAUSE
#define KEY_LSUPER       GLFW_KEY_LEFT_SUPER
#define KEY_RSUPER       GLFW_KEY_RIGHT_SUPER
#define KEY_MENU         GLFW_KEY_MENU
#define NUM_KEYS         GLFW_KEY_LAST+1

#define MOUSE_BUTTON_1      GLFW_MOUSE_BUTTON_1
#define MOUSE_BUTTON_2      GLFW_MOUSE_BUTTON_2
#define MOUSE_BUTTON_3      GLFW_MOUSE_BUTTON_3
#define MOUSE_BUTTON_4      GLFW_MOUSE_BUTTON_4
#define MOUSE_BUTTON_5      GLFW_MOUSE_BUTTON_5
#define MOUSE_BUTTON_6      GLFW_MOUSE_BUTTON_6
#define MOUSE_BUTTON_7      GLFW_MOUSE_BUTTON_7
#define MOUSE_BUTTON_8      GLFW_MOUSE_BUTTON_8
#define NUM_MOUSE_BUTTONS   GLFW_MOUSE_BUTTON_LAST+1

/* Mouse button aliases */
#define MOUSE_BUTTON_LEFT   GLFW_MOUSE_BUTTON_LEFT
#define MOUSE_BUTTON_RIGHT  GLFW_MOUSE_BUTTON_RIGHT
#define MOUSE_BUTTON_MIDDLE GLFW_MOUSE_BUTTON_MIDDLE
