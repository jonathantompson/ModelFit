#if defined(WIN32) || defined(_WIN32)

#include <Windows.h>
#include <WinUser.h>  // for MAKEINTRESOURCE 
#include <iostream>
#include "windowing/window.h"
#include "string_util/string_util.h"
#include "windowing/message_dialog_win32.h"

namespace jtil {
namespace windowing {

  void NativeErrorBox(const wchar_t* str) {
    // Note: this will likely throw an unhandeled exception if MessageBox
    // returns an error.  But an unhandled exception is better than nothing.
    int ret = MessageBox(NULL, (LPCTSTR)str, TEXT("Error"), 
      MB_OK | MB_ICONERROR);
    ER_CHECK_WIN32(ret);

    // MessageDialogWin32::openTaskDialog(L"Error", str, 400, 200);
  } 

}  // namespace windowing
}  // namespace jtil

#endif
