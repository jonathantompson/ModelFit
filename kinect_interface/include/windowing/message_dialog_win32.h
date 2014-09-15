//
//  message_dialog_win32.h
//
//  Created by Jonathan Tompson on 1/3/13.
//
//  Simple interface to create a blocking dialog box (the static function
//  blocks until the dialog box is closed).  Should be thread safe.
// 

#pragma once

#include <mutex>
#include <string>
#include "data_str/vector_managed.h"

namespace jtil {
namespace windowing {

  class MessageDialogWin32 {
  public:
    // openTaskDialog --> BLOCKING until window is closed.
    static void openTaskDialog(const std::wstring& title, 
      const std::wstring& txt, const uint32_t width, const uint32_t height);

  private:
    // Constructor / Destructor
    MessageDialogWin32();
    ~MessageDialogWin32();

    static std::mutex handle_mutex_;
    static uint64_t handle_cnt_;
    static std::wstring cur_text_;
    static const wchar_t g_szClassName_[];
    static data_str::VectorManaged<TCHAR*> lines_;
    static bool class_registered_;

    static LRESULT CALLBACK MessageDialogWin32::WndProc(HWND hwnd, UINT msg, 
      WPARAM wParam, LPARAM lParam);
    static void MessageDialogWin32::collectLines(const std::wstring& txt, 
      data_str::VectorManaged<TCHAR*>& lines);

    // Non-copyable, non-assignable.
    MessageDialogWin32(MessageDialogWin32&);
    MessageDialogWin32& operator=(const MessageDialogWin32&);
  };
};  // namespace windowing
};  // namspace jtil
