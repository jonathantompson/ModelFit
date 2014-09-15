#if defined(WIN32) || defined(_WIN32)

#include <windows.h>
#include <sstream>
#include <strsafe.h>
#include "windowing/message_dialog_win32.h"
#define IDC_MAIN_EDIT	101

using std::wstring;

namespace jtil {
namespace windowing {

  std::mutex MessageDialogWin32::handle_mutex_;
  std::wstring MessageDialogWin32::cur_text_;
  const wchar_t MessageDialogWin32::g_szClassName_[] = L"MessageDialogWin32";
  data_str::VectorManaged<TCHAR*> MessageDialogWin32::lines_;
  bool MessageDialogWin32::class_registered_ = false;

  MessageDialogWin32::MessageDialogWin32() {
    // empty constructor
  }

  MessageDialogWin32::~MessageDialogWin32() {
    // empty constructor
  }

  // Step 4: the Window Procedure
  LRESULT CALLBACK MessageDialogWin32::WndProc(HWND hwnd, UINT msg, 
    WPARAM wParam, LPARAM lParam) {
    PAINTSTRUCT ps;
    HDC hdc;
    TEXTMETRIC tm; 
    SCROLLINFO si; 
    HFONT hFont,hTmp;

    // These variables are required to display text. 
    static int xClient;     // width of client area 
    static int yClient;     // height of client area 
    static int xClientMax;  // maximum width of client area 

    static int xChar;       // horizontal scrolling unit 
    static int yChar;       // vertical scrolling unit 
    static int xUpper;      // average width of uppercase letters 

    static int xPos;        // current horizontal scrolling position 
    static int yPos;        // current vertical scrolling position 

    int i;                  // loop counter 
    int x, y;               // horizontal and vertical coordinates

    int FirstLine;          // first line in the invalidated area 
    int LastLine;           // last line in the invalidated area 

    bool new_font = false;

    switch(msg) {
    case WM_CREATE : 
      // Get the handle to the client area's device context. 
      hdc = GetDC (hwnd); 

      // Extract font dimensions from the text metrics. 
      GetTextMetrics (hdc, &tm); 
      xChar = tm.tmAveCharWidth; 
      xUpper = (tm.tmPitchAndFamily & 1 ? 3 : 2) * xChar/2; 
      yChar = tm.tmHeight + tm.tmExternalLeading; 

      // Free the device context. 
      ReleaseDC (hwnd, hdc); 

      // Set an arbitrary maximum width for client area. 
      // (xClientMax is the sum of the widths of 48 average 
      // lowercase letters and 12 uppercase letters.) 
      xClientMax = 48 * xChar + 12 * xUpper; 

      return 0; 

    case WM_SIZE: 

      // Retrieve the dimensions of the client area. 
      yClient = HIWORD (lParam); 
      xClient = LOWORD (lParam); 

      // Set the vertical scrolling range and page size
      si.cbSize = sizeof(si); 
      si.fMask  = SIF_RANGE | SIF_PAGE; 
      si.nMin   = 0; 
      si.nMax   = lines_.size() - 1; 
      si.nPage  = yClient / yChar; 
      SetScrollInfo(hwnd, SB_VERT, &si, TRUE); 

      // Set the horizontal scrolling range and page size. 
      si.cbSize = sizeof(si); 
      si.fMask  = SIF_RANGE | SIF_PAGE; 
      si.nMin   = 0; 
      si.nMax   = 2 + xClientMax / xChar; 
      si.nPage  = xClient / xChar; 
      SetScrollInfo(hwnd, SB_HORZ, &si, TRUE); 

      return 0; 
    case WM_HSCROLL:
      // Get all the vertial scroll bar information.
      si.cbSize = sizeof (si);
      si.fMask  = SIF_ALL;

      // Save the position for comparison later on.
      GetScrollInfo (hwnd, SB_HORZ, &si);
      xPos = si.nPos;
      switch (LOWORD (wParam))
      {
        // User clicked the left arrow.
      case SB_LINELEFT: 
        si.nPos -= 1;
        break;

        // User clicked the right arrow.
      case SB_LINERIGHT: 
        si.nPos += 1;
        break;

        // User clicked the scroll bar shaft left of the scroll box.
      case SB_PAGELEFT:
        si.nPos -= si.nPage;
        break;

        // User clicked the scroll bar shaft right of the scroll box.
      case SB_PAGERIGHT:
        si.nPos += si.nPage;
        break;

        // User dragged the scroll box.
      case SB_THUMBTRACK: 
        si.nPos = si.nTrackPos;
        break;

      default :
        break;
      }

      // Set the position and then retrieve it.  Due to adjustments
      // by Windows it may not be the same as the value set.
      si.fMask = SIF_POS;
      SetScrollInfo (hwnd, SB_HORZ, &si, TRUE);
      GetScrollInfo (hwnd, SB_HORZ, &si);

      // If the position has changed, scroll the window.
      if (si.nPos != xPos)
      {
        ScrollWindow(hwnd, xChar * (xPos - si.nPos), 0, NULL, NULL);
      }

      return 0;

    case WM_VSCROLL:
      // Get all the vertial scroll bar information.
      si.cbSize = sizeof (si);
      si.fMask  = SIF_ALL;
      GetScrollInfo (hwnd, SB_VERT, &si);

      // Save the position for comparison later on.
      yPos = si.nPos;
      switch (LOWORD (wParam))
      {

        // User clicked the HOME keyboard key.
      case SB_TOP:
        si.nPos = si.nMin;
        break;

        // User clicked the END keyboard key.
      case SB_BOTTOM:
        si.nPos = si.nMax;
        break;

        // User clicked the top arrow.
      case SB_LINEUP:
        si.nPos -= 1;
        break;

        // User clicked the bottom arrow.
      case SB_LINEDOWN:
        si.nPos += 1;
        break;

        // User clicked the scroll bar shaft above the scroll box.
      case SB_PAGEUP:
        si.nPos -= si.nPage;
        break;

        // User clicked the scroll bar shaft below the scroll box.
      case SB_PAGEDOWN:
        si.nPos += si.nPage;
        break;

        // User dragged the scroll box.
      case SB_THUMBTRACK:
        si.nPos = si.nTrackPos;
        break;

      default:
        break; 
      }

      // Set the position and then retrieve it.  Due to adjustments
      // by Windows it may not be the same as the value set.
      si.fMask = SIF_POS;
      SetScrollInfo (hwnd, SB_VERT, &si, TRUE);
      GetScrollInfo (hwnd, SB_VERT, &si);

      // If the position has changed, scroll window and update it.
      if (si.nPos != yPos)
      {                    
        ScrollWindow(hwnd, 0, yChar * (yPos - si.nPos), NULL, NULL);
        UpdateWindow (hwnd);
      }

      return 0;

    case WM_PAINT :
      // Prepare the window for painting.
      hdc = BeginPaint (hwnd, &ps);

      // Get vertical scroll bar position.
      si.cbSize = sizeof (si);
      si.fMask  = SIF_POS;
      GetScrollInfo (hwnd, SB_VERT, &si);
      yPos = si.nPos;

      // Get horizontal scroll bar position.
      GetScrollInfo (hwnd, SB_HORZ, &si);
      xPos = si.nPos;

      // Find painting limits.
      FirstLine = max (0, yPos + ps.rcPaint.top / yChar);
      LastLine = min ((int)lines_.size() - 1, 
        yPos + ps.rcPaint.bottom / yChar);

      // hFont=CreateFont(0,0,0,0,FW_NORMAL,0,0,0,0,0,0,2,0,L"MS Sans Serif");
      hFont=CreateFont(-12, 0, 0, 0, FW_NORMAL,	FALSE, FALSE, FALSE, 
        ANSI_CHARSET, OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS, PROOF_QUALITY, 
        FF_DONTCARE | DEFAULT_PITCH, L"Consolas");
      new_font = false;
      hTmp = 0;
      if (hFont != NULL) {
        hTmp = (HFONT)SelectObject(hdc, hFont);
        new_font = true;
      }

      for (i = FirstLine; i <= LastLine; i++)
      {
        x = xChar * (1 - xPos);
        y = yChar * (i - yPos);


        // Write a line of text to the client area.
        TextOut(hdc, x, y, lines_[i], (int)wcslen(lines_[i])); 
      }

      if (new_font) {
        DeleteObject(SelectObject(hdc, hTmp));
      }

      // Indicate that painting is finished.
      EndPaint (hwnd, &ps);
      return 0;

    case WM_DESTROY :
      PostQuitMessage (0);
      return 0;

    default:
      return DefWindowProc(hwnd, msg, wParam, lParam);
    }
  }

  void MessageDialogWin32::collectLines(const std::wstring& txt, 
    data_str::VectorManaged<TCHAR*>& lines) {
    const wchar_t* c_str = txt.c_str();
    lines.clear();

    std::wstringstream ss;
    int cur_line = 1;
    int cur_char = 0;
    int str_length = static_cast<int>(wcslen(c_str));
    do {
      if (c_str[cur_char] == '\n') {
        cur_line++;
        // Save the line;
        wstring line = ss.str();
        wchar_t* line_cstr = new wchar_t[line.length()+1];
        wcsncpy(line_cstr, line.c_str(), line.length()+1);
        lines.pushBack(line_cstr);
        // Start a new line
        ss.str(std::wstring(L""));
      } else {
        ss << c_str[cur_char];
      }
      cur_char++;
    } while (cur_char < str_length);
    // Save the last line;
    wstring line = ss.str();
    wchar_t* line_cstr = new wchar_t[line.length()+1];
    wcsncpy(line_cstr, line.c_str(), line.length()+1);
    lines.pushBack(line_cstr);
  }

  void MessageDialogWin32::openTaskDialog(const std::wstring& title,
    const std::wstring& txt, const uint32_t width, const uint32_t height) {
    handle_mutex_.lock();

    // Create a vector strings, each containing the lines of text
    cur_text_ = txt;
    collectLines(txt, lines_);

    if (!class_registered_) {
      WNDCLASSEX wc;
      //Step 1: Registering the Window Class
      wc.cbSize        = sizeof(WNDCLASSEX);
      wc.style         = 0;
      wc.lpfnWndProc   = WndProc;
      wc.cbClsExtra    = 0;
      wc.cbWndExtra    = 0;
      wc.hInstance     = GetModuleHandle(NULL);
      wc.hIcon         = LoadIcon(NULL, IDI_APPLICATION);
      wc.hCursor       = LoadCursor(NULL, IDC_ARROW);
      wc.hbrBackground = (HBRUSH)(COLOR_WINDOW+1);
      wc.lpszMenuName  = NULL;
      wc.lpszClassName = g_szClassName_;
      wc.hIconSm       = LoadIcon(NULL, IDI_APPLICATION);

      if (!RegisterClassEx(&wc)) {
        MessageBox(NULL, L"Window Registration Failed!", L"Error!",
          MB_ICONEXCLAMATION | MB_OK);
        handle_mutex_.unlock();
        return;
      }
      class_registered_ = true;
    }

    HWND hwnd = CreateWindowEx(WS_EX_CLIENTEDGE, g_szClassName_, title.c_str(),
      WS_OVERLAPPEDWINDOW | WS_VISIBLE | WS_VSCROLL | WS_HSCROLL, 
      CW_USEDEFAULT, CW_USEDEFAULT, width, height, NULL, NULL, 
      GetModuleHandle(NULL), NULL);

    if(hwnd == NULL) {
      MessageBox(hwnd, L"Could not create edit box.", L"Error", 
        MB_OK | MB_ICONERROR);
      handle_mutex_.unlock();
      return;
    }

    ShowWindow(hwnd, SW_SHOWNORMAL);
    UpdateWindow(hwnd);

    // Step 3: The Message Loop
    MSG Msg;
    while(GetMessage(&Msg, NULL, 0, 0) > 0)
    {
      TranslateMessage(&Msg);
      DispatchMessage(&Msg);
    }

    handle_mutex_.unlock();
  }
}  // namespace misc
}  // namespace jtil

#endif
