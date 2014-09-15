#ifdef _WIN32
#include <Windows.h>
#include <strsafe.h>
#endif

#if defined(__APPLE__) || defined(__GNUC__)
#include <vector>
#include <cstring>
#endif

#include <string>  // for string
#include "string_util/string_util.h"

namespace jtil {
namespace string_util {

#ifdef _WIN32
  std::string Win32ErrorToString(const DWORD dwErrorCode) {
    // Retrieve the system error message for the last-error code

    LPVOID lpMsgBuf;
    LPVOID lpDisplayBuf;
    DWORD dw = GetLastError(); 

    FormatMessage(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | 
        FORMAT_MESSAGE_FROM_SYSTEM |
        FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL,
        dw,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPTSTR) &lpMsgBuf,
        0, NULL);

    // Display the error message and return in
    lpDisplayBuf = (LPVOID)LocalAlloc(LMEM_ZEROINIT, 
        (lstrlen((LPCTSTR)lpMsgBuf) + 
        lstrlen((LPCTSTR)dwErrorCode) + 40) * sizeof(TCHAR)); 
    StringCchPrintf((LPTSTR)lpDisplayBuf, 
        LocalSize(lpDisplayBuf) / sizeof(TCHAR),
        TEXT("%s failed with error %d: %s"), 
        dwErrorCode, dw, lpMsgBuf); 
    // MessageBox(NULL, (LPCTSTR)lpDisplayBuf, TEXT("Error"), MB_OK); 

    std::string strRet = ToNarrowString(((LPTSTR)lpDisplayBuf));

    LocalFree(lpMsgBuf);
    LocalFree(lpDisplayBuf);
    return strRet;
  }
#endif

  std::wstring ToWideString(const char* pStr , const int len = -1) {
#ifdef _WIN32
    // figure out how many wide characters we are going to get 
    int nChars = MultiByteToWideChar(CP_ACP , 0 , pStr , len , NULL , 0); 
    if (len == -1) {
      --nChars; 
    }
    if (nChars == 0) {
      return L"";
    }

    // convert the narrow string to a wide string 
    // nb: slightly naughty to write directly into the string like this
    std::wstring buf;
    buf.resize(nChars); 
    MultiByteToWideChar(CP_ACP , 0 , pStr , len , 
      const_cast<wchar_t*>(buf.c_str()) , nChars); 

    return buf;
#endif
#if defined(__APPLE__) || defined(__GNUC__)
    static_cast<void>(len);
    const size_t wn = std::mbsrtowcs(NULL, &pStr, 0, NULL);
    if (wn == size_t(-1)) {
      throw std::runtime_error(L"Error in mbsrtowcs()");
    }
    
    std::vector<wchar_t> buf(wn + 1);
    const size_t wn_again = std::mbsrtowcs(buf.data(), &pStr, wn + 1, NULL);
    
    if (wn_again == size_t(-1)) {
      throw std::runtime_error(L"Error in mbsrtowcs()");
    }
    
    return std::wstring(buf.data(), wn);
#endif
  }

  std::wstring ToWideString(const std::string& str) {
    return ToWideString(str.c_str(), -1);
  }

  std::string ToNarrowString(const wchar_t* pStr , const int len = -1) {
#ifdef _WIN32    
    // figure out how many narrow characters we are going to get 
    int nChars = WideCharToMultiByte(CP_ACP , 0 , pStr , len , NULL , 0 , 
      NULL , NULL); 
    if (len == -1) {
      --nChars; 
    }
    if (nChars == 0) {
      return "";
    }

    // convert the wide string to a narrow string
    // nb: slightly naughty to write directly into the string like this
    std::string buf;
    buf.resize(nChars);
    WideCharToMultiByte(CP_ACP , 0 , pStr , len , 
      const_cast<char*>(buf.c_str()) , nChars , NULL , NULL); 

    return buf; 
#endif
#if defined(__APPLE__) || defined(__GNUC__)
    static_cast<void>(len);
    const size_t wn = std::wcsrtombs(NULL, &pStr, 0, NULL);
    if (wn == size_t(-1)) {
      throw std::runtime_error(L"Error in wcsrtombs()");
    }
    
    std::vector<char> buf(wn + 1);
    const size_t wn_again = std::wcsrtombs(buf.data(), &pStr, wn + 1, NULL);
    
    if (wn_again == size_t(-1)) {
      throw std::runtime_error(L"Error in wcsrtombs()");
    }
    
    return std::string(buf.data(), wn);
#endif
  }

  std::string ToNarrowString(const std::wstring& str) {
    return ToNarrowString(str.c_str(), -1);
  }

  std::string getJTilDirEnvVar() {
    char* p_path;
    p_path = getenv("JTIL_DIR");
    if (p_path != NULL) {
      char end_char = p_path[strlen(p_path)-1];
      if (end_char == '\\' || end_char == '/') {
        return std::string(p_path);
      } else {
        return (std::string(p_path) + std::string("\\"));
      }
    } else {
      return std::string("");
    }
  }

  char* cStrCpy(const char* str) {
    int len = (int)strlen(str);
    char* ret_val = new char[len + 1];  // +1 is space for null terminator
    strncpy(ret_val, str, len + 1);
    return ret_val;
  }
  char* cStrCpy(const std::string& str) {
    return cStrCpy(str.c_str());
  }

}  // namespace string_util
}  // namespace jtil
