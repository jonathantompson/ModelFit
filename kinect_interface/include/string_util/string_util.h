//
//  string_util.h
//
//  Created by Jonathan Tompson on 5/1/12.
//

#pragma once

#ifdef _WIN32
  #include <windows.h>
#endif
#include <string>
#include <sstream>

namespace jtil {
namespace string_util {
#ifdef _WIN32
  std::string Win32ErrorToString(const DWORD dwErrorCode);
#endif
  std::string getJTilDirEnvVar();

  // Convert a string to a number
  template <class T>
  T Str2Num(const std::string& s) {
    std::istringstream stream(s);
    T t;
    stream >> t;
    return t; 
  };

  // Convert a number to a string
  template <class T>
  std::string Num2Str(const T num) {
    std::stringstream stream;
    stream << num;
    return stream.str();
  };

  std::wstring ToWideString(const char* pStr, const int len);
  std::wstring ToWideString(const std::string& str);
  std::string ToNarrowString(const wchar_t* pStr, const int len);
  std::string ToNarrowString(const std::wstring& str);

  char* cStrCpy(const char* str);  // Allocates new data
  char* cStrCpy(const std::string& str);  // Allocates new data
  
};  // namespace string_util
};  // namespace jtil

#ifdef _WIN32
#define ER_CHECK_WIN32(expr) \
  if ((expr) == 0) { \
    throw std::runtime_error(jtil::string_util::Win32ErrorToString(GetLastError()));\
  }
#endif
