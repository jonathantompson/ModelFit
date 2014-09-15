//#include "threading/callback_instances.h"
//  callback.h
//
//  Created by Jonathan Tompson on 7/10/12.
//
//  This is a templated class which allows us to wrap function pointers in a
//  clean and type safe manner.  It was particularly useful for pthreads since
//  pthread method inputs must be wrapped in a void* struct.  It is still useful
//  for C++11 since thread function inputs are required to be static.  Using
//  these callback wrappers allows us to call a thread on a class's non-static
//  member function.

#pragma once

#include "threading/callback_instances.h"

// If you need more template arguments, you can generate a new 
// callback_instances.h file by typing:
// >> python callback_instances.py N
// with N being the maximum number of arguments you want.  By default I have 8.
