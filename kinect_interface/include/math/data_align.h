//
//  data_align.h
//
//  Created by Jonathan Tompson on 3/14/12.
//  Copyright (c) 2012 NYU. All rights reserved.
//

#pragma once

#ifndef ALIGNMENT
#define ALIGNMENT 16
#endif

#if defined(_MSC_VER)
  #define DATA_ALIGN(alignment, dec) __declspec(align(alignment)) dec
#elif defined(GCC) || defined(__APPLE__)
  #define DATA_ALIGN(alignment, dec) dec __attribute__ ((aligned (alignment)))
#else
  #define DATA_ALIGN(alignment, dec)
#endif
