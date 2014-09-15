/*
*  perlin_noise.h
*
*  Created by Kristofer Schlachter on 8/29/09.
*  Edited by Jonathan Tompson 1/16/2013
*  Copyright 2009 Gotham Wave Games, Inc. All rights reserved.
*
*/

#pragma once

#include <cmath>

namespace jtil {
namespace math {

  static int noise_permutation[512] = {151,160,137,91,90,15,131,13,201,95,96,
    53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,190,6,148,247,
    120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,88,237,149,
    56,87,174,20,125,136,171,168,68,175,74,165,71,134,139,48,27,166,77,146,
    158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,102,
    143,54,65,25,63,161,1,216,80,73,209,76,132,187,208,89,18,169,200,196,
    135,130,116,188,159,86,164,100,109,198,173,186,3,64,52,217,226,250,124,
    123,5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,
    28,42,223,183,170,213,119,248,152,2,44,154,163,70,221,153,101,155,167,43,
    172,9,129,22,39,253,19,98,108,110,79,113,224,232,178,185,112,104,218,246,
    97,228,251,34,242,193,238,210,144,12,191,179,162,241,81,51,145,235,249,14,
    239,107,49,192,214,31,181,199,106,157,184,84,204,176,115,121,50,45,127,4,
    150,254,138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,
    156,180,151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,140,36,103,
    30,69,142,8,99,37,240,21,10,23,190,6,148,247,120,234,75,0,26,197,62,94,
    252,219,203,117,35,11,32,57,177,33,88,237,149,56,87,174,20,125,136,171,168,
    68,175,74,165,71,134,139,48,27,166,77,146,158,231,83,111,229,122,60,211,
    133,230,220,105,92,41,55,46,245,40,244,102,143,54,65,25,63,161,1,216,80,
    73,209,76,132,187,208,89,18,169,200,196,135,130,116,188,159,86,164,100,
    109,198,173,186,3,64,52,217,226,250,124,123,5,202,38,147,118,126,255,82,
    85,212,207,206,59,227,47,16,58,17,182,189,28,42,223,183,170,213,119,248,
    152,2,44,154,163,70,221,153,101,155,167,43,172,9,129,22,39,253,19,98,108,
    110,79,113,224,232,178,185,112,104,218,246,97,228,251,34,242,193,238,210,
    144,12,191,179,162,241,81,51,145,235,249,14,239,107,49,192,214,31,181,199,
    106,157,184,84,204,176,115,121,50,45,127,4,150,254,138,236,205,93,222,114,
    67,29,24,72,243,141,128,195,78,66,215,61,156,180
  };

  class PerlinNoise
  {
  public:

    // Returns noise between 0 - 1
    static float NoiseNormalized(float x, float y)
    {
      //-0.697 - 0.795 + 0.697
      float value = Noise(x, y);

      /* Replacing division with multipling by the inverse as a decial
      0.793+0.69 	= 1.483
      1/1.483		= 0.67430883345
      */
      //value = (value + 0.69F) / (0.793F + 0.69F);
      value = (value + 0.69F) *0.67430883345F;
      return value;
    };

    static float Noise(float x, float y)
    {
      int X = (int)floor(x) & 255,                  // FIND UNIT CUBE THAT
        Y = (int)floor(y) & 255;                  // CONTAINS POINT.
      x -= floor(x);                                // FIND RELATIVE X,Y,Z
      y -= floor(y);                                // OF POINT IN CUBE.
      float u = fade(x),                                // COMPUTE FADE CURVES
        v = fade(y);                                // FOR EACH OF X,Y,Z.
      int A = noise_permutation[X  ]+Y, AA = noise_permutation[A], AB = noise_permutation[A+1];  // HASH COORDINATES OF
      int B = noise_permutation[X+1]+Y, BA = noise_permutation[B], BB = noise_permutation[B+1];  // THE 8 CUBE CORNERS,

      float res = lerp(v, lerp(u, grad2(noise_permutation[AA  ], x  , y   ),  // AND ADD
        grad2(noise_permutation[BA  ], x-1, y )), // BLENDED
        lerp(u, grad2(noise_permutation[AB  ], x  , y-1 ),  // RESULTS
        grad2(noise_permutation[BB  ], x-1, y-1 )));// FROM  8
      return res;
    };

    static float Noise(float x, float y, float z) {
      int X = (int)floor(x) & 255,                  // FIND UNIT CUBE THAT
        Y = (int)floor(y) & 255,                  // CONTAINS POINT.
        Z = (int)floor(z) & 255;
      x -= floor(x);                                // FIND RELATIVE X,Y,Z
      y -= floor(y);                                // OF POINT IN CUBE.
      z -= floor(z);
      float u = fade(x),                                // COMPUTE FADE CURVES
        v = fade(y),                                // FOR EACH OF X,Y,Z.
        w = fade(z);
      int A = noise_permutation[X  ]+Y, AA = noise_permutation[A]+Z, AB = 
        noise_permutation[A+1]+Z,      // HASH COORDINATES OF
        B = noise_permutation[X+1]+Y, BA = noise_permutation[B]+Z, BB = 
        noise_permutation[B+1]+Z;      // THE 8 CUBE CORNERS,

      return lerp(w, lerp(v, lerp(u, grad(noise_permutation[AA  ], x, y, z),  // AND ADD
        grad(noise_permutation[BA  ], x-1, y  , z   )), // BLENDED
        lerp(u, grad(noise_permutation[AB  ], x  , y-1, z   ),  // RESULTS
        grad(noise_permutation[BB  ], x-1, y-1, z   ))),// FROM  8
        lerp(v, lerp(u, grad(noise_permutation[AA+1], x  , y  , z-1 ),  // CORNERS
        grad(noise_permutation[BA+1], x-1, y  , z-1 )), // OF CUBE
        lerp(u, grad(noise_permutation[AB+1], x  , y-1, z-1 ),
        grad(noise_permutation[BB+1], x-1, y-1, z-1 ))));
    };

    static float fade(float t) { return t * t * t * (t * (t * 6 - 15) + 10); };
    static float lerp(float t, float a, float b) { return a + t * (b - a); };
    static float grad(int hash, float x, float y, float z) {
      int h = hash & 15;                      // CONVERT LO 4 BITS OF HASH CODE
      float u = h<8 ? x : y,                 // INTO 12 gradIENT DIRECTIONS.
        v = h<4 ? y : h==12||h==14 ? x : z;
      return ((h&1) == 0 ? u : -u) + ((h&2) == 0 ? v : -v);
    };

    static float grad2(int hash, float x, float y) {
      int h = hash & 15;                      // CONVERT LO 4 BITS OF HASH CODE
      float u = h < 8 ? x : y,                 // INTO 12 gradIENT DIRECTIONS.
        v = h < 4 ? y : h==12 || h==14 ? x : 0;
      return ((h&1) == 0 ? u : -u) + ((h&2) == 0 ? v : -v);
    };

  };

};  // namespace math
};  // namespace jtil
