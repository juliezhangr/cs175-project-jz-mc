#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <vector>
#include <cmath>

#include "cvec.h"

// A generic vertex structure containing position, normal, and texture information
// Used by make* functions to pass vertex information to the caller
struct GenericVertex {
  Cvec3f pos;
  Cvec3f normal;
  Cvec2f tex;
  Cvec3f tangent, binormal;

GenericVertex(
              float x, float y, float z,
              float nx, float ny, float nz,
              float tu, float tv,
              float tx, float ty, float tz,
              float bx, float by, float bz)
: pos(x,y,z), normal(nx,ny,nz), tex(tu, tv), tangent(tx, ty, tz), binormal(bx, by, bz)
  {}
};


inline void getCubeVbIbLen(int& vbLen, int& ibLen) {
  vbLen = 24;
  ibLen = 36;
}

template<typename VtxOutIter, typename IdxOutIter>
  void makeCube(float size, VtxOutIter vtxIter, IdxOutIter idxIter) {
  float h = size / 2.0;
#define DEFV(x, y, z, nx, ny, nz, tu, tv) {             \
    *vtxIter = GenericVertex(x h, y h, z h,             \
                             nx, ny, nz, tu, tv,        \
                             tan[0], tan[1], tan[2],    \
                             bin[0], bin[1], bin[2]);   \
    ++vtxIter;                                          \
  }
  Cvec3f tan(0, 1, 0), bin(0, 0, -1);
  DEFV(+, -, -, 1, 0, 0, 0, 0); // facing +X
  DEFV(+, +, -, 1, 0, 0, 1, 0);
  DEFV(+, +, +, 1, 0, 0, 1, 1);
  DEFV(+, -, +, 1, 0, 0, 0, 1);

  tan = Cvec3f(0, -1, 0);
  bin = Cvec3f(0, 0, -1);
  DEFV(-, -, -, -1, 0, 0, 0, 0); // facing -X
  DEFV(-, -, +, -1, 0, 0, 1, 0);
  DEFV(-, +, +, -1, 0, 0, 1, 1);
  DEFV(-, +, -, -1, 0, 0, 0, 1);

  tan = Cvec3f(-1, 0, 0);
  bin = Cvec3f(0, 0, -1);
  DEFV(-, +, -, 0, 1, 0, 0, 0); // facing +Y
  DEFV(-, +, +, 0, 1, 0, 1, 0);
  DEFV(+, +, +, 0, 1, 0, 1, 1);
  DEFV(+, +, -, 0, 1, 0, 0, 1);

  tan = Cvec3f(1, 0, 0);
  bin = Cvec3f(0, 0, -1);
  DEFV(-, -, -, 0, -1, 0, 0, 0); // facing -Y
  DEFV(+, -, -, 0, -1, 0, 1, 0);
  DEFV(+, -, +, 0, -1, 0, 1, 1);
  DEFV(-, -, +, 0, -1, 0, 0, 1);

  tan = Cvec3f(0, 1, 0);
  bin = Cvec3f(1, 0, 0);
  DEFV(-, -, +, 0, 0, 1, 0, 0); // facing +Z
  DEFV(+, -, +, 0, 0, 1, 1, 0);
  DEFV(+, +, +, 0, 0, 1, 1, 1);
  DEFV(-, +, +, 0, 0, 1, 0, 1);

  tan = Cvec3f(0, 1, 0);
  bin = Cvec3f(-1, 0, 0);
  DEFV(-, -, -, 0, 0, -1, 0, 0); // facing -Z
  DEFV(-, +, -, 0, 0, -1, 1, 0);
  DEFV(+, +, -, 0, 0, -1, 1, 1);
  DEFV(+, -, -, 0, 0, -1, 0, 1);
#undef DEFV

  for (int v = 0; v < 24; v +=4) {
    *idxIter = v;
    *++idxIter = v + 1;
    *++idxIter = v + 2;
    *++idxIter = v;
    *++idxIter = v + 2;
    *++idxIter = v + 3;
    ++idxIter;
  }
}

inline void getSphereVbIbLen(int slices, int stacks, int& vbLen, int& ibLen) {
  assert(slices > 1);
  assert(stacks >= 2);
  vbLen = 2 + slices * (stacks - 1);
  ibLen = slices * (stacks - 1) * 6;
}

template<typename VtxOutIter, typename IdxOutIter>
  void makeSphere(float radius, int slices, int stacks, VtxOutIter vtxIter, IdxOutIter idxIter) {
  using namespace std;
  assert(slices > 1);
  assert(stacks >= 2);

  // the two poles
  *vtxIter = GenericVertex(0, 0, radius,
                           0, 0, 1,
                           0, 1,
                           1, 0, 0,
                           0, -1, 0);
  *++vtxIter = GenericVertex(0, 0, -radius,
                             0, 0, -1,
                             0, 0,
                             1, 0, 0,
                             0, 1, 0);
  ++vtxIter;

  const double radPerSlice = 2 * CS175_PI / slices;
  const double radPerStack = CS175_PI / stacks;

  vector<double> longSin(slices), longCos(slices);
  vector<double> latSin(stacks-1), latCos(stacks-1);
  for (int i = 0; i < slices; ++i) {
    longSin[i] = sin(radPerSlice * i);
    longCos[i] = cos(radPerSlice * i);
  }
  for (int i = 0; i < stacks - 1; ++i) {
    latSin[i] = sin(radPerStack * (i+1));
    latCos[i] = cos(radPerStack * (i+1));
  }

  const Cvec3f up(0, 0, 1);

  for (int i = 0; i < slices; ++i) {
    *idxIter = 0;
    *++idxIter = 2 + (stacks-1) * i ;
    *++idxIter = 2 + (stacks-1) * ((i + 1) % slices);
    ++idxIter;

    for (int j = 0; j < stacks - 1; ++j) {
      float x = longCos[i] * latSin[j];
      float y = longSin[i] * latSin[j];
      float z = latCos[j];

      Cvec3f n(x, y, z);
      Cvec3f t = normalize(cross(up, n));
      Cvec3f b = cross(t, n);

      *vtxIter = GenericVertex(
                               x * radius, y * radius, z * radius,
                               x, y, z,
                               1.0/slices*i, 1.0/stacks*(j+1),
                               t[0], t[1], t[2], b[0], b[1], b[2]);
      ++vtxIter;

      if (j < stacks - 2) {
        *idxIter = 2 + (stacks-1) * i + j;
        *++idxIter = 2 + (stacks-1) * i + j + 1;
        *++idxIter = 2 + (stacks-1) * ((i + 1) % slices) + j + 1;

        *++idxIter = 2 + (stacks-1) * i + j;
        *++idxIter = 2 + (stacks-1) * ((i + 1) % slices) + j + 1;
        *++idxIter = 2 + (stacks-1) * ((i + 1) % slices) + j;
        ++idxIter;
      }
    }

    *idxIter = 2 + (stacks-1) * i  + stacks - 2;
    *++idxIter = 1;
    *++idxIter = 2 + (stacks-1) * ((i + 1)%slices)  + stacks - 2;
    ++idxIter;
  }
}

#endif
