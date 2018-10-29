#pragma once
#include <precomp.h>
#include <math.h>
namespace zyk{
template<typename T>
T dot(const T* n1, const T* n2, const int dim)
{
  T res = 0;
  for (int i = 0; i < dim; ++i)
    res += n1[i] * n2[i];
  return res;
}
template<typename T>
T norm(const T* n, const int dim)
{
  T res = 0;
  for (int i = 0; i < dim; ++i)
    res += n[i] * n[i];
  return sqrt(res);
}
template<typename T>
T dist(const T* n1, const T* n2, const int dim)
{
  T res = 0;
  for (int i = 0; i < dim; ++i)
  {
    T tmp = n1[i]- n2[i];
    res += tmp*tmp;
  }
  return sqrt(res);
}
}

#ifndef WIN32
void _splitpath(const char *path, char *drive, char *dir, char *fname, char *ext);
static void _split_whole_name(const char *whole_name, char *fname, char *ext);
#endif
