#pragma once
#include <precomp.h>
#include <math.h>
#include <vector>
// makros for coloring console output

#define GREENTEXT(output) "\x1b[32;1m" << output << "\x1b[0m"
#define REDTEXT(output) "\x1b[31;1m" << output << "\x1b[0m" 
#define BLUETEXT(output) "\x1b[34;1m" << output << "\x1b[0m" 
#define YELLOWTEXT(output) "\x1b[33;1m" << output << "\x1b[0m" 

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
ZYK_EXPORTS void test_func();

/**
* @brief Splits a string using the given delimiter character.
*
* @param s String to split.
* @param delim Delimiter used to split the string into chunks. The delimiter will be removed.
* @return std::vector< std::string, std::allocator< void > > List of string chunks.
*/
ZYK_EXPORTS std::vector<std::string> split(const std::string& s, char delim);

/**
* @brief Splits a string at spaces.
*
* @param s String to split.
* @return std::vector< std::string, std::allocator< void > > List of string chunks.
*/
ZYK_EXPORTS std::vector<std::string> split(const std::string& s);