#pragma once
#include <precomp.h>
namespace zyk{
	ZYK_EXPORTS double dot(const float* n1, const float* n2, const int dim);
	ZYK_EXPORTS double norm(const float* n, const int dim);
	ZYK_EXPORTS double dist(const float* n1, const float* n2, const int dim);
}