#include <util.h>
#include <math.h>
#include <string>
double zyk::dot(const float* n1, const float* n2, const int dim)
{
	double res = 0;
	for (int i = 0; i < dim; ++i)
		res += n1[i] * n2[i];
	return res;
}

double zyk::norm(const float* n, const int dim)
{
	double res = 0;
	for (int i = 0; i < dim; ++i)
		res += n[i] * n[i];
	return sqrt(res);
}

double zyk::dist(const float* n1, const float* n2, const int dim)
{
	double res = 0;
	for (int i = 0; i < dim; ++i)
	{
		double tmp = n1[i]- n2[i];
		res += tmp*tmp;
	}
	return sqrt(res);
}