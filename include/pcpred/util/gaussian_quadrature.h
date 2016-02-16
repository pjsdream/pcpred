#ifndef GAUSSIAN_QUADRATURE_H
#define GAUSSIAN_QUADRATURE_H


#include <vector>


namespace pcpred
{

void getGaussianQuadratureCoefficients3(std::vector<double>& x, std::vector<double>& w);
double getGaussianQuadratureCoefficient3X(int i);
double getGaussianQuadratureCoefficient3W(int i);

}


#endif // GAUSSIAN_QUADRATURE_H
