#include <pcpred/util/gaussian_quadrature.h>


namespace pcpred
{

static const double gaussian_quadrature_coefficient_3[4][2] =
{
    {-0.339981, 0.652145},
    { 0.339981, 0.652145},
    {-0.861136, 0.347855},
    { 0.861136, 0.347855},
};

void getGaussianQuadratureCoefficients3(std::vector<double>& x, std::vector<double>& w)
{
    x.resize(4);
    w.resize(4);

    for (int i=0; i<4; i++)
    {
        x[i] = gaussian_quadrature_coefficient_3[i][0];
        w[i] = gaussian_quadrature_coefficient_3[i][1];
    }
}

double getGaussianQuadratureCoefficient3X(int i)
{
    return gaussian_quadrature_coefficient_3[i][0];
}

double getGaussianQuadratureCoefficient3W(int i)
{
    return gaussian_quadrature_coefficient_3[i][1];
}

}
