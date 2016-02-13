#include <pcpred/util/gaussian_quadrature.h>


namespace pcpred
{

void getGaussianQuadratureCoefficients3(std::vector<double>& x, std::vector<double>& w)
{
    x.resize(4);
    w.resize(4);

    x[0] = -0.339981; w[0] = 0.652145;
    x[1] =  0.339981; w[1] = 0.652145;
    x[2] = -0.861136; w[2] = 0.347855;
    x[3] =  0.861136; w[3] = 0.347855;
}

}
