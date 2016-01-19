#include <pcpred/util/chi_square.h>

#include <map>
#include <cmath>


static bool chi_square_3d_initialized_ = false;
static std::map<double, double> chi_square_3d_table_;
static std::map<double, double> gaussian_distribution_3d_radius_table_;


static void initializeChiSquareTable();


double pcpred::chiSquare3D(double probability)
{
    if (chi_square_3d_initialized_ == false)
        initializeChiSquareTable();

    return chi_square_3d_table_[probability];
}

double pcpred::gaussianDistributionRadius3D(double probability)
{
    if (chi_square_3d_initialized_ == false)
        initializeChiSquareTable();

    return gaussian_distribution_3d_radius_table_[probability];
}


static void initializeChiSquareTable()
{
    chi_square_3d_initialized_ = true;

    chi_square_3d_table_[0.95 ] =  0.35;
    chi_square_3d_table_[0.90 ] =  0.58;
    chi_square_3d_table_[0.80 ] =  1.01;
    chi_square_3d_table_[0.70 ] =  1.42;
    chi_square_3d_table_[0.50 ] =  2.37;
    chi_square_3d_table_[0.30 ] =  3.66;
    chi_square_3d_table_[0.20 ] =  4.64;
    chi_square_3d_table_[0.10 ] =  6.25;
    chi_square_3d_table_[0.05 ] =  7.82;
    chi_square_3d_table_[0.01 ] = 11.34;
    chi_square_3d_table_[0.001] = 16.27;

    for (std::map<double, double>::iterator it = chi_square_3d_table_.begin(); it != chi_square_3d_table_.end(); it++)
        gaussian_distribution_3d_radius_table_[1.0 - it->first] = std::sqrt(it->second);
}
