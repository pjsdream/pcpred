#include <pcpred/feature/hermite_curve.h>
#include <pcpred/util/gaussian_quadrature.h>

#include <iostream>

using namespace pcpred;


HermiteCurve::HermiteCurve(int num_pieces)
{
    setCurveShape(num_pieces);
}

void HermiteCurve::setCurveShape(int num_pieces)
{
    num_pieces_ = num_pieces;

    control_points_.resize(3, (num_pieces + 1) * 2);
    control_points_.setZero();
}

void HermiteCurve::setControlPoint(int i, const Eigen::Vector3d& x, const Eigen::Vector3d& v)
{
    control_points_.col(2*i  ) = x;
    control_points_.col(2*i+1) = v;
}

void HermiteCurve::makeStationaryPoint(const Eigen::Vector3d& x)
{
    for (int i=0; i<=num_pieces_; i++)
    {
        control_points_.col(2*i  ) = x;
        control_points_.col(2*i+1) = Eigen::Vector3d::Zero();
    }
}

int HermiteCurve::featureSize()
{
    return num_pieces_ * 12;
}

Eigen::VectorXd HermiteCurve::toFeature()
{
    Eigen::VectorXd x;
    x.resize( num_pieces_ * 12 );

    std::vector<double> gx;
    std::vector<double> gw;
    getGaussianQuadratureCoefficients3(gx, gw);

    for (int i=0; i<num_pieces_; i++)
    {
        x.block(12*i    , 0, 3, 1) = gw[0] * (*this)(i + 0.5 + 0.5 * gx[0]);
        x.block(12*i + 3, 0, 3, 1) = gw[1] * (*this)(i + 0.5 + 0.5 * gx[1]);
        x.block(12*i + 6, 0, 3, 1) = gw[2] * (*this)(i + 0.5 + 0.5 * gx[2]);
        x.block(12*i + 9, 0, 3, 1) = gw[3] * (*this)(i + 0.5 + 0.5 * gx[3]);
    }

    return x;
}

int HermiteCurve::encodingSize()
{
    return (num_pieces_ + 1) * 2 * 3;
}

Eigen::VectorXd HermiteCurve::encode()
{
    Eigen::VectorXd y( (num_pieces_ + 1) * 2 * 3 );

    for (int i=0; i<=num_pieces_; i++)
    {
        y.block(6*i  , 0, 3, 1) = control_points_.col(2*i  );
        y.block(6*i+3, 0, 3, 1) = control_points_.col(2*i+1);
    }

    return y;
}

void HermiteCurve::decode(const Eigen::VectorXd& code)
{
    for (int i=0; i<code.rows(); i+=3)
        control_points_.col(i) = code.block(i, 0, 3, 1);
}

Eigen::Vector3d HermiteCurve::operator () (double t) const
{
    int i = (int)t;
    if (i == num_pieces_) i--;

    const double s = t-i;
    const double s2 = s * s;
    const double s3 = s2 * s;

    /* Hermite spline basis
     * p0 : 2t^3 - 3t^2 + 1
     * v0 : t^3 - 2t^2 + t
     * p1 : -2t^3 + 3t^2
     * v1 : t^3 - t^2
     */
    return control_points_.col(2*i) * (2*s3 - 3*s2 + 1)
            + control_points_.col(2*i+1) * (s3 - 2*s2 + s)
            + control_points_.col(2*i+2) * (-2*s3 + 3*s2)
            + control_points_.col(2*i+3) * (s3 - s2);
}

HermiteCurve HermiteCurve::operator + (const HermiteCurve& rhs) const
{
    HermiteCurve curve(num_pieces_);
    curve.control_points_ = control_points_ + rhs.control_points_;
    return curve;
}

HermiteCurve HermiteCurve::operator - (const HermiteCurve& rhs) const
{
    HermiteCurve curve(num_pieces_);
    curve.control_points_ = control_points_ - rhs.control_points_;
    return curve;
}

HermiteCurve HermiteCurve::operator * (double rhs) const
{
    HermiteCurve curve(num_pieces_);
    curve.control_points_ = control_points_ * rhs;
    return curve;
}

HermiteCurve& HermiteCurve::operator += (const HermiteCurve& rhs)
{
    control_points_ += rhs.control_points_;
    return *this;
}

HermiteCurve& HermiteCurve::operator -= (const HermiteCurve& rhs)
{
    control_points_ -= rhs.control_points_;
    return *this;
}

HermiteCurve& HermiteCurve::operator *= (double rhs)
{
    control_points_ *= rhs;
    return *this;
}
