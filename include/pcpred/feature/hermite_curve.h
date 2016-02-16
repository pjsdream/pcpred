#ifndef HERMITE_CURVE_H
#define HERMITE_CURVE_H


#include <Eigen/Dense>


namespace pcpred
{

// piecewise cubic hermite curve
class HermiteCurve
{
public:

    HermiteCurve(int num_pieces = 1);

    inline int getNumPieces()
    {
        return num_pieces_;
    }

    void setCurveShape(int num_pieces);
    void setControlPoint(int i, const Eigen::Vector3d& x, const Eigen::Vector3d& v);
    void makeStationaryPoint(const Eigen::Vector3d& x);

    Eigen::VectorXd toFeature();
    int featureSize();

    Eigen::Vector3d operator () (double t) const;

    HermiteCurve operator + (const HermiteCurve& rhs) const;
    HermiteCurve operator - (const HermiteCurve& rhs) const;
    HermiteCurve operator * (double rhs) const;

    HermiteCurve& operator += (const HermiteCurve& rhs);
    HermiteCurve& operator -= (const HermiteCurve& rhs);
    HermiteCurve& operator *= (double rhs);

private:

    int num_pieces_;
    Eigen::Matrix3Xd control_points_;
};

}


#endif // HERMITE_CURVE_H
