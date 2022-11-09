#ifndef POLYPLOYINTERSEC_H
#define POLYPLOYINTERSEC_H

#include "clipper.hpp"
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using Eigen::Matrix;
using std::vector;

namespace block{
class PolyPolyBoolean
{
public:
    const double error_large_ = 1E-4;
    const double error_small_ = 1E-6;

    typedef vector<Eigen::Vector3d> PolyVector3;
    typedef vector<vector<Eigen::Vector3d>> PolysVector3;

    typedef vector<Eigen::Vector2d> PolyVector2;
    typedef vector<vector<Eigen::Vector2d>> PolysVector2;

public:
    PolyPolyBoolean(){

    }

public:
    void computePolygonsUnion(PolysVector3 &polys, PolysVector3 &polysUnion);

    void computePolygonsIntersection(const PolyVector3 &polyA, const PolyVector3 &polyB, PolyVector3 &polyIntsec);

    void computePolygonsIntersection(const PolyVector3 &polyA, const PolyVector3 &polyB, PolysVector3 &polyIntsec);

    bool check2DPolygonsIntersection(const PolyVector2 &polyA, const PolyVector2 &polyB, double &area);

public:

    void printPolygon(const PolyVector3& poly);

    void cleanPath(PolyVector3 &polyIntsec);

    void computeFrame(const PolyVector3 &poly, Eigen::Vector3d &xaxis, Eigen::Vector3d &yaxis, Eigen::Vector3d &origin);

    double computeScale(const PolysVector3 &poly, Eigen::Vector3d &xaxis, Eigen::Vector3d &yaxis, Eigen::Vector3d &origin);

    double computeScale(const PolysVector2 &poly);

    Eigen::Vector3d computeNormal(const PolyVector3 &poly);

    Eigen::Vector3d computeCenter(const PolyVector3 &poly);

    ClipperLib::Path projectToNormalPlane(const PolyVector3 &poly, Eigen::Vector3d xaxis, Eigen::Vector3d yaxis, Eigen::Vector3d origin, double Scale);

    PolyVector3 projectTo3D(const ClipperLib::Path &path, Eigen::Vector3d xaxis, Eigen::Vector3d yaxis, Eigen::Vector3d origin, double scale);
};
}



#endif
