//
// Created by 汪子琦 on 04.09.22.
//

#include "rigid_block/util/PolyPolyBoolean.h"

namespace rigid_block
{
void PolyPolyBoolean::computePolygonsUnion(PolyPolyBoolean::PolysVector3 &polys,
                                           PolyPolyBoolean::PolysVector3 &polysUnion) {
    polysUnion.clear();
    if(polys.empty())return;

    Eigen::Vector3d x_axis, y_axis, origin;
    computeFrame(polys[0], x_axis, y_axis, origin);

    double Scale = computeScale(polys, x_axis, y_axis, origin);

    vector<ClipperLib::Path> clipperPaths;

    for(size_t id = 0; id < polys.size(); id++)
    {
        ClipperLib::Path path;
        path = projectToNormalPlane(polys[id], x_axis, y_axis, origin, Scale);

        ClipperLib::ClipperOffset offsetter;
        offsetter.AddPath(path, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
        ClipperLib::Paths off_sol;
        offsetter.Execute(off_sol, 10.0);
        if(!off_sol.empty()) clipperPaths.push_back(off_sol[0]);
    }

    ClipperLib::Clipper solver;
    solver.AddPaths(clipperPaths, ClipperLib::ptSubject, true);
    ClipperLib::Paths path_union;
    solver.Execute(ClipperLib::ctUnion, path_union, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

    if (path_union.empty())
        return;

    for(ClipperLib::Path path : path_union)
    {
        PolyVector3 polyUnion = projectTo3D(path, x_axis, y_axis, origin, Scale);
        cleanPath(polyUnion);
        polysUnion.push_back(polyUnion);
    }

    return;
}

void PolyPolyBoolean::computePolygonsIntersection(const PolyPolyBoolean::PolyVector3 &polyA,
                                                  const PolyPolyBoolean::PolyVector3 &polyB,
                                                  PolyPolyBoolean::PolyVector3 &polyIntsec) {
    PolysVector3 polylists;
    computePolygonsIntersection(polyA, polyB, polylists);
    if(!polylists.empty()){
        polyIntsec = polylists[0];
    }

    return;
}

void PolyPolyBoolean::printPolygon(const PolyPolyBoolean::PolyVector3 &poly) {
    std::cout << "{";
    for (size_t id = 0; id < poly.size(); id++) {
        std::cout << "{" << poly[id].x() << ", " << poly[id].y() << "}, ";
    }
    std::cout << "}\n";
}

void PolyPolyBoolean::computePolygonsIntersection(const PolyPolyBoolean::PolyVector3 &polyA,
                                                  const PolyPolyBoolean::PolyVector3 &polyB,
                                                  PolyPolyBoolean::PolysVector3 &polyIntsec) {
    polyIntsec.clear();

    Eigen::Vector3d x_axis, y_axis, origin;
    computeFrame(polyA, x_axis, y_axis, origin);

    PolysVector3 polys;
    polys.push_back(polyA);
    polys.push_back(polyB);

    double Scale = computeScale(polys, x_axis, y_axis, origin);

    ClipperLib::Path pathA, pathB;
    pathA = projectToNormalPlane(polyA, x_axis, y_axis, origin, Scale);
    pathB = projectToNormalPlane(polyB, x_axis, y_axis, origin, Scale);

    ClipperLib::Clipper solver;
    solver.AddPath(pathA, ClipperLib::ptSubject, true);
    solver.AddPath(pathB, ClipperLib::ptClip, true);
    ClipperLib::Paths path_int;
    solver.StrictlySimple(true);
    solver.Execute(ClipperLib::ctIntersection, path_int, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);
    ClipperLib::ClipperOffset offset;
    offset.AddPaths(path_int, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
    offset.Execute(path_int, -10);
    ClipperLib::SimplifyPolygons(path_int);
    if (path_int.empty())
        return;

    for(size_t id = 0; id < path_int.size(); id++)
    {
        PolyVector3 polylist = projectTo3D(path_int[id], x_axis, y_axis, origin, Scale);
        cleanPath(polylist);
        polyIntsec.push_back(polylist);
    }

    return;
}

bool PolyPolyBoolean::check2DPolygonsIntersection(const PolyPolyBoolean::PolyVector2 &polyA,
                                                  const PolyPolyBoolean::PolyVector2 &polyB,
                                                  double &area){
    PolysVector2 polys;
    polys.push_back(polyA);
    polys.push_back(polyB);
    double Scale = computeScale(polys);

    ClipperLib::Path pathA, pathB;
    for(size_t id = 0; id < polyA.size(); id++){
        int x = polyA[id].x() * Scale;
        int y = polyA[id].y() * Scale;
        pathA.push_back(ClipperLib::IntPoint(x, y));
    }

    for(size_t id = 0; id < polyB.size(); id++){
        int x = polyB[id].x() * Scale;
        int y = polyB[id].y() * Scale;
        pathB.push_back(ClipperLib::IntPoint(x, y));
    }

    if(!ClipperLib::Orientation(pathA))
    {
        ClipperLib::ReversePath(pathA);
        //std::cout << "PathA is Wrong" << std::endl;
    }
    if(!ClipperLib::Orientation(pathB))
    {
        ClipperLib::ReversePath(pathB);
        //std::cout << "pathB is Wrong" << std::endl;
    }

    ClipperLib::Clipper solver;
    solver.AddPath(pathA, ClipperLib::ptSubject, true);
    solver.AddPath(pathB, ClipperLib::ptClip, true);
    ClipperLib::Paths path_int;
    solver.Execute(ClipperLib::ctIntersection, path_int, ClipperLib::pftPositive, ClipperLib::pftPositive);

    if(path_int.empty() || path_int.front().empty())
    {
        area = 0;
        return false;
    }
    else{
        area = 0;
        for(ClipperLib::Path path : path_int){
            area += ClipperLib::Area(path) / Scale / Scale;
        }
        return true;
    }
}

void PolyPolyBoolean::cleanPath(PolyPolyBoolean::PolyVector3 &polyIntsec)
{
    PolyVector3 polySimplest;
    bool doAgain = true;
    float big_zero_eps = error_large_;


    int N = polyIntsec.size();
    //remove duplicate points first
    for (size_t id = 0; id < polyIntsec.size(); id++) {

        Eigen::Vector3d ppt = polyIntsec[(id - 1 + N) % N];
        Eigen::Vector3d pt = polyIntsec[id];
        Eigen::Vector3d npt = polyIntsec[(id + 1) % N];
        Eigen::Vector3d tA = ppt - pt;
        Eigen::Vector3d tB = npt - pt;
        if (tA.norm() < big_zero_eps)
        {
            doAgain = true;
            continue;
        }
        else {
            polySimplest.push_back(pt);
        }
    }

    //remove points in a same line
    polyIntsec = polySimplest;
    polySimplest.clear();
    N = polyIntsec.size();
    for(size_t id = 0; id < polyIntsec.size(); id++){
        Eigen::Vector3d ppt = polyIntsec[(id - 1 + N) % N];
        Eigen::Vector3d pt = polyIntsec[id];
        Eigen::Vector3d npt = polyIntsec[(id + 1) % N];
        Eigen::Vector3d tA = ppt - pt;
        Eigen::Vector3d tB = npt - pt;
        double cross_product = (tA.cross(tB)).norm() / tA.norm() / tB.norm();
        if (cross_product > big_zero_eps) {
            polySimplest.push_back(pt);
        }
    }
    polyIntsec = polySimplest;
}

void PolyPolyBoolean::computeFrame(const PolyPolyBoolean::PolyVector3 &poly,
                                   Eigen::Vector3d &xaxis,
                                   Eigen::Vector3d &yaxis,
                                   Eigen::Vector3d &origin) {
    Eigen::Vector3d normal = computeNormal(poly);
    Eigen::Vector3d center = computeCenter(poly);

    xaxis = normal.cross(Eigen::Vector3d(1, 0, 0));
    if(xaxis.norm() < error_large_)
        xaxis = normal.cross(Eigen::Vector3d(0, 1, 0));
    xaxis.normalize();

    yaxis = normal.cross(xaxis);
    yaxis.normalize();

    origin = center;

    return;
}

Matrix<double, 3, 1> PolyPolyBoolean::computeNormal(const PolyPolyBoolean::PolyVector3 &poly)
{
    Eigen::Vector3d normal(0, 0 ,0), center(0, 0, 0);
    center = computeCenter(poly);

    for(int id = 0; id < (int)(poly.size()) - 1; id++){
        normal += (poly[id] - center).cross(poly[id + 1] - center);
    }

    if(normal.norm() < error_large_)
        return Eigen::Vector3d(0, 0, 0);

    return normal.normalized();
}

Matrix<double, 3, 1> PolyPolyBoolean::computeCenter(const PolyPolyBoolean::PolyVector3 &poly)
{
    Eigen::Vector3d center(0, 0, 0);

    for(size_t id = 0; id < poly.size(); id++)
    {
        center += poly[id];
    }
    if(!poly.empty())
        center /= poly.size();
    else
        center = Eigen::Vector3d(0, 0, 0);

    return center;
}

ClipperLib::Path
PolyPolyBoolean::projectToNormalPlane(const PolyPolyBoolean::PolyVector3 &poly,
                                      Eigen::Vector3d xaxis,
                                      Eigen::Vector3d yaxis,
                                      Eigen::Vector3d origin,
                                      double Scale) {
    ClipperLib::Path path;
    for(size_t id = 0; id < poly.size(); id++)
    {
        Eigen::Vector3d pos = poly[id];
        int x = (int)((pos - origin).dot(xaxis) * Scale);
        int y = (int)((pos - origin).dot(yaxis) * Scale);
        path.push_back(ClipperLib::IntPoint(x, y));
    }

    return path;
}

vector<Matrix<double, 3, 1>>
PolyPolyBoolean::projectTo3D(const ClipperLib::Path &path,
                             Eigen::Vector3d xaxis,
                             Eigen::Vector3d yaxis,
                             Eigen::Vector3d origin,
                             double Scale) {

    PolyVector3 poly;
    for(size_t id = 0; id < path.size(); id++)
    {
        float x = path[id].X / Scale;
        float y = path[id].Y / Scale;
        Eigen::Vector3d pos = xaxis * x + yaxis * y + origin;
        poly.push_back(pos);
    }

    return poly;
}

double PolyPolyBoolean::computeScale(const PolyPolyBoolean::PolysVector3 &polys,
                                     Eigen::Vector3d &xaxis,
                                     Eigen::Vector3d &yaxis,
                                     Eigen::Vector3d &origin){

    double Scale = 1;
    int maxdigit = 0;
    for(PolyVector3 poly:polys)
    {
        for(size_t id = 0; id < poly.size(); id++)
        {
            Eigen::Vector3d pos = poly[id];
            double x = std::abs((pos - origin).dot(xaxis));
            double y = std::abs((pos - origin).dot(yaxis));
            int digit = std::floor(std::max(std::log10(x) + 1, std::log10(y) + 1));
            maxdigit = std::max(digit, maxdigit);
        }
    }

    Scale = std::max(Scale, (double)(std::pow(10, std::max(0, 8 - maxdigit))));
    return Scale;
}

double PolyPolyBoolean::computeScale(const PolyPolyBoolean::PolysVector2 &polys) {

    double Scale = 1;
    int maxdigit = 0;

    for(PolyVector2 poly:polys)
    {
        for(size_t id = 0; id < poly.size(); id++)
        {
            Eigen::Vector2d pos = poly[id];
            double x = pos.x();
            double y = pos.y();
            int digit = std::floor(std::max(std::log10(x) + 1, std::log10(y) + 1));
            maxdigit = std::max(digit, maxdigit);
        }
    }

    Scale = std::max(Scale, std::pow(10, std::max(0, 8 - maxdigit)));
    return Scale;
}
}
