//
// Created by Ziqi Wang on 02.03.2024.
//

#ifndef COLLISION_H
#define COLLISION_H
#include <Eigen/Dense>
#include <ccd/ccd.h>

namespace robot {
    struct _ccd_convex {
        Eigen::MatrixXd V;
    };


    inline void ccdSupportConvex(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v)
    {
        const _ccd_convex *c = (const _ccd_convex *)obj;
        ccd_vec3_t dir, p;
        ccd_real_t maxdot, dot;
        size_t i;

        ccdVec3Copy(&dir, _dir);
        maxdot = -CCD_REAL_MAX;
        for (int vi = 0; vi < c->V.rows(); vi++)
        {
            ccdVec3Set(&p, c->V(vi, 0), c->V(vi, 1), c->V(vi, 2));
            dot = ccdVec3Dot(&dir, &p);
            if (dot > maxdot){
                ccdVec3Copy(v, &p);
                maxdot = dot;
            }
        }
    }

    inline bool checkCollisionWithGround(const Eigen::MatrixXd &V) {
        ccd_t ccd;
        CCD_INIT(&ccd); // initialize ccd_t struct

        ccd.support1 = ccdSupportConvex;
        ccd.support2 = ccdSupportConvex;
        ccd.max_iterations = 100;     // maximal number of iterations

        _ccd_convex *obj1 = new _ccd_convex();
        _ccd_convex *obj2 = new _ccd_convex();
        obj1->V = V;
        obj2->V = Eigen::MatrixXd(8, 3);
        double bnd = 100;
        obj2->V << -bnd, -bnd, 0,
        bnd, -bnd, 0,
        bnd, bnd, 0,
        -bnd, bnd, 0,
        -bnd, -bnd, -bnd,
        bnd, -bnd, -bnd,
        bnd, bnd, -bnd,
        -bnd, bnd, -bnd;

        int intersec = ccdGJKIntersect(obj1, obj2, &ccd);

        delete obj1;
        delete obj2;

        return intersec;
    }

    inline bool checkCollision(const Eigen::MatrixXd &V1, const Eigen::MatrixXd &V2)
    {
        ccd_t ccd;
        CCD_INIT(&ccd); // initialize ccd_t struct

        // set up ccd_t struct
        ccd.support1       = ccdSupportConvex; // support function for first object
        ccd.support2       = ccdSupportConvex; // support function for second object
        ccd.max_iterations = 100;     // maximal number of iterations

        _ccd_convex *obj1 = new _ccd_convex();
        _ccd_convex *obj2 = new _ccd_convex();
        obj1->V = V1;
        obj2->V = V2;

        int intersec = ccdGJKIntersect(obj1, obj2, &ccd);

        delete obj1;
        delete obj2;

        return intersec;
    }
}
#endif //COLLISION_H
