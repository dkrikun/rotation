
#include "rotation.h"
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace Eigen;


rotation::rotation(double heading, double pitch, double roll)
{
    double hrad(heading*M_PI/180.);
    double prad(pitch*M_PI/180.);
    double rrad(roll*M_PI/180.);

    quat_ = AngleAxisd(hrad, Vector3d::UnitZ())
        * AngleAxisd(prad, Vector3d::UnitX())
        * AngleAxisd(rrad, Vector3d::UnitY());

    quat_.normalize();
}


rotation rotation_compose(const rotation& lhs, const rotation& rhs)
{
    rotation nrv(lhs);
    rotation_compose(&nrv, rhs);
    return nrv;
}

rotation rotation_substract(const rotation& lhs, const rotation& rhs)
{
    rotation nrv(lhs);
    rotation_substract(&nrv, rhs);
    return nrv;
}

rotation rotation_inverse(const rotation& r)
{
    rotation nrv(r);
    rotation_inverse(&nrv);
    return nrv;
}

void rotation_compose(rotation* self, const rotation& rhs)
{
    self->quat_ *= rhs.quat_;
}

void rotation_substract(rotation* self, const rotation& rhs)
{
    rotation_compose(self, rotation_inverse(rhs));
}

void rotation_inverse(rotation* self)
{
    self->quat_ = self->quat_.inverse().normalized();
}

bool rotation_equivalent(const rotation &lhs, const rotation& rhs)
{
    return lhs.quat_.isApprox(rhs.quat_)
        || lhs.quat_.coeffs().isApprox(-rhs.quat_.coeffs());
}

Vector3d rotation_hpr(const rotation& self)
{
    Vector3d hpr = self.quat_.normalized().toRotationMatrix()
                                .eulerAngles(2,0,1);
    return 180.*M_1_PI*hpr;
}

void rotation_hpr(const rotation& self, double* h, double* p, double* r)
{
    Vector3d hpr_deg = rotation_hpr(self);

    *h = hpr_deg[0];
    *p = hpr_deg[1];
    *r = hpr_deg[2];
}

