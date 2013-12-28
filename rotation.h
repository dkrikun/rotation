#ifndef ROTATION_H
#define ROTATION_H

#include <Eigen/Geometry>

class rotation;

rotation rotation_compose(const rotation& lhs, const rotation& rhs);
rotation rotation_substract(const rotation& lhs, const rotation& rhs);
rotation rotation_inverse(const rotation&);

void rotation_compose(rotation* self, const rotation& rhs);
void rotation_substract(rotation* self, const rotation& rhs);
void rotation_inverse(rotation* self);

bool rotation_equivalent(const rotation& lhs, const rotation& rhs);

void rotation_hpr(const rotation&, double* heading, double* pitch,
    double* roll);

Eigen::Vector3d rotation_hpr(const rotation&);

class rotation
{
    public:
    static rotation make_identity()
    { return rotation(0.,0.,0.); }

    static rotation from_vec(const Eigen::Vector3d& v)
    { return rotation(v[0], v[1], v[2]); }

    rotation(double heading, double pitch, double roll);

    rotation& operator+=(const rotation& rhs)
    {
        rotation_compose(this, rhs);
        return *this;
    }

    rotation& operator-=(const rotation& rhs)
    {
        rotation_substract(this, rhs);
        return *this;
    }

    rotation operator-() const { return rotation_inverse(*this); }

    Eigen::Vector3d hpr() const { return rotation_hpr(*this); }

    Eigen::Quaterniond quat_;
};

inline rotation operator+(const rotation& lhs, const rotation& rhs)
{
    return rotation_compose(lhs,rhs);
}

inline rotation operator-(const rotation& lhs, const rotation& rhs)
{
    return rotation_substract(lhs,rhs);
}

inline bool operator==(const rotation& lhs, const rotation& rhs)
{
    return rotation_equivalent(lhs, rhs);
}

inline bool operator!=(const rotation& lhs, const rotation& rhs)
{
    return !operator==(lhs,rhs);
}

inline std::ostream& operator<<(std::ostream& os, const rotation& r)
{
    os << r.quat_.x() << " "
        << r.quat_.y() << " "
        << r.quat_.z() << " "
        << r.quat_.w();
}


#endif
