#include "rotation.h"

#include <iostream>
#include <assert.h>


int main(int argc, char** argv)
{
    double eps = 0.5;

    rotation zero_rotation = rotation::make_identity();

    rotation r1(30.,40.,50.), r2(90.,90.,90.),
        r3(-20.,45.,100.), r4(-180.,90.,125.),
        r5(-729., 800., 1900.);

    rotation rs[] = { r1, r2, r3, r4, r5 };

    for(int j = 0; j<4; ++j)
    {
        rotation& rj = rs[j];

        assert( rj + (-rj) == zero_rotation );
        assert( rj - rj - rj == -rj );
        assert( rj != -rj );
        assert( rj != zero_rotation );

        rotation rr = rotation::from_vec(rj.hpr());

        std::cout << "------\n"
            << rj.hpr() << " -- " << rr.hpr() << std::endl;


        assert( rr == rj );

        using namespace Eigen;

        Vector3d xyz(30.,20.,35.);

        Vector3d rr_xyz = rr.quat_._transformVector(xyz);
        Vector3d rj_xyz = rj.quat_._transformVector(xyz);

        std::cout << "*** " << rr_xyz << " &&& " << rj_xyz << " ^^^" << std::endl;

        assert( rr_xyz == rj_xyz );
    }



    return 0;
}
