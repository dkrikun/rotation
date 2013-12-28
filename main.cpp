#include "rotation.h"

#include <iostream>
#include <assert.h>
#include <vector>


int main(int argc, char** argv)
{
    using namespace Eigen;
    double eps = 0.5;

    rotation zero_rotation = rotation::make_identity();

# if 1
    IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
    IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
    IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
# endif


    std::vector<double> degs;
    degs.push_back(-361.);
    degs.push_back(-360.2);
    degs.push_back(-360.1);
    degs.push_back(-360.001);
    degs.push_back(-360.);
    degs.push_back(-359.999);
    degs.push_back(-359.9);
    degs.push_back(-359.8);
    degs.push_back(-359.);
    degs.push_back(-271.);
    degs.push_back(-270.2);
    degs.push_back(-270.1);
    degs.push_back(-270.001);
    degs.push_back(-270.);
    degs.push_back(-269.999);
    degs.push_back(-269.9);
    degs.push_back(-269.8);
    degs.push_back(-181.);
    degs.push_back(-180.2);
    degs.push_back(-180.1);
    degs.push_back(-180.001);
    degs.push_back(-180.);
    degs.push_back(-179.999);
    degs.push_back(-179.9);
    degs.push_back(-179.8);
    degs.push_back(-178.);
    degs.push_back(-91.);
    degs.push_back(-90.2);
    degs.push_back(-90.1);
    degs.push_back(-90.001);
    degs.push_back(-90.);
    degs.push_back(-89.999);
    degs.push_back(-89.9);
    degs.push_back(-89.8);
    degs.push_back(-88.);

# if 0
    degs.push_back(-1.);
    degs.push_back(-0.2);
    degs.push_back(-0.1);
    degs.push_back(-0.001);
    degs.push_back(+0.);
    degs.push_back(+0.001);
    degs.push_back(+0.1);
    degs.push_back(+0.2);
    degs.push_back(+1.);

    degs.push_back(+361.);
    degs.push_back(+360.2);
    degs.push_back(+360.1);
    degs.push_back(+360.001);
    degs.push_back(+360.);
    degs.push_back(+359.999);
    degs.push_back(+359.9);
    degs.push_back(+359.8);
    degs.push_back(+359.);
    degs.push_back(+271.);
    degs.push_back(+270.2);
    degs.push_back(+270.1);
    degs.push_back(+270.001);
    degs.push_back(+270.);
    degs.push_back(+269.999);
    degs.push_back(+269.9);
    degs.push_back(+269.8);
    degs.push_back(+181.);
    degs.push_back(+180.2);
    degs.push_back(+180.1);
    degs.push_back(+180.001);
    degs.push_back(+180.);
    degs.push_back(+179.999);
    degs.push_back(+179.9);
    degs.push_back(+179.8);
    degs.push_back(+178.);
    degs.push_back(+91.);
    degs.push_back(+90.2);
    degs.push_back(+90.1);
    degs.push_back(+90.001);
    degs.push_back(+90.);
    degs.push_back(+89.999);
    degs.push_back(+89.9);
    degs.push_back(+89.8);
    degs.push_back(+89.);
# endif

    std::cout << "degs.size()=" << degs.size() << std::endl;

    for(int xj = 0; xj <= degs.size(); xj++)
    for(int yj = 0; yj <= degs.size(); yj++)
    for(int zj = 0; zj <= degs.size(); zj++)
    {
        rotation r(degs[xj], degs[yj], degs[zj]);

        assert( r + (-r) == zero_rotation );
        assert( r - r - r == -r );

        rotation rr = rotation::from_vec(r.hpr());
# if 0
        if(rr != r)
        {
            std::cout << "r=[" << r << "]" << std::endl;
            std::cout << "rr=[" << rr << "]" << std::endl;
            std::cout << "xyz=" << x << " " << y << " " << z << std::endl;
        }
# endif
        assert( rr == r );

        for(int vxj = 0; vxj <= degs.size(); vxj++)
        for(int vyj = 0; vyj <= degs.size(); vyj++)
        for(int vzj = 0; vzj <= degs.size(); vzj++)
        {
            Vector3d xyz(degs[vxj],degs[vyj],degs[vzj]);

            Vector3d rr_xyz = rr.quat_._transformVector(xyz);
            Vector3d r_xyz = r.quat_._transformVector(xyz);

            assert( rr_xyz.isApprox(r_xyz) );
        }
        std::cout << ".";
    }



    return 0;
}
