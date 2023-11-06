#ifndef DYNAMICS_FUNCTIONS_HXX
#define DYNAMICS_FUNCTIONS_HXX

#include "3d_vectors_and_rotation.hxx"
#include "spatial_vector_arithmetic.hxx"
#include "common_defines.hxx"
#include "data_structures.hxx"

namespace rbda
{
    /**
     * @brief Compute coordinate transformation matrix of free joint
     * 
     * @param q coordinate of the free joint [x,y,z,qw,qx,qy,qz] 
     * @return Eigen::Matrix<myfloat,6,6> coordinate transformation matrix
     */
    inline Eigen::Matrix<myfloat,6,6> Xfree(Eigen::Matrix<myfloat,7,1> q) {
        const Eigen::Matrix<myfloat,3,1> r = q.block<3,1>(0,0); // first 3 q are 3D rotation [x,y,z]
        const Eigen::Matrix<myfloat,3,3> E = Eigen::Quaternion<myfloat>(-q(3),q(4),q(5),q(6)).toRotationMatrix(); // last 4 q are quaternion [w,x,y,z] -q(3) for transposed rotation
        std::cout<<"E: "<<E<<std::endl;
        return plux(E,r);
    }


};
#endif // DYNAMICS_FUNCTIONS_HXX