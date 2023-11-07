#ifndef DYNAMICS_FUNCTIONS_HXX
#define DYNAMICS_FUNCTIONS_HXX

#include "3d_vectors_and_rotation.hxx"
#include "spatial_vector_arithmetic.hxx"
#include "common_defines.hxx"
#include "data_structures.hxx"

namespace rbda
{
    // /**
    //  * @brief Compute coordinate transformation matrix of free joint
    //  * 
    //  * @param q coordinate of the free joint [x,y,z,qw,qx,qy,qz] 
    //  * @return Eigen::Matrix<myfloat,6,6> coordinate transformation matrix
    //  */
    // inline Eigen::Matrix<myfloat,6,6> Xfree(Eigen::Matrix<myfloat,7,1> q) {
    //     const Eigen::Matrix<myfloat,3,1> r = q.block<3,1>(0,0); // first 3 q are 3D rotation [x,y,z]
    //     const Eigen::Matrix<myfloat,3,3> E = Eigen::Quaternion<myfloat>(-q(3),q(4),q(5),q(6)).toRotationMatrix(); // last 4 q are quaternion [w,x,y,z] -q(3) for transposed rotation
    //     std::cout<<"E: "<<E<<std::endl;
    //     return plux(E,r);
    // }

    inline Eigen::Matrix<myfloat,6,-1> motion_subspace_matrix(JointType joint_type, Eigen::Matrix<myfloat,3,1> joint_axis = Eigen::Matrix<myfloat,3,1>::UnitZ()) {
        Eigen::Matrix<myfloat,6,-1> S;
        switch (joint_type)
        {
        case JointType::REVOLUTE:
            S.resize(6,1);
            S.block<3,1>(0,0) = joint_axis;
            S.block<3,1>(3,0) = Eigen::Matrix<myfloat,3,1>::Zero();
            break;
        case JointType::TRANSLATIONAL:
            S.resize(6,3);
            S.block<3,3>(0,0) = Eigen::Matrix<myfloat,3,3>::Zero();
            S.block<3,3>(3,0) = Eigen::Matrix<myfloat,3,3>::Identity();
            break;
        case JointType::SPHERICAL:
            S.resize(6,3);
            S.block<3,3>(0,0) = Eigen::Matrix<myfloat,3,3>::Identity();
            S.block<3,3>(3,0) = Eigen::Matrix<myfloat,3,3>::Zero();
            break;
        default:
            std::cerr << "Joint type not implemented" << std::endl;
            assert(false);
            break;
        }
        return S;
    }

};
#endif // DYNAMICS_FUNCTIONS_HXX