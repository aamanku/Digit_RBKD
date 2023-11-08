#ifndef DYNAMICS_FUNCTIONS_HXX
#define DYNAMICS_FUNCTIONS_HXX

#include "3d_vectors_and_rotation.hxx"
#include "spatial_vector_arithmetic.hxx"
#include "common_defines.hxx"
#include "data_structures.hxx"

namespace rbda
{

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