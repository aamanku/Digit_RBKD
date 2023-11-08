/*
MIT License

Copyright (c) 2023 Abhijeet M. Kulkarni

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
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