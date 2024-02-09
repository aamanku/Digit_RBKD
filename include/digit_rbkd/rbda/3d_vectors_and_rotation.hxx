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
#ifndef THREE_D_VECTORS_AND_ROTATION_HXX
#define THREE_D_VECTORS_AND_ROTATION_HXX

#include <eigen3/Eigen/Dense>
#include "digit_rbkd/common_defines.hxx"

// Porting all the function form roy featherstone's rbda library
// https://royfeatherstone.org/spatial/v2/index.html

namespace rbda {

/**
 * @brief rx(theta)  calculates the 3x3 rotational coordinate transform matrix from A to B coordinates, where coordinate frame B is rotated by an angle theta (radians) relative to frame A about their common X axis.
 * 
 * @param theta Angle in radians
 * @return Eigen::Matrix<myfloat,3,3> E 3x3 rotational coordinate transform matrix from A to B coordinates
 */
inline Eigen::Matrix<myfloat,3,3> rx(myfloat theta){
    Eigen::Matrix<myfloat,3,3> E;
    E << 1, 0, 0,
         0, cos(theta), sin(theta),
         0, -sin(theta), cos(theta);
    return E;
}

/**
 * @brief ry(theta)  calculates the 3x3 rotational coordinate transform matrix from A to B coordinates, where coordinate frame B is rotated by an angle theta (radians) relative to frame A about their common Y axis.
 * 
 * @param theta Angle in radians
 * @return Eigen::Matrix<myfloat,3,3> E 3x3 rotational coordinate transform matrix from A to B coordinates
 */
inline Eigen::Matrix<myfloat,3,3> ry(myfloat theta){
    Eigen::Matrix<myfloat,3,3> E;
    E << cos(theta), 0, -sin(theta),
         0, 1, 0,
         sin(theta), 0, cos(theta);
    return E;
}

/**
 * @brief rz(theta)  calculates the 3x3 rotational coordinate transform matrix from A to B coordinates, where coordinate frame B is rotated by an angle theta (radians) relative to frame A about their common Z axis.
 * 
 * @param theta Angle in radians
 * @return Eigen::Matrix<myfloat,3,3> E 3x3 rotational coordinate transform matrix from A to B coordinates
 */
inline Eigen::Matrix<myfloat,3,3> rz(myfloat theta){
    Eigen::Matrix<myfloat,3,3> E;
    E << cos(theta), sin(theta), 0,
         -sin(theta), cos(theta), 0,
         0, 0, 1;
    return E;
}

///**
// * @brief vtoE(v) converts a 3x1 vector v to a 3x3 coordinate transform matrix E.
// *
// * @param v 3x1 vector
// * @return Eigen::Matrix<myfloat,3,3> E 3x3 coordinate transform matrix
// */
//inline Eigen::Matrix<myfloat,3,3> vtoE(Eigen::Matrix<myfloat,3,1> v){
//    myfloat theta = v.norm();
//    Eigen::Matrix<myfloat,3,3> E;
//    if(theta == 0){
//        E = Eigen::Matrix<myfloat,3,3>::Identity();
//    }else{
//        Eigen::Matrix<myfloat,3,1> axis = v/theta;
//        E = Eigen::AngleAxis<myfloat>(theta, axis).toRotationMatrix();
//    }
//    return E;
//}
//
///**
// * @brief Etov(E) converts a 3x3 coordinate transform matrix E to a 3x1 vector v.
// *
// * @param E 3x3 coordinate transform matrix
// * @return Eigen::Matrix<myfloat,3,1> v 3x1 vector
// */
//inline Eigen::Matrix<myfloat,3,1> Etov(Eigen::Matrix<myfloat,3,3> E) {
//    Eigen::AngleAxis<myfloat> aa(E);
//    return aa.angle()*aa.axis();
//}


// /**
//  * @brief Etoq(E) converts a 3x3 coordinate rotation matrix E to a 4x1 quaternion q.
//  * 
//  * @param E 3x3 coordinate rotation matrix
//  * @return Eigen::Matrix<myfloat,4,1> q 4x1 quaternion
//  */
// inline Eigen::Matrix<myfloat,4,1> Etoq(Eigen::Matrix<myfloat,3,3> E){
//     Eigen::Quaternion<myfloat> q(E);
//     return Eigen::Matrix<myfloat,4,1>(q.w(), q.x(), q.y(), q.z());
// }

// /**
//  * @brief qtoE(q) converts a 4x1 quaternion q to a 3x3 coordinate rotation matrix E.
//  * 
//  * @param q 4x1 quaternion
//  * @return Eigen::Matrix<myfloat,3,3> E 3x3 coordinate rotation matrix
//  */
// inline Eigen::Matrix<myfloat,3,3> qtoE(Eigen::Matrix<myfloat,4,1> q){
//     Eigen::Quaternion<myfloat> quat(q(0), q(1), q(2), q(3));
//     return quat.toRotationMatrix();
// }

/**
 * @brief skew  convert 3D vector <--> 3x3 skew-symmetric matrix
 * 
 * @param v 3x1 vector
 * @return Eigen::Matrix<myfloat,3,3> E 3x3 skew-symmetric matrix
 */
inline Eigen::Matrix<myfloat,3,3> skew(Eigen::Matrix<myfloat,3,1> v){
    Eigen::Matrix<myfloat,3,3> E;
    E << 0, -v(2), v(1),
         v(2), 0, -v(0),
         -v(1), v(0), 0;
    return E;
}

inline Eigen::Matrix<myfloat,3,1> unskew(Eigen::Matrix<myfloat,3,3> E){
    Eigen::Matrix<myfloat,3,1> v;
    v << E(2,1), E(0,2), E(1,0);
    return v;
}




};


#endif // THREE_D_VECTORS_AND_ROTATION_HXX
