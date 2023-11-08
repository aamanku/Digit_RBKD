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
#ifndef COMMON_DEFINES_HXX
#define COMMON_DEFINES_HXX
#include <eigen3/Eigen/Dense>

#define DEBUG 1
#define MYINF 1e10

typedef double myfloat;
typedef int myint;

typedef Eigen::Matrix<myfloat, 3, 1> Vector3;
typedef Eigen::Matrix<myfloat, 3, 3> Matrix3;

typedef Eigen::Matrix<myfloat, 4, 1> Vector4;
typedef Eigen::Matrix<myfloat, 4, 4> Matrix4;

typedef Eigen::Matrix<myfloat, 6, 1> Vector6;
typedef Eigen::Matrix<myfloat, 6, 6> Matrix6;

enum class JointType { 
    FIXED, 
    REVOLUTE, 
    PRISMATIC, 
    SPHERICAL, 
    UNIVERSAL, 
    PLANAR, 
    FLOATING, 
    TRANSLATIONAL, 
    ROTATIONAL 
};

// define the number of degrees of freedom for each joint type
const myint dof[9] = {0, 1, 1, 3, 2, 3, 6, 3, 3};

// define the number of generalized coordinates for each joint type
const myint gc[9] = {0, 1, 1, 4, 3, 4, 7, 6, 6};

// Print Macro
#if DEBUG
#define PRINT(x) std::cout << x << std::endl
#else
#define PRINT(x)
#endif


#endif // COMMON_DEFINES_HXX
