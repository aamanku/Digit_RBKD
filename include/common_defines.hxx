
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
