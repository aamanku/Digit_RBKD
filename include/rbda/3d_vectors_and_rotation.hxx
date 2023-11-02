#ifndef THREE_D_VECTORS_AND_ROTATION_HXX
#define THREE_D_VECTORS_AND_ROTATION_HXX

#include <eigen3/Eigen/Dense>

// Porting all the function form roy featherstone's rbda library
// https://royfeatherstone.org/spatial/v2/index.html

namespace rbda {

/**
 * @brief rx(theta)  calculates the 3x3 rotational coordinate transform matrix from A to B coordinates, where coordinate frame B is rotated by an angle theta (radians) relative to frame A about their common X axis.
 * 
 * @tparam T Data type
 * @param theta Angle in radians
 * @return Eigen::Matrix<T,3,3> E 3x3 rotational coordinate transform matrix from A to B coordinates
 */
template <typename T>
inline Eigen::Matrix<T,3,3> rx(T theta){
    Eigen::Matrix<T,3,3> E;
    E << 1, 0, 0,
         0, cos(theta), sin(theta),
         0, -sin(theta), cos(theta);
    return E;
}

/**
 * @brief ry(theta)  calculates the 3x3 rotational coordinate transform matrix from A to B coordinates, where coordinate frame B is rotated by an angle theta (radians) relative to frame A about their common Y axis.
 * 
 * @tparam T Data type
 * @param theta Angle in radians
 * @return Eigen::Matrix<T,3,3> E 3x3 rotational coordinate transform matrix from A to B coordinates
 */
template <typename T>
inline Eigen::Matrix<T,3,3> ry(T theta){
    Eigen::Matrix<T,3,3> E;
    E << cos(theta), 0, -sin(theta),
         0, 1, 0,
         sin(theta), 0, cos(theta);
    return E;
}

/**
 * @brief rz(theta)  calculates the 3x3 rotational coordinate transform matrix from A to B coordinates, where coordinate frame B is rotated by an angle theta (radians) relative to frame A about their common Z axis.
 * 
 * @tparam T Data type
 * @param theta Angle in radians
 * @return Eigen::Matrix<T,3,3> E 3x3 rotational coordinate transform matrix from A to B coordinates
 */
template <typename T>
inline Eigen::Matrix<T,3,3> rz(T theta){
    Eigen::Matrix<T,3,3> E;
    E << cos(theta), sin(theta), 0,
         -sin(theta), cos(theta), 0,
         0, 0, 1;
    return E;
}

/**
 * @brief vtoE(v) converts a 3x1 vector v to a 3x3 coordinate transform matrix E.
 * 
 * @tparam T Data type
 * @param v 3x1 vector
 * @return Eigen::Matrix<T,3,3> E 3x3 coordinate transform matrix
 */
template <typename T>
inline Eigen::Matrix<T,3,3> vtoE(Eigen::Matrix<T,3,1> v){
    T theta = v.norm();
    Eigen::Matrix<T,3,3> E;
    if(theta == 0){
        E = Eigen::Matrix<T,3,3>::Identity();
    }else{
        Eigen::Matrix<T,3,1> axis = v/theta;
        E = Eigen::AngleAxis<T>(theta, axis).toRotationMatrix();
    }
}

/**
 * @brief Etov(E) converts a 3x3 coordinate transform matrix E to a 3x1 vector v.
 * 
 * @tparam T Data type
 * @param E 3x3 coordinate transform matrix
 * @return Eigen::Matrix<T,3,1> v 3x1 vector
 */
template <typename T>
inline Eigen::Matrix<T,3,1> Etov(Eigen::Matrix<T,3,3> E) {
    Eigen::AngleAxis<T> aa(E);
    return aa.angle()*aa.axis();
}

/**
 * @brief  rv  3D rotation vector <--> 3x3 coordinate rotation matrix E=rv(v) and v=rv(E) convert between a rotation vector v, whose magnitude and direction describe the angle and axis of rotation of a coordinate frame B relative to frame A, and the 3x3 coordinate rotation matrix E that transforms from A to B coordinates.
 * 
 * @tparam T Data type
 * @param E 3x3 coordinate rotation matrix
 * @return Eigen::Matrix<T,3,1> v 3x1 vector
 */
template <typename T>
inline Eigen::Matrix<T,3,1> rv(Eigen::Matrix<T,3,3> E){
    return Etov(E);
}

/**
 * @brief  rv  3D rotation vector <--> 3x3 coordinate rotation matrix E=rv(v) and v=rv(E) convert between a rotation vector v, whose magnitude and direction describe the angle and axis of rotation of a coordinate frame B relative to frame A, and the 3x3 coordinate rotation matrix E that transforms from A to B coordinates.
 * 
 * @tparam T Data type
 * @param v 3x1 vector
 * @return E 3x3 coordinate rotation matrix
 */
template <typename T>
inline Eigen::Matrix<T,3,3> rv(Eigen::Matrix<T,3,1> v){
    return vtoE(v);
}

/**
 * @brief Etoq(E) converts a 3x3 coordinate rotation matrix E to a 4x1 quaternion q.
 * 
 * @tparam T Data type
 * @param E 3x3 coordinate rotation matrix
 * @return Eigen::Matrix<T,4,1> q 4x1 quaternion
 */
template <typename T>
inline Eigen::Matrix<T,4,1> Etoq(Eigen::Matrix<T,3,3> E){
    Eigen::Quaternion<T> q(E);
    return Eigen::Matrix<T,4,1>(q.w(), q.x(), q.y(), q.z());
}

/**
 * @brief qtoE(q) converts a 4x1 quaternion q to a 3x3 coordinate rotation matrix E.
 * 
 * @tparam T Data type
 * @param q 4x1 quaternion
 * @return Eigen::Matrix<T,3,3> E 3x3 coordinate rotation matrix
 */
template <typename T>
inline Eigen::Matrix<T,3,3> qtoE(Eigen::Matrix<T,4,1> q){
    Eigen::Quaternion<T> quat(q(0), q(1), q(2), q(3));
    return quat.toRotationMatrix();
}

/**
 * @brief  rq  unit quaternion <--> 3x3 coordinate rotation matrix E=rq(q) and q=rq(E)  convert between a unit quaternion q, representing the orientation of a coordinate frame B relative to frame A, and the 3x3 coordinate rotation matrix E that transforms from A to B coordinates.
 * @tparam T Data type
 * @param E 3x3 coordinate rotation matrix
 * @return Eigen::Matrix<T,4,1> q 4x1 quaternion
 */
template <typename T>
inline Eigen::Matrix<T,4,1> rq(Eigen::Matrix<T,3,3> E){
    return Etoq(E);
}

/**
 * @brief  rq  unit quaternion <--> 3x3 coordinate rotation matrix E=rq(q) and q=rq(E)  convert between a unit quaternion q, representing the orientation of a coordinate frame B relative to frame A, and the 3x3 coordinate rotation matrix E that transforms from A to B coordinates.
 * @tparam T Data type
 * @param q 4x1 quaternion
 * @return Eigen::Matrix<T,3,3> E 3x3 coordinate rotation matrix
 */
template <typename T>
inline Eigen::Matrix<T,3,3> rq(Eigen::Matrix<T,4,1> q){
    return qtoE(q);
}

/**
 * @brief skew  convert 3D vector <--> 3x3 skew-symmetric matrix
 * 
 * @tparam T Data type
 * @param v 3x1 vector
 * @return Eigen::Matrix<T,3,3> E 3x3 skew-symmetric matrix
 */
template <typename T>
inline Eigen::Matrix<T,3,3> skew(Eigen::Matrix<T,3,1> v){
    Eigen::Matrix<T,3,3> E;
    E << 0, -v(2), v(1),
         v(2), 0, -v(0),
         -v(1), v(0), 0;
    return E;
}

template <typename T>
inline Eigen::Matrix<T,3,1> unskew(Eigen::Matrix<T,3,3> E){
    Eigen::Matrix<T,3,1> v;
    v << E(2,1), E(0,2), E(1,0);
    return v;
}

/**
 * @brief skew  convert 3D vector <--> 3x3 skew-symmetric matrix
 * 
 * @tparam T Data type
 * @param E 3x3 skew-symmetric matrix
 * @return Eigen::Matrix<T,3,1> v 3x1 vector
 */
template <typename T>
inline Eigen::Matrix<T,3,1> skew(Eigen::Matrix<T,3,3> E){
    return unskew(E);
}



};


#endif // THREE_D_VECTORS_AND_ROTATION_HXX
