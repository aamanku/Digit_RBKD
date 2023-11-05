#ifndef SPATIAL_AND_VECTOR_ARITHMETIC_HXX
#define SPATIAL_AND_VECTOR_ARITHMETIC_HXX

#include <eigen3/Eigen/Dense>
#include "3d_vectors_and_rotation.hxx"
#include "common_defines.hxx"

// Porting all the function form roy featherstone's rbda library
// https://royfeatherstone.org/spatial/v2/index.html



namespace rbda {

    /**
     * @brief crm(v)  calculates the 6x6 (or 3x3) matrix such that the expression crm(v)*m is the cross product of the motion vectors v and m.
     * 
     * @param v 6x1 (or 3x1) motion vector
     * @return Eigen::Matrix<myfloat,6,6> 6x6 (or 3x3) matrix
     */
    inline Eigen::Matrix<myfloat,6,6> crm(Eigen::Matrix<myfloat,6,1> v) {
        Eigen::Matrix<myfloat,6,6> vcross;
        vcross << 0, -v(2), v(1), 0, 0, 0,
                  v(2), 0, -v(0), 0, 0, 0,
                  -v(1), v(0), 0, 0, 0, 0,
                  0, -v(5), v(4), 0, -v(2), v(1),
                  v(5), 0, -v(3), v(2), 0, -v(0),
                  -v(4), v(3), 0, -v(1), v(0), 0;
        return vcross;
    }

    inline Eigen::Matrix<myfloat,3,3> crm(Eigen::Matrix<myfloat,3,1> v) {
        Eigen::Matrix<myfloat,3,3> vcross;
        vcross << 0, -v(2), v(1),
                  v(2), 0, -v(0),
                  -v(1), v(0), 0; // this is incorrect in the rbda matlab code
        return vcross;
    }

    /**
     * @brief crf(v)  calculates the 6x6 (or 3x3) matrix such that the expression crf(v)*f is the cross product of the force vectors v and f.
     * 
     * @param v 6x1 (or 3x1) force vector
     * @return Eigen::Matrix<myfloat,6,6> 6x6 (or 3x3) matrix
     */
    inline Eigen::Matrix<myfloat,6,6> crf(Eigen::Matrix<myfloat,6,1> v) {
        return -crm(v).transpose();
    }

    inline Eigen::Matrix<myfloat,3,3> crf(Eigen::Matrix<myfloat,3,1> v) {
        return -crm(v).transpose();
    }

    /**
     * @brief mcI_to_rbi convert mass (1x1), center of mass location (3x1), and inertia matrix (3x3) to 6x6 rigid-body inertia matrix.
     * 
     * @param m mass
     * @param c center of mass location
     * @param I inertia matrix
     * @return Eigen::Matrix<myfloat,6,6> 6x6 rigid-body inertia matrix
     */
    inline Eigen::Matrix<myfloat,6,6> mcI_to_rbi(myfloat m, Eigen::Matrix<myfloat,3,1> c, Eigen::Matrix<myfloat,3,3> I) {
        Eigen::Matrix<myfloat,6,6> rbi;
        rbi.block<3,3>(0,0) = I + m*crm(c)*crm(c).transpose();
        rbi.block<3,3>(0,3) = m*crm(c);
        rbi.block<3,3>(3,0) = m*crm(c).transpose();
        rbi.block<3,3>(3,3) = m*Eigen::Matrix<myfloat,3,3>::Identity();
        return rbi;
    }

    /**
     * @brief Struct to hold mass (1x1), center of mass location (3x1), and inertia matrix (3x3). 
     * 
     */
    struct mcI_struct {
        myfloat m;
        Eigen::Matrix<myfloat,3,1> c;
        Eigen::Matrix<myfloat,3,3> I;
    };

    /**
     * @brief rbi_to_mcI convert 6x6 rigid-body inertia matrix to mass (1x1), center of mass location (3x1), and inertia matrix (3x3).
     * 
     * @param rbi 6x6 rigid-body inertia matrix
     * @return mcI_struct<myfloat> Struct to hold mass (1x1), center of mass location (3x1), and inertia matrix (3x3).
     */
    
    inline mcI_struct rbi_to_mcI(Eigen::Matrix<myfloat,6,6> rbi) {
        mcI_struct mcI;
        mcI.m = rbi(5,5);
        Eigen::Matrix<myfloat,3,3> mC = rbi.block<3,3>(0,3);
        mcI.c = unskew(mC)/mcI.m;
        mcI.I = rbi.block<3,3>(0,0) - mC*mC.transpose()/mcI.m;
        return mcI;
    }

    /**
     * @brief rotx(theta)  calculates the coordinate transform matrix from A to B coordinates for spatial motion vectors, where coordinate frame B is rotated by an angle theta (radians) relative to frame A about their common X axis.
     * 
     * @param theta angle in radians
     * @return Eigen::Matrix<myfloat,6,6> 6x6 coordinate transform matrix
     */
    
    inline Eigen::Matrix<myfloat,6,6> rotx(myfloat theta) {
        Eigen::Matrix<myfloat,6,6> X;
        X << rx(theta), Eigen::Matrix<myfloat,3,3>::Zero(),
             Eigen::Matrix<myfloat,3,3>::Zero(), rx(theta);
        return X;
    }

    /**
     * @brief roty(theta)  calculates the coordinate transform matrix from A to B coordinates for spatial motion vectors, where coordinate frame B is rotated by an angle theta (radians) relative to frame A about their common Y axis.
     * 
     * @param theta angle in radians
     * @return Eigen::Matrix<myfloat,6,6> 6x6 coordinate transform matrix
     */
    
    inline Eigen::Matrix<myfloat,6,6> roty(myfloat theta) {
        Eigen::Matrix<myfloat,6,6> X;
        X << ry(theta), Eigen::Matrix<myfloat,3,3>::Zero(),
             Eigen::Matrix<myfloat,3,3>::Zero(), ry(theta);
        return X;
    }

    /**
     * @brief rotz(theta) calculates the coordinate transform matrix from A to B coordinates for spatial motion vectors, where coordinate frame B is rotated by an angle theta (radians) relative to frame A about their common Z axis.
     * 
     * @param theta angle in radians
     * @return Eigen::Matrix<myfloat,6,6> 6x6 coordinate transform matrix
     */
    
    inline Eigen::Matrix<myfloat,6,6> rotz(myfloat theta) {
        Eigen::Matrix<myfloat,6,6> X;
        X << rz(theta), Eigen::Matrix<myfloat,3,3>::Zero(),
             Eigen::Matrix<myfloat,3,3>::Zero(), rz(theta);
        return X;
    }

    /**
     * @brief xlt  spatial coordinate transform (translation of origin). xlt(r)  calculates the coordinate transform matrix from A to B coordinates for spatial motion vectors, in which frame B is translated by an amount r (3D vector) relative to frame A.  r can be a row or column vector.
     * 
     * @param r 3D vector
     * @return Eigen::Matrix<myfloat,6,6> 6x6 coordinate transform matrix
     */
    
    inline Eigen::Matrix<myfloat,6,6> xlt(Eigen::Matrix<myfloat,3,1> r) {
        Eigen::Matrix<myfloat,6,6> X;
        X << Eigen::Matrix<myfloat,3,3>::Identity(), Eigen::Matrix<myfloat,3,3>::Zero(),
             -skew(r), Eigen::Matrix<myfloat,3,3>::Identity();
        return X;
    }

    /**
     * @brief plux  compose/decompose Plucker coordinate transform. X=plux(E,r) and [E,r]=plux(X)  compose a Plucker coordinate transform X from its component parts E and r, and decompose it into those parts, respectively.  E is a 3x3 rotational coordinate transform and r is a 3D vector.  r is returned as a column vector, but it can be supplied as a row or column vector.  X is a coordinate transform corresponding to a shift of origin by an amount specified by r, followed by a rotation about the new origin as specified by E. 
     * @param E 3x3 matrix
     * @param r 3D vector
     * @return Eigen::Matrix<myfloat,6,6> 6x6 coordinate transform matrix
     */
    
    inline Eigen::Matrix<myfloat,6,6> plux(Eigen::Matrix<myfloat,3,3> E, Eigen::Matrix<myfloat,3,1> r) {
        Eigen::Matrix<myfloat,6,6> X;
        X << E, Eigen::Matrix<myfloat,3,3>::Zero(),
             -E*skew(r), E;
        return X;
    }

    /**
     * @brief A struct to hold E and r
     * 
     */
    
    struct Er_struct {
        Eigen::Matrix<myfloat,3,3> E;
        Eigen::Matrix<myfloat,3,1> r;
    };

    /**
     * @brief plux  compose/decompose Plucker coordinate transform. X=plux(E,r) and [E,r]=plux(X)  compose a Plucker coordinate transform X from its component parts E and r, and decompose it into those parts, respectively.  E is a 3x3 rotational coordinate transform and r is a 3D vector.  r is returned as a column vector, but it can be supplied as a row or column vector.  X is a coordinate transform corresponding to a shift of origin by an amount specified by r, followed by a rotation about the new origin as specified by E. 
     * @param X 6x6 spatial coordinate transform matrix 
     * @return Eigen::Matrix<myfloat,6,6> 6x6 coordinate transform matrix
     */
    inline Er_struct plux(Eigen::Matrix<myfloat,6,6> X) {
        Er_struct Er;
        Er.E = X.block<3,3>(0,0);
        Er.r = -unskew(Er.E.transpose()*X.block<3,3>(3,0));
        return Er;
    }

    /**
     * @brief pluho  convert Plucker <--> 4x4 homogeneous coordinate transform. X=pluho(T) and T=pluho(X) convert between a Plucker coordinate transform matrix X and a 4x4 homogeneous coordinate transform matrix T.  If the argument is a 6x6 matrix then it is taken to be X, otherwise T. NOTE: the 4x4 matrices used in 3D graphics (e.g. OpenGL and Matlab handle graphics) are displacement operators, which are the inverses of coordinate transforms.  For example, to set the 'matrix' property of a Matlab hgtransform graphics object so as to rotate its children by an angle theta about the X axis, use inv(pluho(rotx(theta))).
     * 
     * @param X 6x6 spatial coordinate transform matrix
     * @return Eigen::Matrix<myfloat,4,4> 4x4 homogeneous coordinate transform matrix
     */
    
    inline Eigen::Matrix<myfloat,4,4> pluho(Eigen::Matrix<myfloat,6,6> X) {
        Eigen::Matrix<myfloat,4,4> H;
        const Eigen::Matrix<myfloat,3,3> E = X.block<3,3>(0,0); 
        const Eigen::Matrix<myfloat,3,3> mErx = X.block<3,3>(3,0); // -E r cross
        H << E, -unskew(mErx*E.transpose()), Eigen::Matrix<myfloat,3,1>::Zero(), 1;
        return H;
    }

    
    inline Eigen::Matrix<myfloat,6,6> pluho(Eigen::Matrix<myfloat,4,4> H) {
        Eigen::Matrix<myfloat,6,6> X;
        const Eigen::Matrix<myfloat,3,3> E = H.block<3,3>(0,0);
        const Eigen::Matrix<myfloat,3,1> mEr = H.block<3,1>(0,3); // -E r
        X << E, Eigen::Matrix<myfloat,3,3>::Zero(),
             skew(mEr)*E, E; 
        return X;
    }

    /**
     * @brief  Xpt  apply Plucker/planar coordinate transform to 2D/3D points xp=Xpt(X,p)  applies the coordinate transform X to the points in p, returning the new coordinates in xp.
     * 
     * @param X 6x6 spatial coordinate transform matrix
     * @param p 3D vector
     * @return Eigen::Matrix<myfloat,3,1> 3D vector
     */
    
    inline Eigen::Matrix<myfloat,3,1> Xpt(Eigen::Matrix<myfloat,6,6> X, Eigen::Matrix<myfloat,3,1> p) {
        Er_struct Er = plux(X);
        return Er.E*(p - Er.r);
    }

    /**
     * @brief Vpt  spatial/planar velocities --> velocities at points vp=Vpt(v,p)  calculates the linear velocities vp at one or more points p due to one or more spatial/planar velocities v.
     * 
     * @tparam T 
     * @param v 
     * @param p 
     * @return Eigen::Matrix<myfloat,3,1> 
     */
    
    inline Eigen::Matrix<myfloat,3,1> Vpt(Eigen::Matrix<myfloat,6,1> v, Eigen::Matrix<myfloat,3,1> p) {
        return v.block<3,1>(3,0) + v.block<3,1>(0,0).cross(p);
    }

    /**
     * @brief f=Fpt(fp,p)  converts one or more linear forces fp acting at one or more points p to their equivalent spatial or planar forces.
     * 
     * @param fp Force at point p
     * @param p Point
     * @return Eigen::Matrix<myfloat,6,1> Spatial force
     */
    
    inline Eigen::Matrix<myfloat,6,1> Fpt(Eigen::Matrix<myfloat,3,1> fp, Eigen::Matrix<myfloat,3,1> p) {
        Eigen::Matrix<myfloat,6,1> f;
        f << p.cross(fp), fp;
        return f;
    }


};

#endif // SPATIAL_AND_VECTOR_ARITHMETIC_HXX
