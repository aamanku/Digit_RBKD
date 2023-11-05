#ifndef DATA_STRUCTURES_HXX
#define DATA_STRUCTURES_HXX

#include "3d_vectors_and_rotation.hxx"
#include "spatial_vector_arithmetic.hxx"
#include "common_defines.hxx"


namespace rbda
{




struct PluckerTransform
{
    Eigen::Matrix<myfloat, 6, 6> X;

    // constructors
    PluckerTransform() : X(Eigen::Matrix<myfloat, 6, 6>::Identity()){};
    PluckerTransform(Eigen::Matrix<myfloat, 6, 6> X) : X(X){};

    PluckerTransform(Eigen::Matrix<myfloat, 3, 3> E, Eigen::Matrix<myfloat, 3, 1> r) 
    {
        X = Eigen::Matrix<myfloat, 6, 6>::Zero();
        X.block<3,3>(0,0) = E;
        X.block<3,3>(3,0) = -E*skew(r);
        X.block<3,3>(3,3) = E;
    };

    PluckerTransform(Eigen::Matrix<myfloat,3,1> euler_deg, Eigen::Matrix<myfloat,3,1> r)
    {   
        Eigen::AngleAxis<myfloat> aa(euler_deg(0)*M_PI/180.0, Eigen::Matrix<myfloat,3,1>::UnitX());
        Eigen::AngleAxis<myfloat> ab(euler_deg(1)*M_PI/180.0, Eigen::Matrix<myfloat,3,1>::UnitY());
        Eigen::AngleAxis<myfloat> ac(euler_deg(2)*M_PI/180.0, Eigen::Matrix<myfloat,3,1>::UnitZ());
        
        Eigen::Matrix<myfloat,3,3> E = ac.matrix()*ab.matrix()*aa.matrix();
        X = Eigen::Matrix<myfloat, 6, 6>::Zero();
        X.block<3,3>(0,0) = E;
        X.block<3,3>(3,0) = -E*skew(r);
        X.block<3,3>(3,3) = E;
    };

    // operators <<
    friend std::ostream &operator<<(std::ostream &os, const PluckerTransform &plucker_transform)
    {
        os << "X: " << std::endl;
        os << plucker_transform.X << std::endl;

        return os;
    }
};

struct SpatialInertia
{
    Eigen::Matrix<myfloat, 6, 6> mat;

    // constructors
    SpatialInertia() : mat(Eigen::Matrix<myfloat, 6, 6>::Zero()){};
    SpatialInertia(Eigen::Matrix<myfloat, 6, 6> spI) : mat(spI){};

    SpatialInertia(myfloat m, Eigen::Matrix<myfloat, 3, 1> c, Eigen::Matrix<myfloat, 3, 3> I) {
        this->mat = mcI_to_rbi(m, c, I);
        
    }

    SpatialInertia(myfloat m, Eigen::Matrix<myfloat, 3, 1> c, Eigen::Matrix<myfloat, 6, 1> I) {
        Eigen::Matrix<myfloat, 3, 3> I_full;
        I_full << I(0), I(3), I(4),
              I(3), I(1), I(5),
              I(4), I(5), I(2);
        this->mat = mcI_to_rbi(m, c, I_full);
    }

    myfloat m() const { return mat(5, 5); }

    Eigen::Matrix<myfloat, 3, 1> c() const { return unskew(mat.block<3,3>(0,3)) / mat(5, 5); }

    Eigen::Matrix<myfloat, 3, 3> I() const { return mat.block<3,3>(0,0) - mat.block<3,3>(0,3)*mat.block<3,3>(0,3).transpose()/mat(5, 5); }

    // operators <<
    friend std::ostream &operator<<(std::ostream &os, const SpatialInertia &spI)
    {
        os << "mass: " << spI.m() << std::endl;
        os << "com: " << spI.c().transpose() << std::endl;
        os << "inertia: " << spI.I() << std::endl;

        return os;
    }
};

}; // namespace rbda

#endif // DATA_STRUCTURES_HXX
