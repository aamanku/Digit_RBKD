#ifndef DATA_STRUCTURES_HXX
#define DATA_STRUCTURES_HXX

#include "3d_vectors_and_rotation.hxx"
#include "spatial_vector_arithmetic.hxx"
#include "common_defines.hxx"


namespace rbda
{




struct PluckerTransform
{
    // Eigen::Matrix<myfloat, 6, 6> X;
    Eigen::Matrix<myfloat,3,3> E;
    Eigen::Matrix<myfloat,3,1> r;

    // constructors
    PluckerTransform() : E(Eigen::Matrix<myfloat,3,3>::Identity()),r(Eigen::Matrix<myfloat,3,1>::Zero()){};
    PluckerTransform(Eigen::Matrix<myfloat, 6, 6> X) {
        this->E = X.block<3,3>(0,0);
        this->r = unskew(X.block<3,3>(3,0)*X.block<3,3>(0,0).transpose());
    };

    PluckerTransform(Eigen::Matrix<myfloat,3,1> euler_deg, Eigen::Matrix<myfloat,3,1> p)
    {   
        Eigen::AngleAxis<myfloat> aa(euler_deg(0)*M_PI/180.0, Eigen::Matrix<myfloat,3,1>::UnitX());
        Eigen::AngleAxis<myfloat> ab(euler_deg(1)*M_PI/180.0, Eigen::Matrix<myfloat,3,1>::UnitY());
        Eigen::AngleAxis<myfloat> ac(euler_deg(2)*M_PI/180.0, Eigen::Matrix<myfloat,3,1>::UnitZ());
        
        this->E = (aa.matrix()*ab.matrix()*ac.matrix()).transpose();
        this->r = p;
    };

    PluckerTransform(Eigen::Matrix<myfloat,3,3> E, Eigen::Matrix<myfloat,3,1> r) : E(E), r(r){};

    // inverse
    PluckerTransform inverse() const {
        PluckerTransform Xinv;
        Xinv.E = this->E.transpose();
        Xinv.r = -this->E.transpose()*this->r;
        return Xinv;
    }

    Eigen::Matrix<myfloat,6,6> toMatrix() const {
        Eigen::Matrix<myfloat,6,6 > X;
        X.setZero();
        X.block<3,3>(0,0) = this->E;
        X.block<3,3>(3,3) = this->E;
        X.block<3,3>(3,0) =-this->E*skew(this->r);
        return X;
    }


    // operators *
    friend PluckerTransform operator*(const PluckerTransform &lhs, const PluckerTransform &rhs)
    {
        PluckerTransform X;
        X.E = lhs.E*rhs.E;
        X.r = rhs.r + rhs.E.transpose()*lhs.r;
        return X;
    }

    // operator =
    PluckerTransform &operator=(const PluckerTransform &rhs)
    {
        this->E = rhs.E;
        this->r = rhs.r;
        return *this;
    }

    // operator <<
    friend std::ostream &operator<<(std::ostream &os, const PluckerTransform &plucker_transform)
    {
        os << "E: " << std::endl;
        os << plucker_transform.E << std::endl;
        os << "r: " << std::endl;
        os << plucker_transform.r << std::endl;
        os << "X: " << std::endl;
        os << plucker_transform.toMatrix() << std::endl;

        return os;
    }


};

struct SpatialInertia
{
    // Eigen::Matrix<myfloat, 6, 6> mat;
    myfloat m;
    Eigen::Matrix<myfloat,3,1> h;
    Eigen::Matrix<myfloat,3,3> I; // TODO: work with triangular matrix

    // constructors
    SpatialInertia() : m(0),h(Eigen::Matrix<myfloat,3,1>::Zero()),I(Eigen::Matrix<myfloat,3,3>::Zero()){};

    SpatialInertia(myfloat m, Eigen::Matrix<myfloat, 3, 1> c, Eigen::Matrix<myfloat, 3, 3> Ic) {
        this->m = m;
        this->h = m*c;
        this->I = Ic - m*skew(c)*skew(c); // parallel axis theorem
    };        

    SpatialInertia(myfloat m, Eigen::Matrix<myfloat, 3, 1> c, Eigen::Matrix<myfloat, 6, 1> Ic) {
        this->m = m;
        this->h = m*c;
        Eigen::Matrix<myfloat, 3, 3> Ic_full;
        Ic_full << Ic(0), Ic(3), Ic(4),
             Ic(3), Ic(1), Ic(5),
              Ic(4), Ic(5), Ic(2);
        this->I = Ic_full - m*skew(c)*skew(c); // parallel axis theorem
    };

    Eigen::Matrix<myfloat,6,6> mat() const {
        Eigen::Matrix<myfloat,6,6> mat;
        mat.block<3,3>(0,0) = this->I;
        mat.block<3,3>(0,3) = skew(this->h);
        mat.block<3,3>(3,0) = -skew(this->h);
        mat.block<3,3>(3,3) = this->m*Eigen::Matrix<myfloat,3,3>::Identity();
        return mat;
    }

    // operators <<
    friend std::ostream &operator<<(std::ostream &os, const SpatialInertia &spI)
    {
        os << "mass: " << spI.m << std::endl;
        os << "h: " << spI.h.transpose() << std::endl;
        os << "c: " << (spI.h/spI.m).transpose() << std::endl;
        os << "I: " << std::endl;
        os << spI.I << std::endl;
        os << "Ic: " << std::endl;
        os << (spI.I + spI.m*skew(spI.h/spI.m)*skew(spI.h/spI.m)) << std::endl;
        os << "mat: " << std::endl;
        os << spI.mat() << std::endl;

        return os;
    }
};

}; // namespace rbda

#endif // DATA_STRUCTURES_HXX
