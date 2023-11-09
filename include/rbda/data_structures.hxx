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
#ifndef DATA_STRUCTURES_HXX
#define DATA_STRUCTURES_HXX

#include "3d_vectors_and_rotation.hxx"
#include "spatial_vector_arithmetic.hxx"
#include "common_defines.hxx"


namespace rbda
{

struct Pose {
        Eigen::Matrix<myfloat,3,3> R;
        Eigen::Matrix<myfloat,3,1> p;

        // constructors
        Pose() : R(Eigen::Matrix<myfloat,3,3>::Identity()), p(Eigen::Matrix<myfloat,3,1>::Zero()){};
        Pose(Eigen::Matrix<myfloat,3,3> R, Eigen::Matrix<myfloat,3,1> p) : R(R), p(p){};
        Pose(Eigen::Matrix<myfloat,3,1> euler_deg, Eigen::Matrix<myfloat,3,1> p) {
            Eigen::AngleAxis<myfloat> aa(euler_deg(0)*M_PI/180.0, Eigen::Matrix<myfloat,3,1>::UnitX());
            Eigen::AngleAxis<myfloat> ab(euler_deg(1)*M_PI/180.0, Eigen::Matrix<myfloat,3,1>::UnitY());
            Eigen::AngleAxis<myfloat> ac(euler_deg(2)*M_PI/180.0, Eigen::Matrix<myfloat,3,1>::UnitZ());
            
            this->R = (aa.matrix()*ab.matrix()*ac.matrix());
            this->p = p;
        }

        // set to identity
        void setIdentity()
        {
                R = Eigen::Matrix<myfloat,3,3>::Identity();
                p = Eigen::Matrix<myfloat,3,1>::Zero();
        };


        // operators <<
        friend std::ostream &operator<<(std::ostream &os, const Pose &pose)
        {
                os << "R: " << std::endl;
                os << pose.R << std::endl;
                os << "p: " << std::endl;
                os << pose.p << std::endl;
                return os;
        }
};
struct ForceVector {
    Eigen::Matrix<myfloat,3,1> n;
    Eigen::Matrix<myfloat,3,1> f;

    // constructors
    ForceVector() : n(Eigen::Matrix<myfloat,3,1>::Zero()), f(Eigen::Matrix<myfloat,3,1>::Zero()){};
    ForceVector(Eigen::Matrix<myfloat,3,1> n, Eigen::Matrix<myfloat,3,1> f) : n(n), f(f){};
    ForceVector(Eigen::Matrix<myfloat,6,1> fv) : n(fv.block<3,1>(0,0)), f(fv.block<3,1>(3,0)){};
    ForceVector(Eigen::Matrix<myfloat,6,-1> fv) : n(fv.block<3,1>(0,0)), f(fv.block<3,1>(3,0)){};

    Eigen::Matrix<myfloat,6,1> toVector() const {
        Eigen::Matrix<myfloat,6,1> fv;
        fv << this->n, this->f;
        return fv;
    }

    // operators <<
    friend std::ostream &operator<<(std::ostream &os, const ForceVector &fv)
    {
        os << "n: " << fv.n.transpose() << std::endl;
        os << "f: " << fv.f.transpose() << std::endl;
        return os;
    }

    // operators +
    friend ForceVector operator+(const ForceVector &lhs, const ForceVector &rhs)
    {
        ForceVector fv;
        fv.n = lhs.n + rhs.n;
        fv.f = lhs.f + rhs.f;
        return fv;
    }

    // operators -
    friend ForceVector operator-(const ForceVector &lhs, const ForceVector &rhs)
    {
        ForceVector fv;
        fv.n = lhs.n - rhs.n;
        fv.f = lhs.f - rhs.f;
        return fv;
    }

    // operators *
    friend ForceVector operator*(const myfloat &lhs, const ForceVector &rhs)
    {
        ForceVector fv;
        fv.n = lhs*rhs.n;
        fv.f = lhs*rhs.f;
        return fv;
    }

    // operators *
    friend ForceVector operator*(const ForceVector &lhs, const myfloat &rhs)
    {
        ForceVector fv;
        fv.n = lhs.n*rhs;
        fv.f = lhs.f*rhs;
        return fv;
    }

    // operators =
    ForceVector &operator=(const ForceVector &rhs)
    {
        this->n = rhs.n;
        this->f = rhs.f;
        return *this;
    }

    
};

struct MotionVector {
    Eigen::Matrix<myfloat,3,1> omega;
    Eigen::Matrix<myfloat,3,1> v;

    // constructors
    MotionVector() : omega(Eigen::Matrix<myfloat,3,1>::Zero()), v(Eigen::Matrix<myfloat,3,1>::Zero()){};
    MotionVector(Eigen::Matrix<myfloat,3,1> omega, Eigen::Matrix<myfloat,3,1> v) : omega(omega), v(v){};
    MotionVector(Eigen::Matrix<myfloat,6,1> mv) : omega(mv.block<3,1>(0,0)), v(mv.block<3,1>(3,0)){};
    MotionVector(Eigen::Matrix<myfloat,6,-1> mv) : omega(mv.block<3,1>(0,0)), v(mv.block<3,1>(3,0)){};

    Eigen::Matrix<myfloat,6,1> toVector() const {
        Eigen::Matrix<myfloat,6,1> mv;
        mv << this->omega, this->v;
        return mv;
    }

    // operators <<
    friend std::ostream &operator<<(std::ostream &os, const MotionVector &mv)
    {
        os << "omega: " << mv.omega.transpose() << std::endl;
        os << "v: " << mv.v.transpose() << std::endl;
        return os;
    }
    // operators +
    friend MotionVector operator+(const MotionVector &lhs, const MotionVector &rhs)
    {
        MotionVector mv;
        mv.omega = lhs.omega + rhs.omega;
        mv.v = lhs.v + rhs.v;
        return mv;
    }
    // operators -
    friend MotionVector operator-(const MotionVector &lhs, const MotionVector &rhs)
    {
        MotionVector mv;
        mv.omega = lhs.omega - rhs.omega;
        mv.v = lhs.v - rhs.v;
        return mv;
    }
    // operators *
    friend MotionVector operator*(const myfloat &lhs, const MotionVector &rhs)
    {
        MotionVector mv;
        mv.omega = lhs*rhs.omega;
        mv.v = lhs*rhs.v;
        return mv;
    }
    // operators *
    friend MotionVector operator*(const MotionVector &lhs, const myfloat &rhs)
    {
        MotionVector mv;
        mv.omega = lhs.omega*rhs;
        mv.v = lhs.v*rhs;
        return mv;
    }
    // operators =
    MotionVector &operator=(const MotionVector &rhs)
    {
        this->omega = rhs.omega;
        this->v = rhs.v;
        return *this;
    }
    
    MotionVector cross(const MotionVector &rhs) const {
        MotionVector mv;
        mv.omega = this->omega.cross(rhs.omega);
        mv.v = this->omega.cross(rhs.v) + this->v.cross(rhs.omega);
        return mv;
    }

    ForceVector cross(const ForceVector &rhs) const {
        ForceVector fv;
        fv.n = this->omega.cross(rhs.n) + this->v.cross(rhs.f);
        fv.f = this->omega.cross(rhs.f);
        return fv;
    }

};



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

    PluckerTransform(Pose pose) {
        this->E = pose.R.transpose(); // coordinate transform matrix from A to B coordinates
        this->r = pose.p;
    }

    PluckerTransform(Eigen::Matrix<myfloat,3,3> E, Eigen::Matrix<myfloat,3,1> r) : E(E), r(r){};

    // inverse
    PluckerTransform inverse() const {
        PluckerTransform Xinv;
        Xinv.E = this->E.transpose();
        Xinv.r = -this->E*this->r;
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

    Pose toPose() const {
        Pose pose;
        pose.R = this->E.transpose();
        pose.p = this->r;
        return pose;
    }

    void setIdentity() {
        this->E = Eigen::Matrix<myfloat,3,3>::Identity();
        this->r = Eigen::Matrix<myfloat,3,1>::Zero();
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

    // operator * (motion vector)
    friend MotionVector operator*(const PluckerTransform &lhs, const MotionVector &rhs)
    {
        MotionVector mv;
        mv.omega = lhs.E*rhs.omega;
        mv.v = lhs.E*(rhs.v - lhs.r.cross(rhs.omega));
        return mv;
    }

    ForceVector invApply(const ForceVector &rhs) const {
        ForceVector fv;
        fv.n = this->E.transpose()*rhs.n + this->r.cross(this->E.transpose()*rhs.f);
        fv.f = this->E.transpose()*rhs.f;
        return fv;
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

    SpatialInertia(Eigen::Matrix<myfloat,6,6> mat) {
        this->m = mat(3,3);
        this->h = unskew(mat.block<3,3>(0,3));
        this->I = mat.block<3,3>(0,0);
    };

    Eigen::Matrix<myfloat,6,6> toMatrix() const {
        Eigen::Matrix<myfloat,6,6> mat;
        mat.block<3,3>(0,0) = this->I;
        mat.block<3,3>(0,3) = skew(this->h);
        mat.block<3,3>(3,0) = -skew(this->h);
        mat.block<3,3>(3,3) = this->m*Eigen::Matrix<myfloat,3,3>::Identity();
        return mat;
    }

    // operators +
    friend SpatialInertia operator+(const SpatialInertia &lhs, const SpatialInertia &rhs)
    {
        SpatialInertia spI;
        spI.m = lhs.m + rhs.m;
        spI.h = lhs.h + rhs.h;
        spI.I = lhs.I + rhs.I;
        return spI;
    }

    // operators -
    friend SpatialInertia operator-(const SpatialInertia &lhs, const SpatialInertia &rhs)
    {
        SpatialInertia spI;
        spI.m = lhs.m - rhs.m;
        spI.h = lhs.h - rhs.h;
        spI.I = lhs.I - rhs.I;
        return spI;
    }

    // operators *
    friend ForceVector operator*(const SpatialInertia &lhs, const MotionVector &rhs)
    {
        ForceVector fv;
        fv.n = lhs.I*rhs.omega + lhs.h.cross(rhs.v);
        fv.f = lhs.m*rhs.v - lhs.h.cross(rhs.omega);
        return fv;
    }

    SpatialInertia apply(PluckerTransform X) const {
        SpatialInertia spI;
        spI.m = this->m;
        spI.h = X.E*(this->h-this->m*X.r);
        spI.I = X.E*(this->I + skew(X.r)*skew(this->h) + skew(spI.h)*skew(X.r))*X.E.transpose();
        return spI;
        // naive implementation
        // return SpatialInertia(X.toMatrix().transpose()*this->toMatrix()*X.toMatrix());
    }

    SpatialInertia invApply(PluckerTransform X) const {
        SpatialInertia spI;
        spI.m = this->m;
        spI.h = X.E.transpose()*this->h + this->m*X.r;
        spI.I = X.E.transpose()*this->I*X.E - skew(X.r)*skew(X.E.transpose()*this->h)-skew(spI.h)*skew(X.r);
        return spI;
    }

    Eigen::Matrix<myfloat,3,1> com_pos() const {
        return this->h/this->m;
    }

    Eigen::Matrix<myfloat,3,3> com_inertia() const {
        return this->I + this->m*skew(this->h/this->m)*skew(this->h/this->m);
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
        os << spI.toMatrix() << std::endl;

        return os;
    }
};


}; // namespace rbda

#endif // DATA_STRUCTURES_HXX
