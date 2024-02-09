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
#include <casadi/casadi.hpp>
#include <Eigen/Core>
#define DEBUG 1
#define MYINF 1e15

//// This is a workaround to make the code compiling with Eigen.
//namespace casadi
//{
//    inline bool operator||(const bool x, const casadi::Matrix<SXElem> & /*y*/)
//    {
//        return x;
//    }
//}


namespace Eigen
{
    namespace internal
    {
        // Specialization of Eigen::internal::cast_impl for Casadi input types
        template<typename Scalar>
        struct cast_impl<casadi::SX,Scalar>
        {
#if EIGEN_VERSION_AT_LEAST(3,2,90)
            EIGEN_DEVICE_FUNC
#endif
            static inline Scalar run(const casadi::SX & x)
            {
                return static_cast<Scalar>(x);
            }
        };

#if EIGEN_VERSION_AT_LEAST(3,2,90) && !EIGEN_VERSION_AT_LEAST(3,2,93)
        template<typename Scalar, bool IsInteger>
    struct significant_decimals_default_impl< ::casadi::Matrix<Scalar>,IsInteger>
    {
      static inline int run()
      {
        return std::numeric_limits<Scalar>::digits10;
      }
    };
#endif
    }
}

namespace Eigen
{
    template<> struct NumTraits<casadi::SX>
    {
        using Real = casadi::SX;
        using NonInteger = casadi::SX;
        using Literal = casadi::SX;
        using Nested = casadi::SX;

        static bool constexpr IsComplex = false;
        static bool constexpr IsInteger = false;
        static int constexpr ReadCost = 1;
        static int constexpr AddCost = 1;
        static int constexpr MulCost = 1;
        static bool constexpr IsSigned = true;
        static bool constexpr RequireInitialization = true;


        static double epsilon()
        {
            return std::numeric_limits<double>::epsilon();
        }


        static double dummy_precision()
        {
            return 1e-10;
        }


        static double hightest()
        {
            return std::numeric_limits<double>::max();
        }


        static double lowest()
        {
            return std::numeric_limits<double>::min();
        }


        static int digits10()
        {
            return std::numeric_limits<double>::digits10;
        }
    };
};
namespace casadi
{
    // Copy casadi matrix to Eigen matrix
    template<typename MT, typename Scalar>
    inline void copy(::casadi::Matrix<Scalar> const & src,
                     Eigen::MatrixBase<MT> & dst)
    {
        Eigen::DenseIndex const m = src.size1();
        Eigen::DenseIndex const n = src.size2();

        dst.resize(m, n);

        for (Eigen::DenseIndex i = 0; i < m; ++i)
            for (Eigen::DenseIndex j = 0; j < n; ++j)
                dst(i, j) = src(i, j);
    }


    // Copy Eigen matrix to casadi matrix
    template<typename MT, typename Scalar>
    inline void copy(Eigen::MatrixBase<MT> const & src,
                     ::casadi::Matrix<Scalar> & dst)
    {
        Eigen::DenseIndex const m = src.rows();
        Eigen::DenseIndex const n = src.cols();

        dst.resize(m, n);

        for (Eigen::DenseIndex i = 0; i < m; ++i)
            for (Eigen::DenseIndex j = 0; j < n; ++j)
                dst(i, j) = src(i, j);
    }

} // namespace casadi




//using scalar = casadi::Matrix<casadi::SXElem>;

using myfloat = casadi::Matrix<casadi::SXElem>;
//using myfloat = double;
using myint = int;

using Vector3 = Eigen::Matrix<myfloat, 3, 1>;
using Matrix3 = Eigen::Matrix<myfloat, 3, 3>;

using Vector4 = Eigen::Matrix<myfloat, 4, 1>;
using Matrix4 = Eigen::Matrix<myfloat, 4, 4>;

using Vector6 = Eigen::Matrix<myfloat, 6, 1>;
using Matrix6 = Eigen::Matrix<myfloat, 6, 6>;







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

// function to invert matrix. If scalar is casadi::SX then use following else use Eigen
template<int nrows, int ncols,typename Scalar>
Eigen::Matrix<Scalar,nrows,ncols> invertMatrix(const Eigen::Matrix<Scalar,nrows,ncols> &input)
{
    return input.inverse();
}

template<int nrows, int ncols>
Eigen::Matrix<casadi::Matrix<casadi::SXElem>,nrows,ncols> invertMatrix(const Eigen::Matrix<casadi::Matrix<casadi::SXElem>,nrows,ncols> &input)
{
    const int nr = input.rows();
    const int nc = input.cols();
    assert(nr == nc && "Matrix should be square");
    // convert Eigen matrix to casadi matrix
    casadi::SX input_casadi = casadi::SX::zeros(nr,nc);
    // copy input to input_casadi
    for(int i=0; i<nr; i++)
    {
        for(int j=0; j<nc; j++)
        {
            input_casadi(i,j) = input(i,j);
        }
    }
    // invert the matrix
    casadi::SX output_casadi = casadi::SX::inv(input_casadi);
    // convert casadi matrix to Eigen matrix

    Eigen::Matrix<casadi::Matrix<casadi::SXElem>,nrows,ncols> output = input;

    // copy output_casadi to output
    for(int i=0; i<nr; i++)
    {
        for(int j=0; j<nc; j++)
        {
            output(i,j) = output_casadi(i,j);
        }
    }

    return output;
}

std::string ev2str(Eigen::Matrix<myfloat, -1, 1> vec, int precision){
    // precision of
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision) << vec.transpose();
    return ss.str();
}




#endif // COMMON_DEFINES_HXX
