//https://github.com/stack-of-tasks/pinocchio/issues/809
#include <casadi/casadi.hpp>
#include <Eigen/Dense>
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
using scalar = casadi::Matrix<casadi::SXElem>;
//template<typename scalar>
Eigen::Matrix<scalar,-1,1> someFunc(Eigen::Matrix<scalar,3,3> A, Eigen::Matrix<scalar,3,1> b){
    return A*b;
}


int main(){
    // Define a casadi SX symbol

    Eigen::Matrix<scalar, 3, 1> b;
    Eigen::Matrix<scalar,3,3> A;
    casadi::SX csA = casadi::SX::sym("A",3,3);
    casadi::SX csb = casadi::SX::sym("b",3,1);
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            A(i,j) = csA(i,j);
        }
        b(i) = csb(i);
    }
    b[2] = 69.5;

    std::cout<<someFunc(A,b)<<std::endl;


}
//template <int nrows, int ncols>
//class ESX : public Eigen::Matrix<casadi::SX, nrows, ncols>
//{
//public:
//    casadi::SX sym;
//    ESX(std::string var) : Eigen::Matrix<casadi::SX, nrows, ncols>(){
//        // create a matrix of casadi SX symbols
//        sym = casadi::SX::sym(var, nrows, ncols);
//
//        for(int i=0; i<nrows; i++){
//            for(int j=0; j<ncols; j++){
//                this->operator()(i,j) = sym(i,j);
//            }
//        }
//    }
//    ESX() : Eigen::Matrix<casadi::SX, nrows, ncols>(){}
//
//};
//
//#define OVERLOAD_MATH_FUNCS_FOR_ESX(OP) \
//    template <int nrows, int ncols>\
//    ESX<nrows,ncols> OP(const ESX<nrows,ncols>& A){\
//        ESX<nrows,ncols> B;\
//        for(int i=0; i<nrows; i++){\
//            for(int j=0; j<ncols; j++){\
//                B(i,j) = OP(A(i,j));\
//            }\
//        }\
//        return B;\
//    }\
//
//OVERLOAD_MATH_FUNCS_FOR_ESX(sin)
//OVERLOAD_MATH_FUNCS_FOR_ESX(cos)
//OVERLOAD_MATH_FUNCS_FOR_ESX(tan)
//OVERLOAD_MATH_FUNCS_FOR_ESX(asin)
//OVERLOAD_MATH_FUNCS_FOR_ESX(acos)
//OVERLOAD_MATH_FUNCS_FOR_ESX(atan)
//OVERLOAD_MATH_FUNCS_FOR_ESX(sinh)
//OVERLOAD_MATH_FUNCS_FOR_ESX(cosh)
//OVERLOAD_MATH_FUNCS_FOR_ESX(tanh)
//OVERLOAD_MATH_FUNCS_FOR_ESX(asinh)
//OVERLOAD_MATH_FUNCS_FOR_ESX(acosh)
//OVERLOAD_MATH_FUNCS_FOR_ESX(atanh)
//OVERLOAD_MATH_FUNCS_FOR_ESX(exp)
//OVERLOAD_MATH_FUNCS_FOR_ESX(log)
//OVERLOAD_MATH_FUNCS_FOR_ESX(log10)
//OVERLOAD_MATH_FUNCS_FOR_ESX(sqrt)
//OVERLOAD_MATH_FUNCS_FOR_ESX(abs)
//OVERLOAD_MATH_FUNCS_FOR_ESX(ceil)
//OVERLOAD_MATH_FUNCS_FOR_ESX(floor)
//OVERLOAD_MATH_FUNCS_FOR_ESX(round)
//
//
//int main(){
//
//
////    Eigen::Matrix<myfloat, 3, 1> x;
////    Eigen::Matrix<myfloat,3,3> A;
////    Eigen::Matrix<myfloat,3,1> b = A*x;
//    ESX<3,3> b( "b");
//    std::cout<<log(exp(b))<<std::endl;
//    std::cout<<b<<std::endl;
//    std::cout<<b.sym<<std::endl;
//
//    return 0;
//}