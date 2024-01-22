#include <Eigen/Dense>
#include <iostream>
#include <cppad/cg.hpp>

namespace Eigen{
//    template<typename S, typename BinOp>
//    struct ScalarBinaryOpTraits<CppAD::AD<S>,S,BinOp>
//    { typedef CppAD::AD<S> ReturnType; };
//    template<typename S, typename BinOp>
//    struct ScalarBinaryOpTraits<S,CppAD::AD<S>,BinOp>
//    { typedef CppAD::AD<S> ReturnType; };
    // for operator +
    template<typename S>
    using CGD = CppAD::cg::CG<S>;

    template<typename S>
    using ADCG = CppAD::AD<CGD<S>>;


    // addition between Scalar and ADCG
    template<typename S>
    struct ScalarBinaryOpTraits<S,ADCG<S>,internal::scalar_sum_op<S,ADCG<S>>>
    { typedef ADCG<S> ReturnType; };
    template<typename S>
    struct ScalarBinaryOpTraits<ADCG<S>,S,internal::scalar_sum_op<ADCG<S>,S>>
    { typedef ADCG<S> ReturnType; };
    // subtraction between Scalar and ADCG
    template<typename S>
    struct ScalarBinaryOpTraits<S,ADCG<S>,internal::scalar_difference_op<S,ADCG<S>>>
    { typedef ADCG<S> ReturnType; };
    template<typename S>
    struct ScalarBinaryOpTraits<ADCG<S>,S,internal::scalar_difference_op<ADCG<S>,S>>
    { typedef ADCG<S> ReturnType; };
    // multiplication between Scalar and ADCG
    template<typename S>
    struct ScalarBinaryOpTraits<S,ADCG<S>,internal::scalar_product_op<S,ADCG<S>>>
    { typedef ADCG<S> ReturnType; };
    template<typename S>
    struct ScalarBinaryOpTraits<ADCG<S>,S,internal::scalar_product_op<ADCG<S>,S>>
    { typedef ADCG<S> ReturnType; };
    // division between Scalar and ADCG
    template<typename S>
    struct ScalarBinaryOpTraits<S,ADCG<S>,internal::scalar_quotient_op<S,ADCG<S>>>
    { typedef ADCG<S> ReturnType; };
    template<typename S>
    struct ScalarBinaryOpTraits<ADCG<S>,S,internal::scalar_quotient_op<ADCG<S>,S>>
    { typedef ADCG<S> ReturnType; };

}


int main(void){
    using CGD = CppAD::cg::CG<double>;
    using ADCG = CppAD::AD<CGD>;
    using ad_vector_t = Eigen::Matrix<ADCG,-1,1>;
    using ad_matrix_t = Eigen::Matrix<ADCG,-1,-1>;
    using ad_fun_t = CppAD::ADFun<CGD>;

    ad_vector_t vars(6);

    ad_vector_t x(2);
    ad_matrix_t M(2,2);
    ad_vector_t y(2);

    CppAD::Independent(vars);
    x = vars.head(2);
    M = vars.segment(2,4).reshaped(2,2);

    y = M*M*M*x*5.0;
    ad_fun_t f(vars,y);
//    f.optimize();

    CppAD::cg::ModelCSourceGen<double> cgen(f,"my_model");

    CppAD::cg::ModelLibraryCSourceGen<double> libcgen(cgen);

//    // generate source code
//    CppAD::cg::DynamicModelLibraryProcessor<double> p(libcgen);
//
//    // compile source code
//    CppAD::cg::GccCompiler<double> compiler;
//    std::unique_ptr<CppAD::cg::DynamicLib<double>> dynamicLib = p.createDynamicLibrary(compiler);
//
    // save to files  to generated folder

    CppAD::cg::SaveFilesModelLibraryProcessor<double> p2(libcgen);
    p2.saveSourcesTo("generated");

    // compile source code
    CppAD::cg::GccCompiler<double> compiler;












}