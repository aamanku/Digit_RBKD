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
#include <iostream>
#include "digit_rbkd/digit_model_library.hpp"
#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include "digit_rbkd/CasadiEigenCodeGenerator.hpp"
#include <vector>


void add_to_code_generation(std::string name, std::vector<Eigen::MatrixX<myfloat>> func_vec, std::vector<casadi::SX> arg_vec, CasadiEigenCodeGen::FixedSizeCG& cg){
    std::vector<casadi::SX> func_vec_cs;
    for(int i=0; i<func_vec.size(); i++){
        casadi::SX func_cs;
        casadi::copy(func_vec.at(i), func_cs);
        func_vec_cs.push_back(func_cs);
    }
    casadi::Function f(name, arg_vec, func_vec_cs);
    cg.add_casadi_function(f);
}

int main(int, char**){
    DigitModel digit = DigitModel();

    // create casadi symbolic variables
    casadi::SX qpos_cs = casadi::SX::sym("qpos", digit.num_q);
    casadi::SX qvel_cs = casadi::SX::sym("qvel", digit.num_v);

    // create Eigen variables
    Eigen::Matrix<myfloat,-1,1> qpos(digit.num_q);
    Eigen::Matrix<myfloat,-1,1> qvel(digit.num_v);

    // copy casadi symbolic variables to Eigen variables
    casadi::copy(qpos_cs, qpos);
    casadi::copy(qvel_cs, qvel);

    // set qpos and qvel
    digit.set_qpos(qpos);
    digit.set_qvel(qvel);

    // print state
    digit.print_state();
    // codegen
    casadi::Dict opts;
    opts["with_header"] = true;
    opts["cpp"] = true;
    CasadiEigenCodeGen::FixedSizeCG cg("digit_rbkd", SOURCE_DIR"/codegen", opts);

    // creating functions
    auto pose = digit.forward_kinematics(DigitSiteIdx::left_foot_id, DigitBodyIdx::world_id);
    auto H = digit.joint_space_inertia_matrix();



    add_to_code_generation("forward_kinematics", {pose.R,pose.p}, {qpos_cs}, cg);
    add_to_code_generation("joint_space_inertia_matrix", {H}, {qpos_cs}, cg);

    cg.generate_with_eigen();




    return 0;
}
