#include <iostream>
#include <Eigen/Dense>
#include <casadi/casadi.hpp>

#include "digit_rbkd/digit_model.hxx"

int main(){
    DigitModel model;

    // create casadi symbolic variables
    casadi::SX qpos_cs = casadi::SX::sym("qpos", model.num_q);
    casadi::SX qvel_cs = casadi::SX::sym("qvel", model.num_v);

    // create Eigen variables
    Eigen::Matrix<myfloat,-1,1> qpos(model.num_q);
    Eigen::Matrix<myfloat,-1,1> qvel(model.num_v);

    // copy casadi symbolic variables to Eigen variables
    casadi::copy(qpos_cs, qpos);
    casadi::copy(qvel_cs, qvel);

    // set qpos and qvel
    model.set_qpos(qpos);
    model.set_qvel(qvel);

    // print state
    model.print_state();

    casadi::SX com_expr;
    auto com = model.com_position();
    casadi::copy(com, com_expr);

    casadi::Function com_func("com_func", {qpos_cs}, {com_expr});

    com_func.generate("com_func");

    casadi::SX H_expr;
    auto H = model.joint_space_inertia_matrix();
    casadi::copy(H, H_expr);

    casadi::Function H_func("H_func", {qpos_cs}, {H_expr});
    H_func.generate("H_func");

    return 0;
}
