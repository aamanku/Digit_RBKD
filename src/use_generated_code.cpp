//
// Created by abhijee on 2/9/24.
//
#include <iostream>
#include <Eigen/Dense>
#include "digit_rbkd.h"
#include <chrono>

int main()
{
    Eigen::Matrix<casadi_real,37,1> qpos; qpos.setZero();
    Eigen::Matrix<casadi_real,3,3> R; R.setZero();
    Eigen::Matrix<casadi_real,3,1> p; p.setZero();
    Eigen::Matrix<casadi_real,36,36> H; H.setZero();

    int N = 10000;

    {
        auto start = std::chrono::high_resolution_clock::now();
        for(int i=0; i<N; i++) {
//            qpos.setRandom();
            forward_kinematics_eigen(qpos, R, p);
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::cout<<"forward_kinematics_eigen: "<<std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()/N<<" ns"<<std::endl;
    }

    {
        auto start = std::chrono::high_resolution_clock::now();
        for(int i=0; i<N; i++) {
//            qpos.setRandom();
            joint_space_inertia_matrix_eigen(qpos, H);
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::cout<<"joint_space_inertia_matrix_eigen: "<<std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()/N<<" ns"<<std::endl;
    }

    std::cout<<"R: "<<R<<std::endl;
    std::cout<<"p: "<<p<<std::endl;
    std::cout<<"H: "<<H<<std::endl;
    return 0;
}
