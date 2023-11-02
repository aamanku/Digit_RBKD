
#ifndef RIGID_BODY_HPP
#define RIGID_BODY_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <string>
#include "common_defines.hxx"

struct RigidBody {
    std::string name; // name of the body
    myfloat mass; // mass of the body
    Vector3 com_pos; // center of mass position w.r.t. body frame
    Matrix3 inertia; // inertia matrix w.r.t. body frame
    JointType joint_type; // joint type
    myint parent; // parent body index
    Vector3 joint_axis; // joint axis w.r.t. body frame
    myfloat armature; // armature of the joint

    // constructors
    RigidBody(  std::string name,
                myfloat mass,
                Vector3 com_pos, 
                Matrix3 inertia, 
                JointType joint_type, 
                myint parent, 
                Vector3 joint_axis,
                myfloat armature = 0.0
                ) : name(name), mass(mass), com_pos(com_pos), inertia(inertia), joint_type(joint_type), parent(parent), joint_axis(joint_axis), armature(armature) 
                {
        PRINT("RigidBody");
                }
        
};

#endif // RIGID_BODY_HPP
