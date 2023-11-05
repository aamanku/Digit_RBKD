
#ifndef RIGID_BODY_HPP
#define RIGID_BODY_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <string>
#include "common_defines.hxx"
#include "rbda/rbda.hxx"

using namespace rbda;


struct Site{
        std::string name;
        myint parent_body; // index of the parent body
        PluckerTransform Xtree; // transform from parent body to site
};

struct Joint {
        std::string name;
        myint parent_body; // index of the parent body
        JointType joint_type;
        Eigen::Matrix<myfloat,3,1> joint_axis;
        myfloat damping = 0.0;
        myfloat frictionloss = 0.0;
        myfloat stiffness = 0.0;
        Eigen::Matrix<myfloat,-1,2> limits; // lower and upper limits

        myfloat gear_ratio = 0.0; // gear ratio of the motor. gear_ratio = 0.0 means no motor
        myfloat armature = 0.0; // armature of the motor
        Eigen::Matrix<myfloat,1,2> motor_limits = Eigen::Matrix<myfloat,1,2>::Zero(); // lower and upper limits of the motor

        // operators << 
        friend std::ostream &operator<<(std::ostream &os, const Joint &joint)
        {
                os << "name: " << joint.name << std::endl;
                os << "parent_body: " << joint.parent_body << std::endl;
                os << "joint_type: " << (int)joint.joint_type << std::endl;
                os << "joint_axis: " << joint.joint_axis.transpose() << std::endl;
                os << "damping: " << joint.damping << std::endl;
                os << "frictionloss: " << joint.frictionloss << std::endl;
                os << "stiffness: " << joint.stiffness << std::endl;
                os << "limits: " << joint.limits << std::endl;
                os << "gear_ratio: " << joint.gear_ratio << std::endl;
                os << "armature: " << joint.armature << std::endl;
                os << "motor_limits: " << joint.motor_limits << std::endl;

                return os;
        }

};

struct RigidBody {
        std::string name;
        myint parent;
        myint id;
        Joint joint;
        PluckerTransform Xtree;
        SpatialInertia spI;
};

class RigidBodyTree {
        public:
        myint num_bodies;
        std::vector<RigidBody> bodies;
        std::vector<Site> sites;
        Eigen::Matrix<myfloat,3,1> gravity = Eigen::Matrix<myfloat,3,1> (0.0, 0.0, -9.81);
};

// struct RigidBody {
//     std::string name; // name of the body
//     myfloat mass; // mass of the body
//     Vector3 com_pos; // center of mass position w.r.t. body frame
//     Matrix3 inertia; // inertia matrix w.r.t. body frame
//     JointType joint_type; // joint type
//     myint parent; // parent body index
//     Vector3 joint_axis; // joint axis w.r.t. body frame
//     myfloat armature; // armature of the joint

//     // constructors
//     RigidBody(  std::string name,
//                 myfloat mass,
//                 Vector3 com_pos, 
//                 Matrix3 inertia, 
//                 JointType joint_type, 
//                 myint parent, 
//                 Vector3 joint_axis,
//                 myfloat armature = 0.0
//                 ) : name(name), mass(mass), com_pos(com_pos), inertia(inertia), joint_type(joint_type), parent(parent), joint_axis(joint_axis), armature(armature) 
//                 {
//         PRINT("RigidBody");
//                 }
        
// };



#endif // RIGID_BODY_HPP
