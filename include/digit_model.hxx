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
#ifndef DIGIT_MODEL_HXX
#define DIGIT_MODEL_HXX

#include "rigid_body.hxx"
#include <chrono>

using namespace rbda;

enum DigitBodyIdx : myint {
        world_id=-1,
        base_trans_id,
        base_rot_id,
        left_hip_roll_id,
        left_hip_yaw_id,
        left_hip_pitch_id,
        left_knee_id,
        left_shin_id,
        left_tarsus_id,
        left_heel_spring_id,
        left_toe_A_id,
        left_toe_B_id,
        left_toe_pitch_id,
        left_toe_roll_id,
        left_shoulder_roll_id,
        left_shoulder_pitch_id,
        left_shoulder_yaw_id,
        left_elbow_id,
        right_hip_roll_id,
        right_hip_yaw_id,
        right_hip_pitch_id,
        right_knee_id,
        right_shin_id,
        right_tarsus_id,
        right_heel_spring_id,
        right_toe_A_id,
        right_toe_B_id,
        right_toe_pitch_id,
        right_toe_roll_id,
        right_shoulder_roll_id,
        right_shoulder_pitch_id,
        right_shoulder_yaw_id,
        right_elbow_id
};


class DigitModel: public RigidBodyTree
{
    public:

    // constructors
    DigitModel() : RigidBodyTree(){

        // start constructing the model
        this->num_bodies = -1;
        this->num_sites = 0;

        // create the bodies
        //base_trans
        num_bodies++;
        RigidBody base_trans;
        base_trans.name = "base_trans";
        base_trans.parent = -1;
        base_trans.id = num_bodies; assert(base_trans.id == base_trans_id);
        base_trans.joint.name = "translational";
        base_trans.joint.joint_type = JointType::TRANSLATIONAL; num_q+=3; num_v+=3;
        base_trans.joint.parent_body = base_trans.id;
        base_trans.joint.limits.resize(3,2);
        base_trans.joint.limits << -MYINF, MYINF,
                            -MYINF, MYINF,
                            -MYINF, MYINF;

        base_trans.spI = SpatialInertia(0, Eigen::Matrix<myfloat,3,1>(0,0,0), Eigen::Matrix<myfloat,6,1>(0,0,0,0,0,0));
        base_trans.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 0.0), Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 0.0));

        //base_rot
        num_bodies++;
        RigidBody base_rot;
        base_rot.name = "base_rot";
        base_rot.parent = base_trans.id;
        base_rot.id = num_bodies; assert(base_rot.id == base_rot_id);
        base_rot.joint.name = "spherical"; 
        base_rot.joint.joint_type = JointType::SPHERICAL; num_q+=4; num_v+=3;
        base_rot.joint.parent_body = base_rot.id;
        base_rot.joint.limits.resize(4,2);
        base_rot.joint.limits << -1.0, 1.0,
                            -1.0, 1.0,
                            -1.0, 1.0,
                            -1.0, 1.0;

        base_rot.spI = SpatialInertia(15.028392, Eigen::Matrix<myfloat,3,1>(0.001636612541, 0.0002001180789, 0.2593064529), Eigen::Matrix<myfloat,6,1>(0.3759052548, 0.3441935, 0.09873232746, -8.776732849e-05, 0.008498611229, 6.621611757e-05));
        base_rot.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 0.0), Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 0.0));

        // left-hip-roll
        num_bodies++;
        RigidBody left_hip_roll;
        left_hip_roll.name = "left-hip-roll";
        left_hip_roll.parent = base_rot.id; 
        left_hip_roll.id = num_bodies; assert(left_hip_roll.id == left_hip_roll_id);
        left_hip_roll.joint.name = "left-hip-roll";
        left_hip_roll.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_hip_roll.joint.parent_body = left_hip_roll.id;
        left_hip_roll.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_hip_roll.joint.limits.resize(1,2);
        left_hip_roll.joint.limits = Eigen::Matrix<myfloat,1,2>(-60,60)*M_PI/180.0;
        left_hip_roll.joint.armature = 0.173823936; num_u+=1;
        left_hip_roll.joint.damping = 1.0;
        left_hip_roll.joint.frictionloss = 1.0;
        left_hip_roll.spI = SpatialInertia(0.915088, Eigen::Matrix<myfloat,3,1>(-0.001967, 0.000244, 0.031435), Eigen::Matrix<myfloat,6,1>(0.001017, 0.001148, 0.000766, -3e-06, 1.3e-05, -4e-06));
        left_hip_roll.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(21.49994484, -90, 0), Eigen::Matrix<myfloat,3,1>(-0.001, 0.091, 0));

        // left-hip-yaw
        num_bodies++;
        RigidBody left_hip_yaw;
        left_hip_yaw.name = "left-hip-yaw";
        left_hip_yaw.parent = left_hip_roll.id;
        left_hip_yaw.id = num_bodies; assert(left_hip_yaw.id == left_hip_yaw_id);
        left_hip_yaw.joint.name = "left-hip-yaw";
        left_hip_yaw.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_hip_yaw.joint.parent_body = left_hip_yaw.id;
        left_hip_yaw.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_hip_yaw.joint.limits.resize(1,2);
        left_hip_yaw.joint.limits = Eigen::Matrix<myfloat,1,2>(-40,40)*M_PI/180.0;
        left_hip_yaw.joint.armature = 0.067899975; num_u+=1;
        left_hip_yaw.joint.damping = 1.0;
        left_hip_yaw.joint.frictionloss = 1.0;

        left_hip_yaw.spI = SpatialInertia(0.818753, Eigen::Matrix<myfloat,3,1>(1e-05, -0.001945, 0.042033), Eigen::Matrix<myfloat,6,1>(0.001627, 0.001929, 0.00077, -1e-06, 2e-06, 5.3e-05));
        left_hip_yaw.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, -90.0, 0), Eigen::Matrix<myfloat,3,1>(-0.0505, 0, 0.044));

        // left-hip-pitch
        num_bodies++;
        RigidBody left_hip_pitch;
        left_hip_pitch.name = "left-hip-pitch";
        left_hip_pitch.parent = left_hip_yaw.id;
        left_hip_pitch.id = num_bodies; assert(left_hip_pitch.id == left_hip_pitch_id);
        left_hip_pitch.joint.name = "left-hip-pitch";
        left_hip_pitch.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_hip_pitch.joint.parent_body = left_hip_pitch.id;
        left_hip_pitch.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, -1.0);
        left_hip_pitch.joint.limits.resize(1,2);
        left_hip_pitch.joint.limits = Eigen::Matrix<myfloat,1,2>(-60,90)*M_PI/180.0;
        left_hip_pitch.joint.armature = 0.1204731904; num_u+=1;
        left_hip_pitch.joint.damping = 1.0;
        left_hip_pitch.joint.frictionloss = 0.5;

        left_hip_pitch.spI = SpatialInertia(6.244279, Eigen::Matrix<myfloat,3,1>(0.060537, 0.000521, -0.038857), Eigen::Matrix<myfloat,6,1>(0.011533, 0.033345, 0.033958, -0.000171, 0.000148, 0.000178));
        left_hip_pitch.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(90, 0, 135), Eigen::Matrix<myfloat,3,1>(0, 0.004, 0.068));

        // left-knee
        num_bodies++;
        RigidBody left_knee;
        left_knee.name = "left-knee";
        left_knee.parent = left_hip_pitch.id;
        left_knee.id = num_bodies; assert(left_knee.id == left_knee_id);
        left_knee.joint.name = "left-knee";
        left_knee.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_knee.joint.parent_body = left_knee.id;
        left_knee.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_knee.joint.limits.resize(1,2);
        left_knee.joint.limits = Eigen::Matrix<myfloat,1,2>(-80,58.4)*M_PI/180.0;
        left_knee.joint.armature = 0.1204731904; num_u+=1;
        left_knee.joint.damping = 1.0;
        left_knee.joint.frictionloss = 0.5;

        left_knee.spI = SpatialInertia(1.227077, Eigen::Matrix<myfloat,3,1>(0.045641, 0.042154, 0.001657), Eigen::Matrix<myfloat,6,1>(0.002643, 0.005098, 0.007019, -0.001832, 6.6e-05, 4.5e-05));
        left_knee.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 0, -90), Eigen::Matrix<myfloat,3,1>(0.12, 0, 0.0045));

        // left-shin
        num_bodies++;
        RigidBody left_shin;
        left_shin.name = "left-shin";
        left_shin.parent = left_knee.id;
        left_shin.id = num_bodies; assert(left_shin.id == left_shin_id);
        left_shin.joint.name = "left-shin";
        left_shin.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_shin.joint.parent_body = left_shin.id;
        left_shin.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_shin.joint.limits.resize(1,2);
        left_shin.joint.limits = Eigen::Matrix<myfloat,1,2>(-90,90)*M_PI/180.0;
        left_shin.joint.stiffness = 6000.0;

        left_shin.spI = SpatialInertia(1.073941, Eigen::Matrix<myfloat,3,1>(0.17637939, -0.01361614, 0.00483379), Eigen::Matrix<myfloat,6,1>(0.00439118, 0.02701227, 0.03037536, 0.0007727, 0.00046775, 0.00052522));
        left_shin.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 0, 0), Eigen::Matrix<myfloat,3,1>(0.060677, 0.047406, 0));

        // left-tarsus
        num_bodies++;
        RigidBody left_tarsus;
        left_tarsus.name = "left-tarsus";
        left_tarsus.parent = left_shin.id;
        left_tarsus.id = num_bodies; assert(left_tarsus.id == left_tarsus_id);
        left_tarsus.joint.name = "left-tarsus";
        left_tarsus.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_tarsus.joint.parent_body = left_tarsus.id;
        left_tarsus.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_tarsus.joint.limits.resize(1,2);
        left_tarsus.joint.limits = Eigen::Matrix<myfloat,1,2>(-50.3,71.6)*M_PI/180.0;

        left_tarsus.spI = SpatialInertia(1.493355, Eigen::Matrix<myfloat,3,1>(0.11692845, -0.03641395, 0.00050495), Eigen::Matrix<myfloat,6,1>(1.60070790e-03, 2.14664590e-02, 2.21038310e-02, 1.98205783e-03, 8.97031761e-05, -5.41167493e-06));
        left_tarsus.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 0, 103), Eigen::Matrix<myfloat,3,1>(0.434759, 0.02, 0));

        // left-heel-spring
        num_bodies++;
        RigidBody left_heel_spring;
        left_heel_spring.name = "left-heel-spring";
        left_heel_spring.parent = left_tarsus.id;
        left_heel_spring.id = num_bodies; assert(left_heel_spring.id == left_heel_spring_id);
        left_heel_spring.joint.name = "left-heel-spring";
        left_heel_spring.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_heel_spring.joint.parent_body = left_heel_spring.id;
        left_heel_spring.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_heel_spring.joint.limits.resize(1,2);
        left_heel_spring.joint.limits = Eigen::Matrix<myfloat,1,2>(-6,6)*M_PI/180.0;
        left_heel_spring.joint.stiffness = 4375.0;

        left_heel_spring.spI = SpatialInertia(0.230018, Eigen::Matrix<myfloat,3,1>(0.049086, 0.004739, -4.5e-05), Eigen::Matrix<myfloat,6,1>(5.5e-05, 0.00074, 0.000701, 1.5e-05, 1e-06, 0));
        left_heel_spring.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(4.470028555, 0.3199786449, 155.799968), Eigen::Matrix<myfloat,3,1>(-0.01766, -0.029456, 0.00104));

        // left-toe-A
        num_bodies++;
        RigidBody left_toe_A;
        left_toe_A.name = "left-toe-A";
        left_toe_A.parent = left_tarsus.id;
        left_toe_A.id = num_bodies; assert(left_toe_A.id == left_toe_A_id);
        left_toe_A.joint.name = "left-toe-A";
        left_toe_A.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_toe_A.joint.parent_body = left_toe_A.id;
        left_toe_A.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_toe_A.joint.limits.resize(1,2);
        left_toe_A.joint.limits = Eigen::Matrix<myfloat,1,2>(-46.2755,44.9815)*M_PI/180.0;
        left_toe_A.joint.armature = 0.036089474999999996; num_u+=1;
        left_toe_A.joint.damping = 1.0;
        left_toe_A.joint.frictionloss = 1.0;

        left_toe_A.spI = SpatialInertia(0.139557, Eigen::Matrix<myfloat,3,1>(0.005161, 1e-06, -0.002248), Eigen::Matrix<myfloat,6,1>(2.9e-05, 5.8e-05, 7.4e-05, 0, -4e-06, 0));
        left_toe_A.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(180, 0, 88.9599851), Eigen::Matrix<myfloat,3,1>(0.059, -0.034, -0.0276));

        // left-toe-B
        num_bodies++;
        RigidBody left_toe_B;
        left_toe_B.name = "left-toe-B";
        left_toe_B.parent = left_tarsus.id;
        left_toe_B.id = num_bodies; assert(left_toe_B.id == left_toe_B_id);
        left_toe_B.joint.name = "left-toe-B";
        left_toe_B.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_toe_B.joint.parent_body = left_toe_B.id;
        left_toe_B.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_toe_B.joint.limits.resize(1,2);
        left_toe_B.joint.limits = Eigen::Matrix<myfloat,1,2>(-45.8918,45.5476)*M_PI/180.0;
        left_toe_B.joint.armature = 0.036089474999999996; num_u+=1;
        left_toe_B.joint.damping = 1.0;
        left_toe_B.joint.frictionloss = 1.0;

        left_toe_B.spI = SpatialInertia(0.139557, Eigen::Matrix<myfloat,3,1>(0.005161, 1e-06, -0.002248), Eigen::Matrix<myfloat,6,1>(2.9e-05, 5.8e-05, 7.4e-05, 0, -4e-06, 0));
        left_toe_B.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 0, -88.9599851), Eigen::Matrix<myfloat,3,1>(0.111, -0.034, 0.0276));

        // left-toe-pitch
        num_bodies++;
        RigidBody left_toe_pitch;
        left_toe_pitch.name = "left-toe-pitch";
        left_toe_pitch.parent = left_tarsus.id;
        left_toe_pitch.id = num_bodies; assert(left_toe_pitch.id == left_toe_pitch_id);
        left_toe_pitch.joint.name = "left-toe-pitch";
        left_toe_pitch.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_toe_pitch.joint.parent_body = left_toe_pitch.id;
        left_toe_pitch.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_toe_pitch.joint.limits.resize(1,2);
        left_toe_pitch.joint.limits = Eigen::Matrix<myfloat,1,2>(-44,34)*M_PI/180.0;

        left_toe_pitch.spI = SpatialInertia(0.043881, Eigen::Matrix<myfloat,3,1>(-0.000141, 2e-06, 3e-06), Eigen::Matrix<myfloat,6,1>(5e-06, 8e-06, 4e-06, 0, 0, 0));
        left_toe_pitch.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 0, 71.75004157), Eigen::Matrix<myfloat,3,1>(0.408, -0.04, 0));

        // left-toe-roll
        num_bodies++;
        RigidBody left_toe_roll;
        left_toe_roll.name = "left-toe-roll";
        left_toe_roll.parent = left_toe_pitch.id;
        left_toe_roll.id = num_bodies; assert(left_toe_roll.id == left_toe_roll_id);
        left_toe_roll.joint.name = "left-toe-roll";
        left_toe_roll.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_toe_roll.joint.parent_body = left_toe_roll.id;
        left_toe_roll.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_toe_roll.joint.limits.resize(1,2);
        left_toe_roll.joint.limits = Eigen::Matrix<myfloat,1,2>(-37,33)*M_PI/180.0;

        left_toe_roll.spI = SpatialInertia(0.531283, Eigen::Matrix<myfloat,3,1>(9e-06, -0.028084, -0.023204), Eigen::Matrix<myfloat,6,1>(0.00187, 0.001616, 0.000843, 0, 0, 0.000566));
        left_toe_roll.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 90, 0), Eigen::Matrix<myfloat,3,1>(0, 0, 0));

        // left-shoulder-roll
        num_bodies++;
        RigidBody left_shoulder_roll;
        left_shoulder_roll.name = "left-shoulder-roll";
        left_shoulder_roll.parent = base_rot.id;
        left_shoulder_roll.id = num_bodies; assert(left_shoulder_roll.id == left_shoulder_roll_id);
        left_shoulder_roll.joint.name = "left-shoulder-roll";
        left_shoulder_roll.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_shoulder_roll.joint.parent_body = left_shoulder_roll.id;
        left_shoulder_roll.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_shoulder_roll.joint.limits.resize(1,2);
        left_shoulder_roll.joint.limits = Eigen::Matrix<myfloat,1,2>(-75,75)*M_PI/180.0;
        left_shoulder_roll.joint.armature = 0.173823936; num_u+=1;
        left_shoulder_roll.joint.damping = 2.0;
        left_shoulder_roll.joint.frictionloss = 2.0;

        left_shoulder_roll.spI = SpatialInertia(0.535396, Eigen::Matrix<myfloat,3,1>(-0.000819, -0.003158, 0.023405), Eigen::Matrix<myfloat,6,1>(0.000704, 0.00075, 0.000298, 1.4e-05, 1.2e-05, 3.5e-05));
        left_shoulder_roll.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(-10.0, -90, 0), Eigen::Matrix<myfloat,3,1>(-0.001, 0.12, 0.4));

        // left-shoulder-pitch
        num_bodies++;
        RigidBody left_shoulder_pitch;
        left_shoulder_pitch.name = "left-shoulder-pitch";
        left_shoulder_pitch.parent = left_shoulder_roll.id;
        left_shoulder_pitch.id = num_bodies; assert(left_shoulder_pitch.id == left_shoulder_pitch_id);
        left_shoulder_pitch.joint.name = "left-shoulder-pitch";
        left_shoulder_pitch.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_shoulder_pitch.joint.parent_body = left_shoulder_pitch.id;
        left_shoulder_pitch.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, -1.0);
        left_shoulder_pitch.joint.limits.resize(1,2);
        left_shoulder_pitch.joint.limits = Eigen::Matrix<myfloat,1,2>(-145,145)*M_PI/180.0;
        left_shoulder_pitch.joint.armature = 0.173823936; num_u+=1;
        left_shoulder_pitch.joint.damping = 2.0;
        left_shoulder_pitch.joint.frictionloss = 2.0;

        left_shoulder_pitch.spI = SpatialInertia(1.440357, Eigen::Matrix<myfloat,3,1>(-4.2e-05, -0.061882, -0.073788), Eigen::Matrix<myfloat,6,1>(0.006761, 0.002087, 0.005778, -6e-06, -3e-06, -0.002046));
        left_shoulder_pitch.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(90, -16.0, -45), Eigen::Matrix<myfloat,3,1>(-0.00317, -0.011055, 0.0555));

        // left-shoulder-yaw
        num_bodies++;
        RigidBody left_shoulder_yaw;
        left_shoulder_yaw.name = "left-shoulder-yaw";
        left_shoulder_yaw.parent = left_shoulder_pitch.id;
        left_shoulder_yaw.id = num_bodies; assert(left_shoulder_yaw.id == left_shoulder_yaw_id);
        left_shoulder_yaw.joint.name = "left-shoulder-yaw";
        left_shoulder_yaw.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_shoulder_yaw.joint.parent_body = left_shoulder_yaw.id;
        left_shoulder_yaw.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_shoulder_yaw.joint.limits.resize(1,2);
        left_shoulder_yaw.joint.limits = Eigen::Matrix<myfloat,1,2>(-100,100)*M_PI/180.0;
        left_shoulder_yaw.joint.armature = 0.067899975; num_u+=1;
        left_shoulder_yaw.joint.damping = 2.0;
        left_shoulder_yaw.joint.frictionloss = 2.0;

        left_shoulder_yaw.spI = SpatialInertia(1.065387, Eigen::Matrix<myfloat,3,1>(-3e-05, 0.001937, 0.11407), Eigen::Matrix<myfloat,6,1>(0.006967, 0.007003, 0.000673, -1e-06, -1e-06, -1.5e-05));
        left_shoulder_yaw.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(90, 0, 0), Eigen::Matrix<myfloat,3,1>(0, -0.165, -0.1));

        // left-elbow
        num_bodies++;
        RigidBody left_elbow;
        left_elbow.name = "left-elbow";
        left_elbow.parent = left_shoulder_yaw.id;
        left_elbow.id = num_bodies; assert(left_elbow.id == left_elbow_id);
        left_elbow.joint.name = "left-elbow";
        left_elbow.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_elbow.joint.parent_body = left_elbow.id;
        left_elbow.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_elbow.joint.limits.resize(1,2);
        left_elbow.joint.limits = Eigen::Matrix<myfloat,1,2>(-77.5,77.5)*M_PI/180.0;
        left_elbow.joint.armature = 0.173823936; num_u+=1;
        left_elbow.joint.damping = 2.0;
        left_elbow.joint.frictionloss = 2.0;

        left_elbow.spI = SpatialInertia(0.550582, Eigen::Matrix<myfloat,3,1>(0.107996, 0.000521, -0.017765), Eigen::Matrix<myfloat,6,1>(0.000476, 0.009564, 0.009437, -2.9e-05, 0.001403, 9e-06));
        left_elbow.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(90, 0, 22.5), Eigen::Matrix<myfloat,3,1>(0, -0.0385, 0.185));

        // right-hip-roll
        num_bodies++;
        RigidBody right_hip_roll;
        right_hip_roll.name = "right-hip-roll";
        right_hip_roll.parent = base_rot.id;
        right_hip_roll.id = num_bodies; assert(right_hip_roll.id == right_hip_roll_id);
        right_hip_roll.joint.name = "right-hip-roll";
        right_hip_roll.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_hip_roll.joint.parent_body = right_hip_roll.id;
        right_hip_roll.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_hip_roll.joint.limits.resize(1,2);
        right_hip_roll.joint.limits = Eigen::Matrix<myfloat,1,2>(-60,60)*M_PI/180.0;
        right_hip_roll.joint.armature = 0.173823936; num_u+=1;
        right_hip_roll.joint.damping = 1.0;
        right_hip_roll.joint.frictionloss = 1.0;

        right_hip_roll.spI = SpatialInertia(0.915088, Eigen::Matrix<myfloat,3,1>(-0.001967, -0.000244, 0.031435), Eigen::Matrix<myfloat,6,1>(0.001017, 0.001148, 0.000766, 3e-06, 1.3e-05, 4e-06));
        right_hip_roll.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(-21.49994484, -90, 0), Eigen::Matrix<myfloat,3,1>(-0.001, -0.091, 0));

        // right-hip-yaw
        num_bodies++;
        RigidBody right_hip_yaw;
        right_hip_yaw.name = "right-hip-yaw";
        right_hip_yaw.parent = right_hip_roll.id;
        right_hip_yaw.id = num_bodies; assert(right_hip_yaw.id == right_hip_yaw_id);
        right_hip_yaw.joint.name = "right-hip-yaw";
        right_hip_yaw.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_hip_yaw.joint.parent_body = right_hip_yaw.id;
        right_hip_yaw.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_hip_yaw.joint.limits.resize(1,2);
        right_hip_yaw.joint.limits = Eigen::Matrix<myfloat,1,2>(-40,40)*M_PI/180.0;
        right_hip_yaw.joint.armature = 0.067899975; num_u+=1;
        right_hip_yaw.joint.damping = 1.0;
        right_hip_yaw.joint.frictionloss = 1.0;

        right_hip_yaw.spI = SpatialInertia(0.818753, Eigen::Matrix<myfloat,3,1>(1e-05, 0.001945, 0.042033), Eigen::Matrix<myfloat,6,1>(0.001627, 0.001929, 0.00077, 1e-06, 2e-06, -5.3e-05));
        right_hip_yaw.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, -90.0, 0), Eigen::Matrix<myfloat,3,1>(-0.0505, 0, 0.044));

        // right-hip-pitch
        num_bodies++;
        RigidBody right_hip_pitch;
        right_hip_pitch.name = "right-hip-pitch";
        right_hip_pitch.parent = right_hip_yaw.id;
        right_hip_pitch.id = num_bodies; assert(right_hip_pitch.id == right_hip_pitch_id);
        right_hip_pitch.joint.name = "right-hip-pitch";
        right_hip_pitch.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_hip_pitch.joint.parent_body = right_hip_pitch.id;
        right_hip_pitch.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, -1.0);
        right_hip_pitch.joint.limits.resize(1,2);
        right_hip_pitch.joint.limits = Eigen::Matrix<myfloat,1,2>(-90,60)*M_PI/180.0;
        right_hip_pitch.joint.armature = 0.1204731904; num_u+=1;
        right_hip_pitch.joint.damping = 1.0;
        right_hip_pitch.joint.frictionloss = 0.5;

        right_hip_pitch.spI = SpatialInertia(6.244279, Eigen::Matrix<myfloat,3,1>(0.060537, -0.000521, -0.038857), Eigen::Matrix<myfloat,6,1>(0.011533, 0.033345, 0.033958, 0.000171, 0.000148, -0.000178));
        right_hip_pitch.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(-90, 0, -135), Eigen::Matrix<myfloat,3,1>(0, -0.004, 0.068));

        // right-knee
        num_bodies++;
        RigidBody right_knee;
        right_knee.name = "right-knee";
        right_knee.parent = right_hip_pitch.id;
        right_knee.id = num_bodies; assert(right_knee.id == right_knee_id);
        right_knee.joint.name = "right-knee";
        right_knee.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_knee.joint.parent_body = right_knee.id;
        right_knee.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_knee.joint.limits.resize(1,2);
        right_knee.joint.limits = Eigen::Matrix<myfloat,1,2>(-58.4,80)*M_PI/180.0;
        right_knee.joint.armature = 0.1204731904; num_u+=1;
        right_knee.joint.damping = 1.0;
        right_knee.joint.frictionloss = 0.5;

        right_knee.spI = SpatialInertia(1.227077, Eigen::Matrix<myfloat,3,1>(0.045641, -0.042154, 0.001657), Eigen::Matrix<myfloat,6,1>(0.002643, 0.005098, 0.007019, 0.001832, 6.6e-05, -4.5e-05));
        right_knee.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 0, 90), Eigen::Matrix<myfloat,3,1>(0.12, 0, 0.0045));

        // right-shin
        num_bodies++;
        RigidBody right_shin;
        right_shin.name = "right-shin";
        right_shin.parent = right_knee.id;
        right_shin.id = num_bodies; assert(right_shin.id == right_shin_id);
        right_shin.joint.name = "right-shin";
        right_shin.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_shin.joint.parent_body = right_shin.id;
        right_shin.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_shin.joint.limits.resize(1,2);
        right_shin.joint.limits = Eigen::Matrix<myfloat,1,2>(-90,90)*M_PI/180.0;    
        right_shin.joint.stiffness = 6000.0;

        right_shin.spI = SpatialInertia(1.073941, Eigen::Matrix<myfloat,3,1>(0.17637939, 0.01361614, 0.00483379), Eigen::Matrix<myfloat,6,1>(0.00439118, 0.02701227, 0.03037536, -0.0007727, 0.00046775, -0.00052522));
        right_shin.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 0, 0), Eigen::Matrix<myfloat,3,1>(0.060677, -0.047406, 0));

        // right-tarsus
        num_bodies++;
        RigidBody right_tarsus;
        right_tarsus.name = "right-tarsus";
        right_tarsus.parent = right_shin.id;
        right_tarsus.id = num_bodies; assert(right_tarsus.id == right_tarsus_id);
        right_tarsus.joint.name = "right-tarsus";
        right_tarsus.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_tarsus.joint.parent_body = right_tarsus.id;
        right_tarsus.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_tarsus.joint.limits.resize(1,2);
        right_tarsus.joint.limits = Eigen::Matrix<myfloat,1,2>(-71.6,50.3)*M_PI/180.0;

        right_tarsus.spI = SpatialInertia(1.493355, Eigen::Matrix<myfloat,3,1>(0.11692845, 0.03641395, 0.00050495), Eigen::Matrix<myfloat,6,1>(1.60070790e-03, 2.14664590e-02, 2.21038310e-02, -1.98205783e-03, 8.97031761e-05, 5.41167493e-06));
        right_tarsus.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 0, -103), Eigen::Matrix<myfloat,3,1>(0.434759, -0.02, 0));

        // right-heel-spring
        num_bodies++;
        RigidBody right_heel_spring;
        right_heel_spring.name = "right-heel-spring";
        right_heel_spring.parent = right_tarsus.id;
        right_heel_spring.id = num_bodies; assert(right_heel_spring.id == right_heel_spring_id);
        right_heel_spring.joint.name = "right-heel-spring";
        right_heel_spring.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_heel_spring.joint.parent_body = right_heel_spring.id;
        right_heel_spring.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_heel_spring.joint.limits.resize(1,2);
        right_heel_spring.joint.limits = Eigen::Matrix<myfloat,1,2>(-6,6)*M_PI/180.0;
        right_heel_spring.joint.stiffness = 4375.0;

        right_heel_spring.spI = SpatialInertia(0.230018, Eigen::Matrix<myfloat,3,1>(0.049086, -0.004739, -4.5e-05), Eigen::Matrix<myfloat,6,1>(5.5e-05, 0.00074, 0.000701, -1.5e-05, 1e-06, 0));
        right_heel_spring.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(-4.470028555, 0.3199786449, -155.799968), Eigen::Matrix<myfloat,3,1>(-0.01766, 0.029456, 0.00104));

        // right-toe-A
        num_bodies++;
        RigidBody right_toe_A;
        right_toe_A.name = "right-toe-A";
        right_toe_A.parent = right_tarsus.id;
        right_toe_A.id = num_bodies; assert(right_toe_A.id == right_toe_A_id);
        right_toe_A.joint.name = "right-toe-A";
        right_toe_A.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_toe_A.joint.parent_body = right_toe_A.id;
        right_toe_A.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_toe_A.joint.limits.resize(1,2);
        right_toe_A.joint.limits = Eigen::Matrix<myfloat,1,2>(-44.9815,46.2755)*M_PI/180.0;
        right_toe_A.joint.armature = 0.036089474999999996; num_u+=1;
        right_toe_A.joint.damping = 1.0;
        right_toe_A.joint.frictionloss = 1.0;

        right_toe_A.spI = SpatialInertia(0.139557, Eigen::Matrix<myfloat,3,1>(0.005161, -1e-06, -0.002248), Eigen::Matrix<myfloat,6,1>(2.9e-05, 5.8e-05, 7.4e-05, 0, -4e-06, 0));
        right_toe_A.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(-180, 0, -88.9599851), Eigen::Matrix<myfloat,3,1>(0.059, 0.034, -0.0276));

        // right-toe-B
        num_bodies++;
        RigidBody right_toe_B;
        right_toe_B.name = "right-toe-B";
        right_toe_B.parent = right_tarsus.id;
        right_toe_B.id = num_bodies; assert(right_toe_B.id == right_toe_B_id);
        right_toe_B.joint.name = "right-toe-B";
        right_toe_B.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_toe_B.joint.parent_body = right_toe_B.id;
        right_toe_B.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_toe_B.joint.limits.resize(1,2);
        right_toe_B.joint.limits = Eigen::Matrix<myfloat,1,2>(-45.5476,45.8918)*M_PI/180.0;
        right_toe_B.joint.armature = 0.036089474999999996; num_u+=1;
        right_toe_B.joint.damping = 1.0;
        right_toe_B.joint.frictionloss = 1.0;

        right_toe_B.spI = SpatialInertia(0.139557, Eigen::Matrix<myfloat,3,1>(0.005161, -1e-06, -0.002248), Eigen::Matrix<myfloat,6,1>(2.9e-05, 5.8e-05, 7.4e-05, 0, -4e-06, 0));
        right_toe_B.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 0, 88.9599851), Eigen::Matrix<myfloat,3,1>(0.111, 0.034, 0.0276));

        // right-toe-pitch
        num_bodies++;
        RigidBody right_toe_pitch;
        right_toe_pitch.name = "right-toe-pitch";
        right_toe_pitch.parent = right_tarsus.id;
        right_toe_pitch.id = num_bodies; assert(right_toe_pitch.id == right_toe_pitch_id);
        right_toe_pitch.joint.name = "right-toe-pitch";
        right_toe_pitch.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_toe_pitch.joint.parent_body = right_toe_pitch.id;
        right_toe_pitch.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_toe_pitch.joint.limits.resize(1,2);
        right_toe_pitch.joint.limits = Eigen::Matrix<myfloat,1,2>(-34,44)*M_PI/180.0;

        right_toe_pitch.spI = SpatialInertia(0.043881, Eigen::Matrix<myfloat,3,1>(-0.000141, -2e-06, 3e-06), Eigen::Matrix<myfloat,6,1>(5e-06, 8e-06, 4e-06, 0, 0, 0));
        right_toe_pitch.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 0, -71.75004157), Eigen::Matrix<myfloat,3,1>(0.408, 0.04, 0));

        // right-toe-roll
        num_bodies++;
        RigidBody right_toe_roll;
        right_toe_roll.name = "right-toe-roll";
        right_toe_roll.parent = right_toe_pitch.id;
        right_toe_roll.id = num_bodies; assert(right_toe_roll.id == right_toe_roll_id);
        right_toe_roll.joint.name = "right-toe-roll";
        right_toe_roll.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_toe_roll.joint.parent_body = right_toe_roll.id;
        right_toe_roll.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_toe_roll.joint.limits.resize(1,2);
        right_toe_roll.joint.limits = Eigen::Matrix<myfloat,1,2>(-33,37)*M_PI/180.0;

        right_toe_roll.spI = SpatialInertia(0.531283, Eigen::Matrix<myfloat,3,1>(9e-06, 0.028084, -0.023204), Eigen::Matrix<myfloat,6,1>(0.00187, 0.001616, 0.000843, 0, 0, -0.000566));
        right_toe_roll.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 90, 0), Eigen::Matrix<myfloat,3,1>(0, 0, 0));

        // right-shoulder-roll
        num_bodies++;
        RigidBody right_shoulder_roll;
        right_shoulder_roll.name = "right-shoulder-roll";
        right_shoulder_roll.parent = base_rot.id;
        right_shoulder_roll.id = num_bodies; assert(right_shoulder_roll.id == right_shoulder_roll_id);
        right_shoulder_roll.joint.name = "right-shoulder-roll";
        right_shoulder_roll.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_shoulder_roll.joint.parent_body = right_shoulder_roll.id;
        right_shoulder_roll.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_shoulder_roll.joint.limits.resize(1,2);
        right_shoulder_roll.joint.limits = Eigen::Matrix<myfloat,1,2>(-75,75)*M_PI/180.0;
        right_shoulder_roll.joint.armature = 0.173823936; num_u+=1;
        right_shoulder_roll.joint.damping = 2.0;
        right_shoulder_roll.joint.frictionloss = 2.0;

        right_shoulder_roll.spI = SpatialInertia(0.535396, Eigen::Matrix<myfloat,3,1>(-0.000819, 0.003158, 0.023405), Eigen::Matrix<myfloat,6,1>(0.000704, 0.00075, 0.000298, -1.4e-05, 1.2e-05, -3.5e-05));
        right_shoulder_roll.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(10.0, -90, 0), Eigen::Matrix<myfloat,3,1>(-0.001, -0.12, 0.4));

        // right-shoulder-pitch
        num_bodies++;
        RigidBody right_shoulder_pitch;
        right_shoulder_pitch.name = "right-shoulder-pitch";
        right_shoulder_pitch.parent = right_shoulder_roll.id;
        right_shoulder_pitch.id = num_bodies; assert(right_shoulder_pitch.id == right_shoulder_pitch_id);
        right_shoulder_pitch.joint.name = "right-shoulder-pitch";
        right_shoulder_pitch.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_shoulder_pitch.joint.parent_body = right_shoulder_pitch.id;
        right_shoulder_pitch.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, -1.0);
        right_shoulder_pitch.joint.limits.resize(1,2);
        right_shoulder_pitch.joint.limits = Eigen::Matrix<myfloat,1,2>(-145,145)*M_PI/180.0;
        right_shoulder_pitch.joint.armature = 0.173823936; num_u+=1;
        right_shoulder_pitch.joint.damping = 2.0;
        right_shoulder_pitch.joint.frictionloss = 2.0;

        right_shoulder_pitch.spI = SpatialInertia(1.440357, Eigen::Matrix<myfloat,3,1>(-4.2e-05, 0.061882, -0.073788), Eigen::Matrix<myfloat,6,1>(0.006761, 0.002087, 0.005778, 6e-06, -3e-06, 0.002046));
        right_shoulder_pitch.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(-90, -16.0, 45), Eigen::Matrix<myfloat,3,1>(-0.00317, 0.011055, 0.0555));

        // right-shoulder-yaw
        num_bodies++;
        RigidBody right_shoulder_yaw;
        right_shoulder_yaw.name = "right-shoulder-yaw";
        right_shoulder_yaw.parent = right_shoulder_pitch.id;
        right_shoulder_yaw.id = num_bodies; assert(right_shoulder_yaw.id == right_shoulder_yaw_id);
        right_shoulder_yaw.joint.name = "right-shoulder-yaw";
        right_shoulder_yaw.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_shoulder_yaw.joint.parent_body = right_shoulder_yaw.id;   
        right_shoulder_yaw.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_shoulder_yaw.joint.limits.resize(1,2);
        right_shoulder_yaw.joint.limits = Eigen::Matrix<myfloat,1,2>(-100,100)*M_PI/180.0;
        right_shoulder_yaw.joint.armature = 0.067899975; num_u+=1;
        right_shoulder_yaw.joint.damping = 2.0;
        right_shoulder_yaw.joint.frictionloss = 2.0;

        right_shoulder_yaw.spI = SpatialInertia(1.065387, Eigen::Matrix<myfloat,3,1>(-3e-05, -0.001937, 0.11407), Eigen::Matrix<myfloat,6,1>(0.006967, 0.007003, 0.000673, 1e-06, -1e-06, 1.5e-05));
        right_shoulder_yaw.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(-90, 0, 0), Eigen::Matrix<myfloat,3,1>(0, 0.165, -0.1));
        
        // right-elbow
        num_bodies++;
        RigidBody right_elbow;
        right_elbow.name = "right-elbow";
        right_elbow.parent = right_shoulder_yaw.id;
        right_elbow.id = num_bodies; assert(right_elbow.id == right_elbow_id);
        right_elbow.joint.name = "right-elbow";
        right_elbow.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_elbow.joint.parent_body = right_elbow.id;
        right_elbow.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_elbow.joint.limits.resize(1,2);
        right_elbow.joint.limits = Eigen::Matrix<myfloat,1,2>(-77.5,77.5)*M_PI/180.0;
        right_elbow.joint.armature = 0.173823936; num_u+=1;
        right_elbow.joint.damping = 2.0;
        right_elbow.joint.frictionloss = 2.0;

        right_elbow.spI = SpatialInertia(0.550582, Eigen::Matrix<myfloat,3,1>(0.107996, -0.000521, -0.017765), Eigen::Matrix<myfloat,6,1>(0.000476, 0.009564, 0.009437, 2.9e-05, 0.001403, -9e-06));
        right_elbow.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(-90, 0, -22.5), Eigen::Matrix<myfloat,3,1>(0, 0.0385, 0.185));

        num_bodies++; // final

        // Add sites
        // // base
        Site imu;
        imu.name = "imu";
        imu.parent_body = base_rot.id;
        imu.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, -90, 0), Eigen::Matrix<myfloat,3,1>(0, 0, 0.0));



        sites.push_back(imu);

        // push all bodies into a vector
        bodies.push_back(base_trans);
        bodies.push_back(base_rot);
        bodies.push_back(left_hip_roll);
        bodies.push_back(left_hip_yaw);
        bodies.push_back(left_hip_pitch);
        bodies.push_back(left_knee);
        bodies.push_back(left_shin);
        bodies.push_back(left_tarsus);
        bodies.push_back(left_heel_spring);
        bodies.push_back(left_toe_A);
        bodies.push_back(left_toe_B);
        bodies.push_back(left_toe_pitch);
        bodies.push_back(left_toe_roll);
        bodies.push_back(left_shoulder_roll);
        bodies.push_back(left_shoulder_pitch);
        bodies.push_back(left_shoulder_yaw);
        bodies.push_back(left_elbow);
        bodies.push_back(right_hip_roll);
        bodies.push_back(right_hip_yaw);
        bodies.push_back(right_hip_pitch);
        bodies.push_back(right_knee);
        bodies.push_back(right_shin);
        bodies.push_back(right_tarsus);
        bodies.push_back(right_heel_spring);
        bodies.push_back(right_toe_A);
        bodies.push_back(right_toe_B);
        bodies.push_back(right_toe_pitch);
        bodies.push_back(right_toe_roll);
        bodies.push_back(right_shoulder_roll);
        bodies.push_back(right_shoulder_pitch);
        bodies.push_back(right_shoulder_yaw);
        bodies.push_back(right_elbow);

        // assert id matches index
        for (int i = 0; i < bodies.size(); i++) {
            if(bodies[i].id != i){
                std::cerr<<"id does not match index"<<std::endl;
                std::cerr<<"id: "<<bodies[i].id<<std::endl;
                std::cerr<<"index: "<<i<<std::endl;
            }
        }

        // add joint states
        for (int i = 0; i < num_bodies; i++) {
            JointType jt = bodies[i].joint.joint_type;
            switch (jt) {
                case JointType::REVOLUTE:
                    bodies[i].joint.q.resize(1); bodies[i].joint.q.setZero();
                    bodies[i].joint.v.resize(1); bodies[i].joint.v.setZero();
                    bodies[i].calculate_Xjtree();
                    break;
                case JointType::TRANSLATIONAL:
                    bodies[i].joint.q.resize(3); bodies[i].joint.q.setZero();
                    bodies[i].joint.v.resize(3); bodies[i].joint.v.setZero();
                    bodies[i].calculate_Xjtree();
                    break;
                case JointType::SPHERICAL:
                    bodies[i].joint.q.resize(4); bodies[i].joint.q.setZero(); bodies[i].joint.q(0) = 1.0;
                    bodies[i].joint.v.resize(3); bodies[i].joint.v.setZero();
                    bodies[i].calculate_Xjtree();
                    break;
                default:
                    std::cerr<<"joint type not implemented"<<std::endl;
                    assert(false);
                    break;
            }
            // std::cout << bodies[i]<<std::endl;
        }

        // tests
        std::cout<<"num_bodies: "<<num_bodies<<std::endl;
        std::cout<<"bodies.size(): "<<bodies.size()<<std::endl;
        std::cout<<"num_q: "<<num_q<<std::endl;
        std::cout<<"num_v: "<<num_v<<std::endl;
        std::cout<<"num_u: "<<num_u<<std::endl;
        
        std::cout<<"FwdKinematics"<<std::endl;
        bodies[0].set_pos(Eigen::Matrix<myfloat,3,1>(0,0,100));
        bodies[1].set_pos(Eigen::Matrix<myfloat,4,1>(0.9387913,0.2397128, 0.2397128, 0.0612087));
        bodies[1].set_vel(Eigen::Matrix<myfloat,3,1>(1,0,0));
        bodies[6].set_vel(Eigen::Matrix<myfloat,1,1>(1.1));
        // bodies[left_hip_pitch_id].set_pos(Eigen::Matrix<myfloat,1,1>(0.1));
        // std::cout<<this->forward_kinematics( Pose(Eigen::Matrix<myfloat,3,1>({-60,0,-90}),Eigen::Matrix<myfloat,3,1>({0,-0.05456,-0.0315})),left_toe_roll_id);

        // std::cout<<"Body Jacobian"<<std::endl;
        // std::cout<<this->spatial_body_jacobian(right_hip_yaw_id)<<std::endl;
        
        // std::cout<<this->spatial_body_corriolis(left_toe_pitch_id)<<std::endl;


        // std::cout<< this->joint_space_inertia_matrix()<<std::endl;
        std::cout<< this->com_position() << std::endl;
        std::cout<<this->centroidal_momentum_corriolis();
        // time inertia matrix in microseconds
        auto start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < 1000; i++) {
            // this->joint_space_inertia_matrix();
            // this->joint_space_nonlinear_effects();
            this->centroidal_momentum_corriolis();
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::cout<<"time: "<<std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()/1000.0<<std::endl;
        this->print_state();

    }

    void print_state() {
        std::cout<<"q: "<<std::endl;
        for (int i = 0; i < bodies.size(); i++) {
            std::cout<<bodies[i].joint.q<<std::endl;
        }
        std::cout<<"v: "<<std::endl;
        for (int i = 0; i < bodies.size(); i++) {
            std::cout<<bodies[i].joint.v<<std::endl;
        }
    }

/**
 * @brief Given a string of the form "a b c", returns a vector of the form [a, b, c] where a, b, c are floats
 * 
 * @param str Input string
 * @return Eigen::Matrix<myfloat, -1, 1> Output vector
 */
Eigen::Matrix<myfloat, -1, 1> str2vec(std::string str){
    std::stringstream ss(str);
    Eigen::Matrix<myfloat, -1, 1> vec;
    myfloat val;
    myint i = 0;
    while(ss >> val){
        // add dimension to the vector
        vec.conservativeResize(i+1);
        vec(i) = val;
        i++;
    }
    return vec;
}

/**
 * @brief myfloat from string
 * 
 * @param std::string
 * @return myfloat 
 */
myfloat str2f(std::string str){
    std::stringstream ss(str);
    myfloat val;
    ss >> val;
    return val;
}
};
#endif // DIGIT_MODEL_HXX
