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
#include <iomanip>

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
    right_elbow_id,
    total_bodies
};

enum DigitSiteIdx : myint {
    imu_id=0,
    left_foot_id,
    right_foot_id,
    left_hand_id,
    right_hand_id,
    total_sites
};

class DigitModel: public RigidBodyTree {
public:
    DigitModel();
    };


#endif // DIGIT_MODEL_HXX