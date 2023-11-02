#include <iostream>
#include "rigid_body.hxx"


int main(int, char**){
    
    RigidBody body1("body1", 1.0, Vector3(0.0, 0.0, 0.0), Matrix3::Identity(), JointType::FIXED, -1, Vector3(0.0, 0.0, 0.0));
    return 0;
}
