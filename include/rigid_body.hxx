
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
        
        Eigen::Matrix<myfloat,-1,1> q; // joint position
        Eigen::Matrix<myfloat,-1,1> v; // joint velocity

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
                os << "q: " << joint.q.transpose() << std::endl;
                os << "v: " << joint.v.transpose() << std::endl;
                return os;
        }

};



struct RigidBody {
        std::string name;
        myint parent;
        myint id;
        Joint joint;
        PluckerTransform Xtree;
        PluckerTransform Xtreej;
        SpatialInertia spI;

        /**
         * @brief Returns X transform matrix after joint position q is applied
         * 
         */
        void calculate_Xtreej()
        {
                switch (joint.joint_type)
                {
                        case JointType::REVOLUTE:
                        {       
                                const Eigen::Matrix<myfloat,3,3> Ej = Eigen::AngleAxis<myfloat>(-joint.q(0), joint.joint_axis).toRotationMatrix(); // -q(0) for transposed rotation
                                this->Xtreej.E = this->Xtree.E*Ej;
                                this->Xtreej.r = Ej.transpose()*this->Xtree.r;
                                break;
                        }
                        case JointType::TRANSLATIONAL:
                        {
                                this->Xtreej.E = this->Xtree.E;
                                this->Xtreej.r = this->Xtree.r - this->Xtree.E*joint.q.block<3,1>(0,0);
                                break;
                        }
                        case JointType::SPHERICAL:
                        {
                                const Eigen::Matrix<myfloat,3,3> Ej = Eigen::Quaternion<myfloat>(-joint.q(0), joint.q(1), joint.q(2), joint.q(3)).toRotationMatrix(); // last 4 q are quaternion [w,x,y,z] - q(0) for transposed rotation
                                this->Xtreej.E = this->Xtree.E*Ej;
                                this->Xtreej.r = Ej.transpose()*this->Xtree.r;
                                break;
                        }
                        default:
                                std::cerr<<"Joint type not implemented"<<std::endl;
                                assert(false);
                                break;
                }      
        }

        void set_state(Eigen::Matrix<myfloat,-1,1> q, Eigen::Matrix<myfloat,-1,1> v)
        {
                this->joint.q = q;
                this->joint.v = v;
                this->calculate_Xtreej();
        }

        void set_pos(Eigen::Matrix<myfloat,-1,1> q)
        {
                this->joint.q = q;
                this->calculate_Xtreej();
        }

        void set_vel(Eigen::Matrix<myfloat,-1,1> v)
        {
                this->joint.v = v;
        }

        //operator <<
        friend std::ostream &operator<<(std::ostream &os, const RigidBody &rb)
        {
                os << "name: " << rb.name << std::endl;
                os << "parent: " << rb.parent << std::endl;
                os << "id: " << rb.id << std::endl;
                os << "joint: " << rb.joint << std::endl;
                os << "Xtree: " << rb.Xtree << std::endl;
                os << "spI: " << rb.spI << std::endl;
                return os;
        }
};

struct Pose {
        Eigen::Matrix<myfloat,3,3> R;
        Eigen::Matrix<myfloat,3,1> p;

        // constructors
        Pose() : R(Eigen::Matrix<myfloat,3,3>::Identity()), p(Eigen::Matrix<myfloat,3,1>::Zero()){};
        Pose(Eigen::Matrix<myfloat,3,3> R, Eigen::Matrix<myfloat,3,1> p) : R(R), p(p){};

        // set to identity
        void setIdentity()
        {
                R = Eigen::Matrix<myfloat,3,3>::Identity();
                p = Eigen::Matrix<myfloat,3,1>::Zero();
        };

        // operators <<
        friend std::ostream &operator<<(std::ostream &os, const Pose &pose)
        {
                os << "R: " << std::endl;
                os << pose.R << std::endl;
                os << "p: " << std::endl;
                os << pose.p << std::endl;
                return os;
        }
};

class RigidBodyTree {
        public:
        myint num_bodies;
        std::vector<RigidBody> bodies;
        std::vector<Site> sites;
        Eigen::Matrix<myfloat,3,1> gravity = Eigen::Matrix<myfloat,3,1> (0.0, 0.0, -9.81);

        // methods
        Pose forward_kinematics(myint body_id, Pose frame)
        {
                std::cout<<bodies[body_id].name<<std::endl;
                std::cout<<"Xtree: "<<std::endl;
                std::cout<<bodies[body_id].Xtree<<std::endl;
                PluckerTransform Xup = bodies[body_id].Xtree;
                // go up the tree
                while(bodies[body_id].parent != -1){
                        body_id = bodies[body_id].parent;
                        Xup = Xup*bodies[body_id].Xtree;
                        std::cout<<bodies[body_id].name<<std::endl;
                        std::cout<<"Xtree: "<<std::endl;
                        std::cout<<bodies[body_id].Xtree<<std::endl;
                        std::cout<<"Xup: "<<std::endl;
                        std::cout<<Xup<<std::endl;
                }
                // return the pose
                Pose pose;
                pose.R = Xup.E*frame.R;
                pose.p = Xup.E*frame.p + Xup.r;
                return pose;
        }
};




#endif // RIGID_BODY_HPP
