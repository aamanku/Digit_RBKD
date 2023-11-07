
#ifndef RIGID_BODY_HPP
#define RIGID_BODY_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <string>
#include "common_defines.hxx"
#include "rbda/rbda.hxx"
#include <numeric>
using namespace rbda;

struct Site
{
        std::string name;
        myint parent_body;      // index of the parent body
        PluckerTransform Xtree; // transform from parent body to site
};

struct Joint
{
        std::string name;
        myint parent_body; // index of the parent body
        JointType joint_type;
        Eigen::Matrix<myfloat, 3, 1> joint_axis;
        myfloat damping = 0.0;
        myfloat frictionloss = 0.0;
        myfloat stiffness = 0.0;
        Eigen::Matrix<myfloat, -1, 2> limits; // lower and upper limits

        myfloat gear_ratio = 0.0;                                                         // gear ratio of the motor. gear_ratio = 0.0 means no motor
        myfloat armature = 0.0;                                                           // armature of the motor
        Eigen::Matrix<myfloat, 1, 2> motor_limits = Eigen::Matrix<myfloat, 1, 2>::Zero(); // lower and upper limits of the motor

        Eigen::Matrix<myfloat, -1, 1> q; // joint position
        Eigen::Matrix<myfloat, -1, 1> v; // joint velocity

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

struct RigidBody
{
        std::string name;
        myint parent;
        myint id;
        Joint joint;
        PluckerTransform Xtree;
        PluckerTransform Xjtree;
        SpatialInertia spI;

        /**
         * @brief Returns X transform matrix after joint position q is applied
         *
         */
        void calculate_Xjtree()
        {
                switch (joint.joint_type)
                {
                case JointType::REVOLUTE:
                {
                        const Eigen::Matrix<myfloat, 3, 3> Ej = Eigen::AngleAxis<myfloat>(-joint.q(0), joint.joint_axis).toRotationMatrix(); // -q(0) for transposed rotation
                        this->Xjtree.E = Ej * this->Xtree.E;
                        this->Xjtree.r = this->Xtree.r;
                        break;
                }
                case JointType::TRANSLATIONAL:
                {
                        this->Xjtree.E = this->Xtree.E;
                        this->Xjtree.r = this->Xtree.r + this->Xtree.E * joint.q.block<3, 1>(0, 0);
                        break;
                }
                case JointType::SPHERICAL:
                {
                        const Eigen::Matrix<myfloat, 3, 3> Ej = Eigen::Quaternion<myfloat>(-joint.q(0), joint.q(1), joint.q(2), joint.q(3)).toRotationMatrix(); // last 4 q are quaternion [w,x,y,z] - q(0) for transposed rotation
                        std::cout<<"Ej: "<<std::endl<<Ej<<std::endl;
                        std::cin.get();
                        this->Xjtree.E = Ej * this->Xtree.E;
                        this->Xjtree.r = this->Xtree.r;
                        break;
                }
                default:
                        std::cerr << "Joint type not implemented" << std::endl;
                        assert(false);
                        break;
                }
        }

        void set_state(Eigen::Matrix<myfloat, -1, 1> q, Eigen::Matrix<myfloat, -1, 1> v)
        {
                this->joint.q = q;
                this->joint.v = v;
                this->calculate_Xjtree();
        }

        void set_pos(Eigen::Matrix<myfloat, -1, 1> q)
        {
                this->joint.q = q;
                this->calculate_Xjtree();
        }

        void set_vel(Eigen::Matrix<myfloat, -1, 1> v)
        {
                this->joint.v = v;
        }

        // operator <<
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

class RigidBodyTree
{
public:
        myint num_bodies=-1;
        myint num_sites=0;
        myint num_q=0;
        myint num_v=0;
        myint num_u=0;
        std::vector<RigidBody> bodies;
        std::vector<Site> sites;
        Eigen::Matrix<myfloat, 3, 1> gravity = Eigen::Matrix<myfloat, 3, 1>(0.0, 0.0, -9.81);

        // methods
        /**
         * @brief Get vector of ids of all bodies between body_id and parent_body_id
         * 
         * @param body_id 
         * @param parent_body_id 
         * @return std::vector<myint> 
         */
        inline std::vector<myint> kappa(myint body_id, myint parent_body_id = -1)
        {
                std::vector<myint> kappa;
                kappa.push_back(body_id);
                // go up the tree
                while (bodies[body_id].parent != parent_body_id)
                {
                        body_id = bodies[body_id].parent;
                        kappa.push_back(body_id);
                }
                // reverse the vector
                std::reverse(kappa.begin(), kappa.end());
                // print kappa
                std::cout << "kappa: ";
                for (myint i = 0; i < kappa.size(); i++)
                {
                        std::cout << kappa.at(i) << " ";
                }
                std::cin.get();
                // return the pose
                return kappa;
        }

        inline std::vector<myint> branch_dof(std::vector<myint> kappa)
        {
                std::vector<myint> branch_dof;
                for (myint i = 0; i < kappa.size(); i++)
                {
                        switch (bodies[kappa.at(i)].joint.joint_type)
                        {
                                case JointType::REVOLUTE:
                                        branch_dof.push_back(1);
                                        break;
                                case JointType::TRANSLATIONAL:
                                        branch_dof.push_back(3);
                                        break;
                                case JointType::SPHERICAL:
                                        branch_dof.push_back(3);
                                        break;
                                default:
                                        std::cerr << "Joint type not implemented" << std::endl;
                                        assert(false);
                                        break;
                        }
                }
                std::cout << "branch_dof: ";
                for (myint i = 0; i < branch_dof.size(); i++)
                {
                        std::cout << branch_dof.at(i) << " ";
                }
                return branch_dof;
        }


        PluckerTransform spatial_body_forward_kinematics(myint body_id, myint parent_body_id = -1)
        {
                PluckerTransform Xup = bodies[body_id].Xjtree;
                // go up the tree
                while (bodies[body_id].parent != parent_body_id)
                {
                        body_id = bodies[body_id].parent;
                        Xup = Xup * bodies[body_id].Xjtree;
                }
                // return the pose
                return Xup;
        }

        Eigen::Matrix<myfloat, 6, -1> spatial_body_jacobian(myint body_id, myint parent_body_id = -1)
        {
                Eigen::Matrix<myfloat, 6, -1> Jb;
                std::vector<myint> branch = this->kappa(body_id, parent_body_id);
                std::vector<myint> branch_dof = this->branch_dof(branch);
                myint dof = std::accumulate(branch_dof.begin(), branch_dof.end(), 0);
                std::cout<<"dof: "<<dof<<std::endl;
                // resize the jacobian
                Jb.resize(6, dof);

                myint id = branch.at(0);
                myint num_col = 0;
                PluckerTransform iXo = bodies[id].Xjtree;
                Eigen::Matrix<myfloat,6,-1> Sj = motion_subspace_matrix(bodies[id].joint.joint_type, bodies[id].joint.joint_axis);
                Jb.block(0,num_col,6,Sj.cols()) << (iXo.inverse().toMatrix()*Sj);
                num_col += Sj.cols();

                // go down the tree
                for (myint i = 1 ; i < branch.size(); i++)
                {
                        std::cout<<"body_name: "<<bodies[branch.at(i-1)].name<<std::endl;
                        std::cout<<"iXo: "<<std::endl<<iXo.toMatrix()<<std::endl;
                        std::cout<<"iXo_inv: "<<std::endl<<iXo.inverse().toMatrix()<<std::endl;
                        std::cout<<"Sj: "<<std::endl<<Sj<<std::endl;
                        std::cin.get();
                        id = branch.at(i);
                        iXo = bodies[id].Xjtree*iXo;
                        Sj = motion_subspace_matrix(bodies[id].joint.joint_type, bodies[id].joint.joint_axis);
                        Jb.block(0,num_col,6,Sj.cols()) << iXo.inverse().toMatrix()*Sj;
                        num_col += Sj.cols();

                }      
                std::cout<<"body_name: "<<bodies[branch.at(branch.size()-1)].name<<std::endl;
                        std::cout<<"iXo: "<<std::endl<<iXo.toMatrix()<<std::endl;
                        std::cout<<"iXo_inv: "<<std::endl<<iXo.inverse().toMatrix()<<std::endl;
                        std::cout<<"Sj: "<<std::endl<<Sj<<std::endl;
                        std::cin.get();          

                // return the jacobian 
                return Jb;
        }

        Pose forward_kinematics(Pose frame, myint body_id, myint parent_body_id = -1)
        {
                PluckerTransform Xup = spatial_body_forward_kinematics(body_id, parent_body_id);
                PluckerTransform Xframe = PluckerTransform(frame) * Xup;
                // // return the pose
                return Xframe.toPose();
        }

        // methods
};

#endif // RIGID_BODY_HPP
