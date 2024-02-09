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
#ifndef RIGID_BODY_HPP
#define RIGID_BODY_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <string>
#include "common_defines.hxx"
#include "digit_rbkd/rbda/rbda.hxx"
#include <numeric>
#include <vector>
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

        // model data
        Eigen::Matrix<myfloat,-1,-1> H;



        // methods
        void initialize(){
                assert(num_bodies > 0);
                H.resize(num_v,num_v);
        }
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
                // return the kappa
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
                return branch_dof;
        }


        PluckerTransform spatial_body_forward_kinematics(myint body_id, myint parent_body_id = -1)
        {
                PluckerTransform iXl = bodies[body_id].Xjtree;
                // go up the tree
                while (bodies[body_id].parent != parent_body_id)
                {
                        body_id = bodies[body_id].parent;
                        iXl = iXl * bodies[body_id].Xjtree;
                }
                // return the pose
                return iXl;
        }

        Eigen::Matrix<myfloat,6,-1> spatial_body_jacobian(myint body_id, myint parent_body_id = -1)
        {
                Eigen::Matrix<myfloat, 6, -1> Jb;
                std::vector<myint> branch = this->kappa(body_id, parent_body_id);
                std::vector<myint> branch_dof = this->branch_dof(branch);
                myint dof = std::accumulate(branch_dof.begin(), branch_dof.end(), 0);
                // resize the jacobian
                Jb.resize(6, dof);
                myint id, num_col=0;
                PluckerTransform iXo;
                Eigen::Matrix<myfloat,6,-1> Sj;
                // go down the tree
                for (myint i = 0 ; i < branch.size(); i++)
                {
                        id = branch.at(i);
                        iXo = (i==0)?bodies[id].Xjtree: bodies[id].Xjtree*iXo;
                        Sj = motion_subspace_matrix(bodies[id].joint.joint_type, bodies[id].joint.joint_axis); //TODO: change to direct block extraction
                        Jb.block(0,num_col,6,Sj.cols()) << iXo.inverse().toMatrix()*Sj; // TODO: change to direct block insertion
                        num_col += Sj.cols();
                }      
                // return the jacobian 
                return Jb; 
        }

        MotionVector spatial_body_corriolis(myint body_id, myint parent_body_id = -1) {
                PluckerTransform oXi,lXi,iXl;
                MotionVector vJ;
                MotionVector ovi;
                MotionVector lvi;
                MotionVector a;

                std::vector<myint> branch = this->kappa(body_id, parent_body_id);

                // go down the tree
                for (myint i = 0 ; i < branch.size(); i++)
                {
                        myint id = branch.at(i);
                        iXl = bodies[id].Xjtree;
                        lXi = bodies[id].Xjtree.inverse();
                        oXi = (i==0)?lXi : oXi*lXi;
                        vJ = MotionVector(Eigen::Matrix<myfloat,6,1>(motion_subspace_matrix(bodies[id].joint.joint_type, bodies[id].joint.joint_axis)*bodies[id].joint.v)); // TODO: find a better way to do this
                        ovi = (i==0)?oXi*vJ : ovi + oXi*vJ;
                        lvi = (i==0)?vJ : iXl*lvi + vJ; 
                        a = (i==0)?MotionVector() : iXl*a + lvi.cross(vJ);
                }
                return oXi*a;
        }

        Pose forward_kinematics(Pose frame, myint body_id, myint parent_body_id = -1)
        {
                PluckerTransform Xup = spatial_body_forward_kinematics(body_id, parent_body_id);
                PluckerTransform Xframe = PluckerTransform(frame) * Xup;
                // // return the pose
                return Xframe.toPose();
        }

        Pose forward_kinematics(Site site, myint parent_body_id = -1)
        {
                return forward_kinematics(site.Xtree.toPose(), site.parent_body, parent_body_id);
        }

        Eigen::Matrix<myfloat,-1,-1> joint_space_inertia_matrix()
        {
                std::vector<myint> dof, csdof; // dof vector and cumulative sum of dof vector
                dof.reserve(num_bodies);
                csdof.reserve(num_bodies);
                for (myint i = 0; i < num_bodies; i++) {
                        dof.push_back(bodies.at(i).joint.v.size());
                        csdof.push_back(std::accumulate(dof.begin(), dof.end(), 0));
                }
                H.setZero();

                // composite inertia calculation
                std::vector<SpatialInertia> Ic;
                Ic.resize(num_bodies);
                myint parent;
                for (myint i = 0; i < num_bodies; i++) {Ic.at(i) = bodies.at(i).spI;}
                for (myint i = num_bodies-1; i >=0 ; i--) {
                        parent = bodies.at(i).parent;
                        if (parent != -1) {
                                Ic.at(parent) = Ic.at(parent) + Ic.at(i).invApply(bodies.at(i).Xjtree);
                        }
                }

                // joint space inertia calculation
                Eigen::Matrix<myfloat,6,-1> F,S;
                myint j=0;

                
                for (myint i = 0; i < num_bodies; i++) {
                        parent = bodies.at(i).parent;
                        S = motion_subspace_matrix(bodies.at(i).joint.joint_type, bodies.at(i).joint.joint_axis);
                        F = Ic.at(i).toMatrix() * S;
                        auto ST = S.transpose() *F;
                        H.block(csdof[i]-dof[i],csdof[i]-dof[i],dof[i],dof[i]) = ST;
                        j = i;
                        while (bodies.at(j).parent != -1) {
                                F = (bodies.at(j).Xjtree.toMatrix().transpose() * F).eval();
                                j = bodies.at(j).parent;
                                S = motion_subspace_matrix(bodies.at(j).joint.joint_type, bodies.at(j).joint.joint_axis);
                                H.block(csdof[j]-dof[j],csdof[i]-dof[i],dof[j],dof[i]) = S.transpose() * F;
                                H.block(csdof[i]-dof[i],csdof[j]-dof[j],dof[i],dof[j]) = H.block(csdof[j]-dof[j],csdof[i]-dof[i],dof[j],dof[i]).transpose().eval();
                        }
                }

                // add armature
                for (myint i = 0; i < num_bodies; i++) {
                        H(csdof[i]-dof[i],csdof[i]-dof[i]) += bodies.at(i).joint.armature;
                }
                
                return H;
        }

        Eigen::Matrix<myfloat,-1,1> joint_space_nonlinear_effects(bool include_gravity = true) { 
                Eigen::Matrix<myfloat,-1,1> C;
                std::vector<myint> dof, csdof; // dof vector and cumulative sum of dof vector
                for (myint i = 0; i < num_bodies; i++) {
                        dof.push_back(bodies.at(i).joint.v.size());
                        csdof.push_back(std::accumulate(dof.begin(), dof.end(), 0));
                }
                C.resize(csdof.back(),1);
                C.setZero();

                // need to create vector of v and avp. Because the value of parent body is needed to calculate the value of child body. But child and parent may not be calculated in order.
                std::vector<MotionVector> avp;
                std::vector<MotionVector> v;
                std::vector<ForceVector> fvp;
                MotionVector vJ;
                avp.resize(num_bodies);
                v.resize(num_bodies);
                fvp.resize(num_bodies);

                avp.at(0) = MotionVector(Eigen::Vector<myfloat,3>::Zero(),-gravity*((include_gravity)?1.0:0.0));
                v.at(0) = MotionVector(Eigen::Matrix<myfloat,6,1>(motion_subspace_matrix(bodies[0].joint.joint_type, bodies[0].joint.joint_axis)*bodies[0].joint.v)); // TODO: find a better way to do this;
                fvp.at(0) = ForceVector(Eigen::Vector<myfloat,3>::Zero(),Eigen::Vector<myfloat,3>::Zero());
                

                //go down the tree
                for(myint i=1; i < num_bodies; i++) {
                        vJ = MotionVector(Eigen::Matrix<myfloat,6,1>(motion_subspace_matrix(bodies[i].joint.joint_type, bodies[i].joint.joint_axis)*bodies[i].joint.v)); // TODO: find a better way to do this;
                        v.at(i) = bodies[i].Xjtree*v.at(bodies[i].parent) + vJ;
                        avp.at(i) = bodies[i].Xjtree*avp.at(bodies[i].parent) + v.at(i).cross(vJ);
                        fvp.at(i) = bodies[i].spI*avp.at(i) + v.at(i).cross(bodies[i].spI*v.at(i));
                }

                // go up the tree
                for(myint i=num_bodies-1; i >=0; i--) {
                        C.block(csdof[i]-dof[i],0,dof[i],1) = motion_subspace_matrix(bodies[i].joint.joint_type, bodies[i].joint.joint_axis).transpose()*fvp.at(i).toVector();
                        if (bodies[i].parent != -1) {
                                fvp.at(bodies[i].parent) = fvp.at(bodies[i].parent) + bodies[i].Xjtree.invApply(fvp.at(i));
                        }
                }
                return C;

        }

        myfloat total_mass() {
                myfloat mass = 0.0;
                for (myint i = 0; i < num_bodies; i++) {
                        mass += bodies.at(i).spI.m;
                }
                return mass;
        }

        Eigen::Matrix<myfloat,3,1> com_position() {
                
                // calculate composite inertia
                std::vector<SpatialInertia> Ic;
                SpatialInertia Itot;
                Ic.resize(num_bodies);
                myint parent;
                for (myint i = 0; i < num_bodies; i++) {Ic.at(i) = bodies.at(i).spI;}
                for (myint i = num_bodies-1; i >=0 ; i--) {
                        parent = bodies.at(i).parent;
                        if (parent != -1) {
                                Ic.at(parent) = Ic.at(parent) + Ic.at(i).invApply(bodies.at(i).Xjtree);
                        } else {
                                Itot = Ic.at(i).invApply(bodies.at(i).Xjtree);
                        }
                }
                return Itot.com_pos();
        }

        Eigen::Matrix<myfloat,6,-1> centroidal_momentum_matrix(Eigen::Matrix<myfloat,-1,-1> H = Eigen::Matrix<myfloat,-1,-1>()) {

                // Using wensing's method https://www.cs.cmu.edu/~cga/z/Wensing_IJHR_2016.pdf
                H = (H.rows()==0)?this->joint_space_inertia_matrix():H;

                Eigen::Matrix<myfloat,6,6> psi = this->spatial_body_jacobian(1).inverse();// body jacobian of rot of base
                PluckerTransform oXG;
                oXG.E = Eigen::Matrix<myfloat,3,3>::Identity();
                oXG.r = - SpatialInertia(psi.transpose()*H.block<6,6>(0,0)*psi).com_pos(); // to directly get Itot

                return oXG.toMatrix().transpose()*psi.transpose()*H.block(0,0,6,H.cols());
        }

        Eigen::Matrix<myfloat,6,1> centroidal_momentum_corriolis(Eigen::Matrix<myfloat,-1,1> C_terms = Eigen::Matrix<myfloat,-1,1>()) {
                
                // Using wensing's method https://www.cs.cmu.edu/~cga/z/Wensing_IJHR_2016.pdf
                C_terms = (C_terms.rows()==0)?this->joint_space_nonlinear_effects(false):C_terms;

                PluckerTransform oXG;
                oXG.E = Eigen::Matrix<myfloat,3,3>::Identity();
                oXG.r = - this->com_position();
                Eigen::Matrix<myfloat,6,6> psi = this->spatial_body_jacobian(1).inverse();// body jacobian of rot of base
                
                return oXG.toMatrix().transpose()*psi.transpose()*C_terms.block<6,1>(0,0);
        }

        Eigen::Matrix<myfloat,-1,1> qvec() {
                Eigen::Matrix<myfloat,-1,1> q;
                q.resize(num_q);
                myint j=0;
                for (myint i = 0; i < num_bodies; i++) {
                        if (bodies.at(i).joint.joint_type != JointType::FIXED) {
                                q.block(j,0,bodies.at(i).joint.q.size(),1) = bodies.at(i).joint.q;
                                j += bodies.at(i).joint.q.size();
                        }
                }
                return q;
        }

        Eigen::Matrix<myfloat,-1,1> vvec() {
                Eigen::Matrix<myfloat,-1,1> v;
                v.resize(num_v);
                myint j=0;
                for (myint i = 0; i < num_bodies; i++) {
                        if (bodies.at(i).joint.joint_type != JointType::FIXED) {
                                v.block(j,0,bodies.at(i).joint.v.size(),1) = bodies.at(i).joint.v;
                                j += bodies.at(i).joint.v.size();
                        }
                }
                return v;
        }


};

#endif // RIGID_BODY_HPP        
