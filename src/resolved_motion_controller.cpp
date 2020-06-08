/******************************************************************************
            ROS computed_torque_controller Package
                           Computed Torque  Controller
          Copyright (C) 2013..2017 Walter Fetter Lages <w.fetter@ieee.org>

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful, but
        WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
        General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.  If not, see
        <http://www.gnu.org/licenses/>.

*******************************************************************************/

#include <sys/mman.h>

#include <resolved_motion_controller/resolved_motion_controller.h>
#include <pluginlib/class_list_macros.h>


namespace effort_controllers
{
    ResolvedMotionController::ResolvedMotionController(void):
        q_(0),dq_(0),qr_(0),dqr_(0),ddqr_(0),torque_(0),fext_(0)
    {
    }

    ResolvedMotionController::~ResolvedMotionController(void)
    {
        sub_command_.shutdown();
        pub.shutdown();
    }

    bool ResolvedMotionController::
            init(hardware_interface::EffortJointInterface *hw,ros::NodeHandle &n)
    {
        node_=n;
        hw_=hw;


        std::vector<std::string> joint_names;
        if(!node_.getParam("joints",joint_names))
        {
            ROS_ERROR("No 'joints' in controller. (namespace: %s)",
                    node_.getNamespace().c_str());
            return false;
        }

        nJoints_=joint_names.size();

        for(int i=0; i < nJoints_;i++)
        {
            try
            {
                joints_.push_back(hw->getHandle(joint_names[i]));
            }
            catch(const hardware_interface::HardwareInterfaceException& e)
            {
                ROS_ERROR_STREAM("Exception thrown: " << e.what());
                return false;
            }
        }
        sub_command_=node_.subscribe("command",1,
                &ResolvedMotionController::commandCB, this);
        pub=node_.advertise<geometry_msgs::PoseStamped>("/topico",1);

        std::string robot_desc_string;
        if(!node_.getParam("/robot_description",robot_desc_string))
        {
            ROS_ERROR("Could not find '/robot_description'.");
            return false;
        }

        if (!kdl_parser::treeFromString(robot_desc_string,tree_))
        {
            ROS_ERROR("Failed to construct KDL tree.");
            return false;
        }

        std::string chainRoot;
        if(!node_.getParam("chain/root",chainRoot))
        {
            ROS_ERROR("Could not find 'chain_root' parameter.");
            return false;
        }

        std::string chainTip;
        if(!node_.getParam("chain/tip",chainTip))
        {
            ROS_ERROR("Could not find 'chain/tip' parameter.");
            return false;
        }

        if (!tree_.getChain(chainRoot,chainTip,chain_))
        {
            ROS_ERROR("Failed to get chain from KDL tree.");
            return false;
        }

//
        if((jnt_to_jac_solver_=new KDL::ChainJntToJacSolver(chain_)) == NULL)
        {
            ROS_ERROR("Failed to create ChainJntToJacSolver.");
            return false;
        }
        J_.resize(nJoints_);
        dJ_.resize(nJoints_);
        ddq_.resize(nJoints_);

        if((jnt_to_pose_solver_=new KDL::ChainFkSolverPos_recursive(chain_)) == NULL)
                {
                    ROS_ERROR("Failed to create ChainFkSolverPos_recursive.");
                    return false;
                }

        if((jnt_to_jac_dot_solver_=new KDL::ChainJntToJacDotSolver(chain_)) == NULL)
                {
                    ROS_ERROR("Failed to create ChainJntToJacDotSolver.");
                    return false;
                }






//
        KDL::Vector g;
        node_.param("gravity/x",g[0],0.0);
        node_.param("gravity/y",g[1],0.0);
        node_.param("gravity/z",g[2],-9.8);

        if((idsolver_=new KDL::ChainIdSolver_RNE(chain_,g)) == NULL)
        {
            ROS_ERROR("Failed to create ChainIDSolver_RNE.");
            return false;
        }

        q_.resize(nJoints_);
        dq_.resize(nJoints_);
        //v_.resize(nJoints_);
        qr_.resize(nJoints_);
        dqr_.resize(nJoints_);
        ddqr_.resize(nJoints_);
        torque_.resize(nJoints_);

        fext_.resize(chain_.getNrOfSegments());

        Kp_.resize(6,6);
        Kd_.resize(6,6);

        std::vector<double> KpVec;
        if(!node_.getParam("Kp",KpVec))
        {
            ROS_ERROR("No 'Kp' in controller %s.",node_.getNamespace().c_str());
            return false;
        }
        Kp_=Eigen::Map<Eigen::MatrixXd>(KpVec.data(),6,6).transpose();

        std::vector<double> KdVec;
        if(!node_.getParam("Kd",KdVec))
        {
            ROS_ERROR("No 'Kd' in controller %s.",node_.getNamespace().c_str());
            return false;
        }
        Kd_=Eigen::Map<Eigen::MatrixXd>(KdVec.data(),6,6).transpose();

        return true;
    }

    void ResolvedMotionController::starting(const ros::Time& time)
    {
        for(unsigned int i=0;i < nJoints_;i++)
        {
            q_(i)=joints_[i].getPosition();
            dq_(i)=joints_[i].getVelocity();
        }

        //referencia no inicio
        double a=0.0;
        double b=0.0;
        double y=0.0;

//                for(unsigned int k=0;k < 3;k++)
//                            for(unsigned int l=0;l < 3;l++)
//                                Xd_.M.data[k,l] = 0.1;

        Xd_.p.data[0] = .20;//x
        Xd_.p.data[1] = .13;//y
        Xd_.p.data[2] = 1.2;//z


        for(int k=0;k < 3;k++)
            dXd_.vel.data[k] = 0;

        for(int k=0;k < 3;k++)
            dXd_.rot.data[k] = 0;


        for(int k=0;k < 3;k++)
            ddXd_.force.data[k] = 0;

        for(int k=0;k < 3;k++)
            ddXd_.torque.data[k] = 0;

        //calcula matriz de rotação a partir de angulos RPY
        Xd_.M.data[0]= cos(y)*cos(b);
        Xd_.M.data[1]= cos(y)*sin(b)*sin(a)-sin(y)*cos(a);
        Xd_.M.data[2]= cos(y)*sin(b)*cos(a)+sin(y)*sin(a);
        Xd_.M.data[3]= sin(y)*cos(b);
        Xd_.M.data[4]=sin(y)*sin(b)*sin(a)+cos(y)*cos(a);
        Xd_.M.data[5]=sin(y)*sin(b)*cos(a)-sin(y)*sin(a);
        Xd_.M.data[6]=-sin(b);
        Xd_.M.data[7]=cos(b)*sin(a);
        Xd_.M.data[8]=cos(b)*cos(a);

        //qr_=q_;
        //dqr_=dq_;
        //SetToZero(ddqr_);

        struct sched_param param;
        if(!node_.getParam("priority",param.sched_priority))
        {
            ROS_WARN("No 'priority' configured for controller %s. Using highest possible priority.",node_.getNamespace().c_str());
            param.sched_priority=sched_get_priority_max(SCHED_FIFO);
        }
        if(sched_setscheduler(0,SCHED_FIFO,&param) == -1)
        {
            ROS_WARN("Failed to set real-time scheduler.");
            return;
        }
        if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
            ROS_WARN("Failed to lock memory.");


    }

    void ResolvedMotionController::update(const ros::Time& time,
            const ros::Duration& duration)
    {
        for(unsigned int i=0;i < nJoints_;i++)
        {
            q_(i)=joints_[i].getPosition();
            dq_(i)=joints_[i].getVelocity();
        }
/////////
        q_in_.q=q_;
        q_in_.qdot=dq_;

        jnt_to_jac_solver_->JntToJac(q_, J_); //atualiza jacobiano
        //std::cout<<J_.data<<std::endl;
        //chain_.getPositions(q_);
        jnt_to_pose_solver_->JntToCart(q_, X_); //atualiza o X_
        std::cout<<X_.p.data[0]<<X_.p.data[1]<<X_.p.data[2]<<std::endl;
        jnt_to_jac_dot_solver_ -> JntToJacDot(q_in_, dJ_);  //atualiza a derivada do jacobiano

        double alpha,beta,gamma;
        double dalpha,dbeta,dgamma;

        X_.M.GetRPY(alpha, beta, gamma);

        Eigen::MatrixXd Ta_(6,6);

        Ta_ << 1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0 ,0 ,0,
                0, 0, 0, cos(gamma)*cos(beta), -sin(gamma), 0,
                0, 0, 0, sin(gamma)*cos(gamma), cos(gamma), 0,
                0, 0, 0, -sin(gamma), 0, 1;

        Eigen::MatrixXd Ja_ = Ta_.inverse()*J_.data;		//salvando em variaveis intermediarias

        //achar os dalpha, dbeta, dgamma: dX = Ja*dq

        Eigen::MatrixXd dX_ = Ja_*dq_.data;

        dalpha = dX_(3);
        dbeta = dX_(4);
        dgamma = dX_(5);

        //calcula derivada de Ta
        Eigen::MatrixXd dTa_(6,6);

        dTa_ <<0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, -dgamma*sin(gamma)*cos(beta)-dbeta*cos(gamma)*sin(beta), -dgamma*cos(gamma), 0,
                0, 0, 0, dgamma*cos(gamma)*cos(beta)-dbeta*sin(gamma)*sin(beta), -dgamma*sin(gamma), 0,
                0, 0, 0, -dgamma*cos(gamma), 0, 0;

        Eigen::MatrixXd dJa_ = Ta_.inverse()*(dJ_.data - dTa_*Ja_);    //calcula deriva do jacobiano analitico

        Eigen::MatrixXd M_(3,3);		//orientacao

        M_ << X_.M.data[0], X_.M.data[1], X_.M.data[2],
                X_.M.data[3], X_.M.data[4], X_.M.data[5],
                X_.M.data[6], X_.M.data[7], X_.M.data[8];

        Eigen::MatrixXd Md_(3,3);		//orientacao

        Md_ << Xd_.M.data[0], Xd_.M.data[1], Xd_.M.data[2],
                Xd_.M.data[3], Xd_.M.data[4], Xd_.M.data[5],
                Xd_.M.data[6], Xd_.M.data[7], Xd_.M.data[8];

        Eigen::Vector3d f = M_.col(0);		//separa as colunas das matrizes de rotacao para fazer produto vetorial
        Eigen::Vector3d s = M_.col(1);
        Eigen::Vector3d a = M_.col(2);
        Eigen::Vector3d fd = Md_.col(0);
        Eigen::Vector3d sd = Md_.col(1);
        Eigen::Vector3d ad = Md_.col(2);

        Eigen::Vector3d eo_(3);

        eo_ = (f.cross(fd) + s.cross(sd) + a.cross(ad))/2; 	//calcula o erro fazendo o produto vetorial
        std::cout<< eo_<<std::endl;

        Eigen::VectorXd erro_(6);

        erro_ << Xd_.p.data[0] - X_.p.data[0],
                Xd_.p.data[1] - X_.p.data[1],
                Xd_.p.data[2] - X_.p.data[2],
                eo_[0],
                eo_[1],
                eo_[2];

        //calcula o erro de velocidade
        Eigen::VectorXd ev_(6);
        ev_ << dXd_.vel.data[0] - dX_(0),
                dXd_.vel.data[1] - dX_(1),
                dXd_.vel.data[2] - dX_(2),
                dXd_.rot.data[0] - dX_(3),
                dXd_.rot.data[1] - dX_(4),
                dXd_.rot.data[2] - dX_(5);

        Eigen::VectorXd aceleracao_d_(6);

        aceleracao_d_ << ddXd_.force.data[0],
                ddXd_.force.data[1],
                ddXd_.force.data[2],
                ddXd_.torque.data[0],
                ddXd_.torque.data[1],
                ddXd_.torque.data[2];

        //calculo da pseudo inversa

        Eigen::MatrixXd Jainversa_ = Ja_*Ja_.transpose();

        Jainversa_ = Ja_.transpose()*Jainversa_.inverse();

        ddq_.data = Jainversa_*(aceleracao_d_ + Kd_*ev_ + Kp_*erro_ - dJa_*dq_.data);


/////
        for(unsigned int i=0;i < fext_.size();i++) fext_[i].Zero();

        //v_.data=ddqr_.data+Kp_*(qr_.data-q_.data)+Kd_*(dqr_.data-dq_.data);
        if(idsolver_->CartToJnt(q_,dq_,ddq_,fext_,torque_) < 0)
                ROS_ERROR("KDL inverse dynamics solver failed.");

        for(unsigned int i=0;i < nJoints_;i++)
                joints_[i].setCommand(torque_(i));


        msg.pose.position.x=X_.p.data[0];
        msg.pose.position.y=X_.p.data[1];
        msg.pose.position.z=X_.p.data[2];

        pub.publish(msg);


    }

    void ResolvedMotionController::commandCB(const trajectory_msgs::
            JointTrajectoryPoint::ConstPtr &referencePoint)
    {
                double a=referencePoint->positions[3];
                double b=referencePoint->positions[4];
                double y=referencePoint->positions[5];

                for(unsigned int i=0;i < 3;i++)
                {
                    Xd_.p.data[i]=referencePoint->positions[i];
                    dXd_.vel.data[i]=referencePoint->velocities[i];
                    ddXd_.force.data[i]=referencePoint->accelerations[i];

        //            Xd_.p.data[i+3]=referencePoint->positions[i+3];
                    dXd_.rot.data[i]=referencePoint->velocities[i+3];
                    ddXd_.torque.data[i]=referencePoint->accelerations[i+3];
                }

                //calcula matriz de rotação a partir de angulos RPY
                Xd_.M.data[0]= cos(y)*cos(b);
                Xd_.M.data[1]= cos(y)*sin(b)*sin(a)-sin(y)*cos(a);
                Xd_.M.data[2]= cos(y)*sin(b)*cos(a)+sin(y)*sin(a);
                Xd_.M.data[3]= sin(y)*cos(b);
                Xd_.M.data[4]=sin(y)*sin(b)*sin(a)+cos(y)*cos(a);
                Xd_.M.data[5]=sin(y)*sin(b)*cos(a)-sin(y)*sin(a);
                Xd_.M.data[6]=-sin(b);
                Xd_.M.data[7]=cos(b)*sin(a);
                Xd_.M.data[8]=cos(b)*cos(a);


    }
}

PLUGINLIB_EXPORT_CLASS(effort_controllers::ResolvedMotionController,
        controller_interface::ControllerBase)