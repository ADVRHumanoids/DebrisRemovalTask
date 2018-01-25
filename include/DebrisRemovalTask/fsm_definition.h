/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Pietro Balatti
 * email: pietro.balattis@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/
#include <XBotInterface/StateMachine.h>
#include<iostream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <ADVR_ROS/advr_segment_control.h>
#include <ADVR_ROS/advr_grasp_control_srv.h>
#include <ADVR_ROS/im_pose_msg.h>
#include<eigen_conversions/eigen_msg.h>

#include <trajectory_utils/segment.h>
#include <trajectory_utils/Cartesian.h>

#include <XBotCore-interfaces/XDomainCommunication.h>

#include <tf/transform_listener.h>
#include <Eigen/Dense>

#include <geometry_msgs/WrenchStamped.h>
#include <XCM/XBotPluginStatus.h>


#define TRAJ_DURATION 5
#define APPROCHING_SHIFT 0.1
#define RETREAT_SHIFT 0.1
#define VALVE_RADIUSE 0.2

#define CENTER_SHIFT 0.2


namespace myfsm{

    inline double RADTODEG(double x) { return x * 180.0 / M_PI;};
    inline double DEGTORAD(double x) { return x * M_PI / 180.0;};


    inline Eigen::Quaterniond zyx2quat(double x, double y, double z)
    {
        Eigen::Matrix3d rot;
        rot =
                Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX());
        return Eigen::Quaterniond(rot).normalized();
    }

/*Example how to define a custom Event*/
/*  class MyEvent : public XBot::FSM::Event {

    public:

      MyEvent(int id): id(id) {}
      
      int id;    

    };
*/

/*Example how to define a custom Message*/   
/*  class MyMessage : public XBot::FSM::Message {

    public:
      
      MyMessage (int id):id(id){};
      
      int id;
	
    };
*/class tfHandler{
    public:
      tfHandler():
      _listener(), _gm_transform(), _transform()
      {
	
      }
      
      bool getTransformTf(const std::string& parent, const std::string& child, Eigen::Affine3d& world_T_bl )
      {
	try
	{
	  ros::Time now = ros::Time::now();
	  //if(_listener.waitForTransform(child, parent, now, ros::Duration(5.0)))
	  //{
	    _listener.lookupTransform(child, parent,  ros::Time(0), _transform);
	  
	    tf::transformTFToMsg(_transform, _gm_transform);
	    tf::transformMsgToEigen(_gm_transform, world_T_bl);
	  
	    return true;
	  //}
// 	  else
// 	    return false;
	}
	catch (tf::TransformException ex)
	{
	  ROS_ERROR("%s",ex.what());
	  return false;
	}
      }
    private:
     tf::TransformListener _listener; 
      geometry_msgs::Transform _gm_transform; 
      tf::StampedTransform _transform;
      
    };

    struct SharedData {
      
      XBot::RobotInterface::Ptr _robot;
      std::shared_ptr<ros::NodeHandle> _nh;
      geometry_msgs::PoseStamped::ConstPtr _debris_pose;
      geometry_msgs::PoseStamped::Ptr _valve_pose;
      std_msgs::String::ConstPtr _debris_number;
      std_msgs::String::ConstPtr _hand_selection;
      
      bool no_hand_selection = true;
      
      ros::ServiceClient _client;
      XBot::SubscriberRT<XBot::Command> command;
      std::shared_ptr<XBot::Command> current_command;
      ros::Publisher _grasp_mag_pub_LSoftHand;
      ros::Publisher _grasp_mag_pub_RSoftHand;
      geometry_msgs::WrenchStamped::ConstPtr _ft_r_arm;
      double _w_F_ft_initial;
      bool _feedback;
      ros::Publisher _SoftHandPose_pub;
      geometry_msgs::PoseStamped::ConstPtr _last_pose;
      geometry_msgs::PoseStamped::ConstPtr _initial_pose_right_hand;
      geometry_msgs::PoseStamped::ConstPtr _initial_pose_left_hand;
      geometry_msgs::PoseStamped::ConstPtr _last_pose_left_hand;
      ros::ServiceClient _grasp_client;
      
      bool _hand_over_phase;
      std::shared_ptr<XBot::PluginStatus> plugin_status;


//        geometry_msgs::PoseStamped left_hand_pose_stamped_global_home_;
//        geometry_msgs::PoseStamped right_hand_pose_stamped_global_home_;
//
//        geometry_msgs::PoseStamped left_hand_pose_stamped_task_home_;
//        geometry_msgs::PoseStamped right_hand_pose_stamped_task_home_;
//
//        geometry_msgs::PoseStamped last_left_hand_pose_stamped_;
//        geometry_msgs::PoseStamped last_right_hand_pose_stamped_;


        //////////////////////////////////////////////////////////////////////////////////////////////////////
        //// valve task
        //////////////////////////////////////////////////////////////////////////////////////////////////////

        XBot::RobotInterface::Ptr robot_;
        tfHandler tf_;
        std::string selectedHand_;

        std::string floating_base_link_name_;
        Eigen::Affine3d world_T_floating_base_;


        Eigen::Affine3d             left_hand_pose_Affine_,         right_hand_pose_Affine_;
        geometry_msgs::Pose         left_hand_pose_Pose_,           right_hand_pose_Pose_;
        geometry_msgs::PoseStamped  left_hand_pose_PoseStamped_,    right_hand_pose_PoseStamped_;




        Eigen::Affine3d             left_hand_pose_home_Affine_,            right_hand_pose_home_Affine_;
        geometry_msgs::Pose         left_hand_pose_home_Pose_,              right_hand_pose_home_Pose_;
        geometry_msgs::PoseStamped  left_hand_pose_home_PoseStamped_,       right_hand_pose_home_PoseStamped_;




        void updateRobotStates(){
            // update robot states
            robot_->sense();
            robot_->model().getFloatingBaseLink(floating_base_link_name_);
            tf_.getTransformTf(floating_base_link_name_, "world_odom", world_T_floating_base_);
            robot_->model().setFloatingBasePose(world_T_floating_base_);
            robot_->model().update();

            // calculate needed information
            robot_->model().getPose("LSoftHand", left_hand_pose_Affine_);
            robot_->model().getPose("RSoftHand", right_hand_pose_Affine_);

            tf::poseEigenToMsg (left_hand_pose_Affine_, left_hand_pose_Pose_);
            tf::poseEigenToMsg (right_hand_pose_Affine_, right_hand_pose_Pose_);

            left_hand_pose_PoseStamped_.pose = left_hand_pose_Pose_;
            right_hand_pose_PoseStamped_.pose = right_hand_pose_Pose_;

        }


        bool home_recoreded_ = false;

        void recordHome(){

                left_hand_pose_home_Affine_ = left_hand_pose_Affine_;
                right_hand_pose_home_Affine_ = right_hand_pose_Affine_;

                left_hand_pose_home_Pose_ = left_hand_pose_Pose_;
                right_hand_pose_home_Pose_ = right_hand_pose_Pose_;

                left_hand_pose_home_PoseStamped_ = left_hand_pose_PoseStamped_;
                right_hand_pose_home_PoseStamped_ = right_hand_pose_PoseStamped_;

                home_recoreded_ = true;

        }


        // define a bunch of key poses based on valve model pose
        geometry_msgs::PoseStamped::ConstPtr valve_center_pose_PoseStamped_ConstPtr_;


        geometry_msgs::PoseStamped valve_center_pose_PoseStamped_;
        geometry_msgs::PoseStamped valve_up_pose_PoseStamped_;
        geometry_msgs::PoseStamped valve_down_pose_PoseStamped_;
        geometry_msgs::PoseStamped valve_left_pose_PoseStamped_;
        geometry_msgs::PoseStamped valve_right_pose_PoseStamped_;

        // valve model parameters needed to calculate those key poses
        double handel_length_ = 0.3;
        Eigen::Vector3d valve_center_position_wrt_base_ = Eigen::Vector3d(0.1, 0.0, 1.2);



        void calcValveKeyPoses(){
            valve_center_pose_PoseStamped_ = *valve_center_pose_PoseStamped_ConstPtr_;

            Eigen::Affine3d valve_center_pose_Affine;
            tf::poseMsgToEigen(valve_center_pose_PoseStamped_.pose, valve_center_pose_Affine);
            std::cout << "valve_center_pose_Affine: " << valve_center_pose_Affine.translation().transpose() << std::endl;


            Eigen::Affine3d valve_up_pose_Affine   ;
            Eigen::Affine3d valve_down_pose_Affine ;
            Eigen::Affine3d valve_left_pose_Affine ;
            Eigen::Affine3d valve_right_pose_Affine;

            Eigen::Affine3d center_T_up_Affine      = Eigen::Affine3d::Identity();
            Eigen::Affine3d center_T_down_Affine    = Eigen::Affine3d::Identity();
            Eigen::Affine3d center_T_left_Affine    = Eigen::Affine3d::Identity();
            Eigen::Affine3d center_T_right_Affine   = Eigen::Affine3d::Identity();

            center_T_up_Affine.translation()    = Eigen::Vector3d(0.0, 0.0, CENTER_SHIFT);
            center_T_down_Affine.translation()  = Eigen::Vector3d(0.0, 0.0, -CENTER_SHIFT);
            center_T_left_Affine.translation()  = Eigen::Vector3d(0.0, -CENTER_SHIFT, 0.0);
            center_T_right_Affine.translation() = Eigen::Vector3d(0.0, CENTER_SHIFT, 0.0);

            valve_up_pose_Affine    = valve_center_pose_Affine*center_T_up_Affine;
            valve_down_pose_Affine  = valve_center_pose_Affine*center_T_down_Affine;
            valve_left_pose_Affine  = valve_center_pose_Affine*center_T_left_Affine;
            valve_right_pose_Affine = valve_center_pose_Affine*center_T_right_Affine;

            std::cout << "valve_up_pose_Affine: " << valve_up_pose_Affine.translation().transpose() << std::endl;
            std::cout << "valve_down_pose_Affine: " << valve_down_pose_Affine.translation().transpose() << std::endl;
            std::cout << "valve_left_pose_Affine: " << valve_left_pose_Affine.translation().transpose() << std::endl;
            std::cout << "valve_right_pose_Affine: " << valve_right_pose_Affine.translation().transpose() << std::endl;


            // transfer datatype
            tf::poseEigenToMsg(valve_up_pose_Affine, valve_up_pose_PoseStamped_.pose);
            tf::poseEigenToMsg(valve_down_pose_Affine, valve_down_pose_PoseStamped_.pose);
            tf::poseEigenToMsg(valve_left_pose_Affine, valve_left_pose_PoseStamped_.pose);
            tf::poseEigenToMsg(valve_right_pose_Affine, valve_right_pose_PoseStamped_.pose);

        }


    };
    
    class MacroState : public  XBot::FSM::State< MacroState , SharedData > {
      
    public:

        virtual void entry(const XBot::FSM::Message& msg) {};
        virtual void react(const XBot::FSM::Event& e){};
        tfHandler tf;
      
    };  

    class Homing : public MacroState {

      virtual std::string get_name() const { return "Homing"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
    class ValveReach : public MacroState {

      virtual std::string get_name() const { return "ValveReach"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
    class ValveTurn : public MacroState {

      virtual std::string get_name() const { return "ValveTurn"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
    class ValveGoBack : public MacroState {

      virtual std::string get_name() const { return "ValveGoBack"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };     
      
}
