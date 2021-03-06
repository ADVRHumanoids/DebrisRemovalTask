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

namespace myfsm{

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
      std_msgs::String::ConstPtr _debris_number;
      std_msgs::String::ConstPtr _hand_selection;
      ros::ServiceClient _client;
      std::shared_ptr<XBot::Command> current_command;
      geometry_msgs::WrenchStamped::ConstPtr _ft_r_arm;
      double _w_F_ft_initial;
      bool _feedback;
      ros::Publisher _SoftHandPose_pub;
      geometry_msgs::PoseStamped::ConstPtr _initial_pose_left_hand;
      geometry_msgs::PoseStamped::ConstPtr _initial_pose_right_hand;
      geometry_msgs::PoseStamped::ConstPtr _last_pose_left_hand;
      geometry_msgs::PoseStamped::ConstPtr _last_pose_right_hand;
      ros::ServiceClient _grasp_client;
      
      bool _hand_over_phase;
      std::shared_ptr<XBot::PluginStatus> plugin_status;
      double _time = 0;
      bool _first = true;

     
    };
    
    class MacroState : public  XBot::FSM::State< MacroState , SharedData > {
      
    public:

        virtual void entry(const XBot::FSM::Message& msg) {};
        virtual void react(const XBot::FSM::Event& e){};
        tfHandler tf;
      
    };  

 
    class Homing_init : public MacroState {

      virtual std::string get_name() const { return "Homing_init"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };

    class Homing_Ree : public MacroState {

      virtual std::string get_name() const { return "Homing_Ree"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };  
     
    class Homing_Lee : public MacroState {

      virtual std::string get_name() const { return "Homing_Lee"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };       

    class HandSelection : public MacroState {

      virtual std::string get_name() const { return "HandSelection"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
    class Reach : public MacroState {

      virtual std::string get_name() const { return "Reach"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
 
    class Grasp : public MacroState {

      virtual std::string get_name() const { return "Grasp"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
 
    class Pick : public MacroState {

      virtual std::string get_name() const { return "Pick"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };

    class PickSecondHand : public MacroState {

      virtual std::string get_name() const { return "PickSecondHand"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     }; 
 
    class MoveAway : public MacroState {

      virtual std::string get_name() const { return "MoveAway"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
 
    class PlaceDown : public MacroState {

      virtual std::string get_name() const { return "PlaceDown"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
     
    class Ungrasp : public MacroState {

      virtual std::string get_name() const { return "Ungrasp"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };

    class AdjustLaterally : public MacroState {

      virtual std::string get_name() const { return "AdjustLaterally"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
     
    class AdjustForward : public MacroState {

      virtual std::string get_name() const { return "AdjustForward"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     }; 
      
}
