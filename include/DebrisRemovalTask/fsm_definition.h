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
#include<eigen_conversions/eigen_msg.h>

#include <trajectory_utils/segment.h>
#include <trajectory_utils/Cartesian.h>

#include <XBotCore-interfaces/XDomainCommunication.h>


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
*/
    struct SharedData {
      
      XBot::RobotInterface::Ptr _robot;
      std::shared_ptr<ros::NodeHandle> _nh;
      geometry_msgs::PoseStamped::ConstPtr _debris_pose;
      std_msgs::String::ConstPtr _debris_number;
      ros::ServiceClient _client;
      XBot::SubscriberRT<XBot::Command> command;
      XBot::Command current_command;
      ros::Publisher _grasp_mag_pub;
     
    };
    
    class MacroState : public  XBot::FSM::State< MacroState , SharedData > {
      
    public:
	
	virtual void entry(const XBot::FSM::Message& msg) {};
	virtual void react(const XBot::FSM::Event& e){};
      
    };  

 
    class Homing : public MacroState {

      virtual std::string get_name() const { return "Homing"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
 
    class Reached : public MacroState {

      virtual std::string get_name() const { return "Reached"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
 
    class Grasped : public MacroState {

      virtual std::string get_name() const { return "Grasped"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
 
    class Picked : public MacroState {

      virtual std::string get_name() const { return "Picked"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
 
    class MovedAway : public MacroState {

      virtual std::string get_name() const { return "MovedAway"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
 
    class PlacedDown : public MacroState {

      virtual std::string get_name() const { return "PlacedDown"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
 
    class Ungrasped : public MacroState {

      virtual std::string get_name() const { return "Ungrasped"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
     
      
}
