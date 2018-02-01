/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Pietro Balatti
 * email: pietro.balatti@iit.it
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

#include <DebrisRemovalTask_rt_plugin.h>


/* Specify that the class XBotPlugin::DebrisRemovalTask is a XBot RT plugin with name "DebrisRemovalTask" */
REGISTER_XBOT_PLUGIN(DebrisRemovalTask, XBotPlugin::DebrisRemovalTask)

namespace XBotPlugin {

bool DebrisRemovalTask::init_control_plugin(XBot::Handle::Ptr handle)
{
    /* This function is called outside the real time loop, so we can
     * allocate memory on the heap, print stuff, ...
     * The RT plugin will be executed only if this init function returns true. */


    /* Save robot to a private member. */
    _robot = handle->getRobotInterface();

    /* Initialize a logger which saves to the specified file. Remember that
     * the current date/time is always appended to the provided filename,
     * so that logs do not overwrite each other. */
    
    _logger = XBot::MatLogger::getLogger("/tmp/DebrisRemovalTask_log");
    
    // ROS initialization
    int argc = 1;
    const char *arg = "dummy_arg";
    char* argg = const_cast<char*>(arg);
    char** argv = &argg;
    
    ros::init(argc, argv, "DebrisRemovalTask");
    fsm.shared_data()._nh =  std::make_shared<ros::NodeHandle>();
    fsm.shared_data().current_command = std::shared_ptr<XBot::Command>(&current_command);
    fsm.shared_data().plugin_status = _custom_status;
    
    fsm.shared_data()._client = fsm.shared_data()._nh->serviceClient<ADVR_ROS::advr_segment_control>("segment_control");    
    _feedBack = fsm.shared_data()._nh->subscribe("Manipulation_status",1,&DebrisRemovalTask::on_manipulation_status,this);
    manipulation_status = true;
    fsm.shared_data()._SoftHandPose_pub = fsm.shared_data()._nh->advertise<geometry_msgs::PoseStamped>("/w_T_right_ee",1);
    fsm.shared_data()._grasp_client = fsm.shared_data()._nh->serviceClient<ADVR_ROS::advr_grasp_control_srv>("grasp_control");
    
    fsm.shared_data()._hand_over_phase = false;
    
    /*Saves robot as shared variable between states*/
    fsm.shared_data()._robot = handle->getRobotInterface();
    
    /*Registers states*/
    fsm.register_state(std::make_shared<myfsm::Homing_init>());
    fsm.register_state(std::make_shared<myfsm::Homing_Ree>());
    fsm.register_state(std::make_shared<myfsm::Homing_Lee>());
    fsm.register_state(std::make_shared<myfsm::HandSelection>());
    fsm.register_state(std::make_shared<myfsm::Reach>());
    fsm.register_state(std::make_shared<myfsm::Grasp>());
    fsm.register_state(std::make_shared<myfsm::Pick>());
    fsm.register_state(std::make_shared<myfsm::PickSecondHand>());
    fsm.register_state(std::make_shared<myfsm::MoveAway>());
    fsm.register_state(std::make_shared<myfsm::PlaceDown>());
    fsm.register_state(std::make_shared<myfsm::Ungrasp>());
    fsm.register_state(std::make_shared<myfsm::AdjustLaterally>());
    fsm.register_state(std::make_shared<myfsm::AdjustForward>());

    return true;
}

void DebrisRemovalTask::on_manipulation_status(const std_msgs::Bool::ConstPtr& msg)
{
  
  manipulation_status = msg->data;
  fsm.shared_data()._feedback = manipulation_status;

}

void DebrisRemovalTask::on_start(double time)
{
    /* This function is called on plugin start, i.e. when the start command
     * is sent over the plugin switch port (e.g. 'rosservice call /DebrisRemovalTask_switch true').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* Save the plugin starting time to a class member */
    _robot->getMotorPosition(_q0);

    /* Save the robot starting config to a class member */
    _start_time = time;
    
        
    // Initialize the FSM with the initial state
    fsm.init("Homing_init");
}

void DebrisRemovalTask::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /DebrisRemovalTask_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
}


void DebrisRemovalTask::control_loop(double time, double period)
{
    /* This function is called on every control loop from when the plugin is start until
     * it is stopped.
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */


//      Logger::warning() << current_command.str() << Logger::endl();
     fsm.run(time, 0.01);
     
}

bool DebrisRemovalTask::close()
{
    /* This function is called exactly once, at the end of the experiment.
     * It can be used to do some clean-up, or to save logging data to disk. */

    /* Save logged data to disk */
    _logger->flush();

    return true;
}



}
