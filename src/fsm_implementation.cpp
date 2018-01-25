#include "fsm_definition.h"

#include <vector>
#include <string>

#include <eigen_conversions/eigen_msg.h>
#include <kdl_conversions/kdl_msg.h>


/******************************** BEGIN Homing *******************************/
///////////////////////////////////////////////////////////////////////////////

void myfsm::Homing::react(const XBot::FSM::Event& e) {
    
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing::entry(const XBot::FSM::Message& msg){
    std::cout << "Homing::entry()" << std::endl;


    shared_data().plugin_status->setStatus("HOMING");
    

  
//    //CALL SERVICE TO MOVE
//    // send a trajectory for the end effector as a segment
//
//
//    // get floating base information and update model
//    shared_data()._robot->sense();
//    std::string floating_base_link_name;
//    shared_data()._robot->model().getFloatingBaseLink(floating_base_link_name);
////    std::cout << "floating_base_link_name" << floating_base_link_name << std::endl;
//
//    Eigen::Affine3d world_T_floating_base;
//    tf.getTransformTf(floating_base_link_name, "world_odom", world_T_floating_base);
////    std::cout << "floating_base_link position: \n" << world_T_floating_base.translation().transpose() << std::endl;
////    std::cout << "floating_base_link orientation: \n" << world_T_floating_base.rotation().eulerAngles(2, 1, 0).transpose() << std::endl;
//
//    shared_data()._robot->model().setFloatingBasePose(world_T_floating_base);
//    shared_data()._robot->model().update();

    shared_data().updateRobotStates();


    if (!shared_data().home_recoreded_){
        shared_data().recordHome();
    }


//    // get current hand poses as global home
//    Eigen::Affine3d right_hand_pose_eigen, left_hand_pose_eigen;
//    shared_data()._robot->model().getPose("LSoftHand", left_hand_pose_eigen);
//    shared_data()._robot->model().getPose("RSoftHand", right_hand_pose_eigen);
////    std::cout << "left_hand_pose_eigen: " << left_hand_pose_eigen.translation().transpose() << std::endl;
////    std::cout << "right_hand_pose_eigen: " << right_hand_pose_eigen.translation().transpose() << std::endl;
//
//
//
//    geometry_msgs::Pose right_hand_pose,left_hand_pose;
//    tf::poseEigenToMsg (left_hand_pose_eigen, left_hand_pose);
//    tf::poseEigenToMsg (right_hand_pose_eigen, right_hand_pose);
//
//    geometry_msgs::PoseStamped right_hand_pose_stamped, left_hand_pose_stamped;
//    left_hand_pose_stamped.pose = left_hand_pose;
//    right_hand_pose_stamped.pose = right_hand_pose;
//
//
//
////    geometry_msgs::PoseStamped right_hand_pose_stamped_global_home, left_hand_pose_stamped_global_home;
////    left_hand_pose_stamped_global_home = left_hand_pose_stamped;
////    right_hand_pose_stamped_global_home = right_hand_pose_stamped;
//
//    shared_data().left_hand_pose_stamped_global_home_ =  left_hand_pose_stamped;
//    shared_data().right_hand_pose_stamped_global_home_ = right_hand_pose_stamped;
//
//
////    std::cout << "left_hand_pose_stamped: " << left_hand_pose_stamped.pose.position << std::endl;
////    std::cout << "right_hand_pose_stamped: " << right_hand_pose_stamped.pose.position << std::endl;


//    // define task home pose for both hands
//    Eigen::Vector3d left_hand_task_home_position, right_hand_task_home_position;
//    Eigen::Quaterniond left_hand_task_home_quaternion, right_hand_task_home_quaternion;
//    left_hand_task_home_position = Eigen::Vector3d(0.3, 0.4, 1.0);
//    left_hand_task_home_quaternion = zyx2quat(0.0, DEGTORAD(-45), 0.0);
//    right_hand_task_home_position = Eigen::Vector3d(0.3, -0.4, 1.0);
//    right_hand_task_home_quaternion = zyx2quat(0.0, DEGTORAD(-45), 0.0);
//
//
//
//
//    geometry_msgs::PoseStamped left_hand_pose_stamped_task_home, right_hand_pose_stamped_task_home;
//    left_hand_pose_stamped_task_home.pose.position.x =    left_hand_task_home_position[0];
//    left_hand_pose_stamped_task_home.pose.position.y =    left_hand_task_home_position[1];
//    left_hand_pose_stamped_task_home.pose.position.z =    left_hand_task_home_position[2];
//    left_hand_pose_stamped_task_home.pose.orientation.x = left_hand_task_home_quaternion.x();
//    left_hand_pose_stamped_task_home.pose.orientation.y = left_hand_task_home_quaternion.y();
//    left_hand_pose_stamped_task_home.pose.orientation.z = left_hand_task_home_quaternion.z();
//    left_hand_pose_stamped_task_home.pose.orientation.w = left_hand_task_home_quaternion.w();
//
//    right_hand_pose_stamped_task_home.pose.position.x =    right_hand_task_home_position[0];
//    right_hand_pose_stamped_task_home.pose.position.y =    right_hand_task_home_position[1];
//    right_hand_pose_stamped_task_home.pose.position.z =    right_hand_task_home_position[2];
//    right_hand_pose_stamped_task_home.pose.orientation.x = right_hand_task_home_quaternion.x();
//    right_hand_pose_stamped_task_home.pose.orientation.y = right_hand_task_home_quaternion.y();
//    right_hand_pose_stamped_task_home.pose.orientation.z = right_hand_task_home_quaternion.z();
//    right_hand_pose_stamped_task_home.pose.orientation.w = right_hand_task_home_quaternion.w();
//
//    shared_data().left_hand_pose_stamped_task_home_ = left_hand_pose_stamped_task_home;
//    shared_data().right_hand_pose_stamped_task_home_ = right_hand_pose_stamped_task_home;

//    std::cout << "left_hand_pose_stamped_task_home: " << left_hand_pose_stamped_task_home.pose.position << std::endl;
//    std::cout << "right_hand_pose_stamped_task_home: " << right_hand_pose_stamped_task_home.pose.position << std::endl;




    // select the hand to use
    std::cout << "Please select the hand you want to move!" << std::endl;

    std::string selectedHand;
    selectedHand = "LSoftHand";
    selectedHand = "RSoftHand";
    shared_data().selectedHand_ = selectedHand;
    std::cout << "Hand selected: " << selectedHand << std::endl;

    trajectory_utils::Cartesian start, end;
    if(selectedHand == "LSoftHand"){
        start.distal_frame = "LSoftHand";
        start.frame = shared_data().left_hand_pose_PoseStamped_;
        end.distal_frame = "LSoftHand";
        end.frame = shared_data().left_hand_pose_home_PoseStamped_;
    }else if(selectedHand == "RSoftHand"){
        start.distal_frame = "RSoftHand";
        start.frame = shared_data().right_hand_pose_PoseStamped_;
        end.distal_frame = "RSoftHand";
        end.frame = shared_data().right_hand_pose_home_PoseStamped_;
    }

    // define one segment
    trajectory_utils::segment segment;
    segment.type.data = 0;        // min jerk traj
    segment.T.data = TRAJ_DURATION;         // traj duration 5 second
    segment.start = start;        // start pose
    segment.end = end;            // end pose

    // define segments
    std::vector<trajectory_utils::segment> segments;
    segments.push_back(segment);

    // prepare the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = "world_odom";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;

    // call the service
    shared_data()._client.call(srv);
    std::cout << "Move to home pose!" << std::endl;


    // blocking call: wait for a pose on topic debris_pose
    std::cout << "Please define the pose of the valve!" << std::endl;
    ADVR_ROS::im_pose_msg::ConstPtr tmp;
    tmp = ros::topic::waitForMessage<ADVR_ROS::im_pose_msg>("valve_pose");
    shared_data()._valve_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(tmp->pose_stamped));

//    std::cout << "Please define the pose of the valve center!" << std::endl;
//    ADVR_ROS::im_pose_msg::ConstPtr valve_center_pose_msg;
//    valve_center_pose_msg = ros::topic::waitForMessage<ADVR_ROS::im_pose_msg>("valve_center_pose");
//    shared_data().valve_center_pose_PoseStamped_ConstPtr_ = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(valve_center_pose_msg->pose_stamped));
//
//    shared_data()._valve_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(valve_center_pose_msg->pose_stamped));
//    shared_data()._valve_pose->pose.position.z = shared_data()._valve_pose->pose.position.z + CENTER_SHIFT;
//
//    shared_data().calcValveKeyPoses();


    std::cout << "----------State Machine----------" << std::endl;
    std::cout << "Current State:   Homing"        << std::endl;
    std::cout << "* success   ->   ValveReach"    << std::endl;
    std::cout << "* fail      ->   Homing"        << std::endl;
    std::cout << "---------------------------------" << std::endl;

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing::run(double time, double period){


    // blocking reading: wait for a command
   if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // Homing Succeeded
    if (!shared_data().current_command->str().compare("success"))
      transit("ValveReach");

    // Homing failed
    if (!shared_data().current_command->str().compare("fail"))
      transit("Homing");

  }

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing::exit (){

}

/********************************* END Homing ********************************/




/****************************** BEGIN ValveReach *****************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveReach::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveReach::entry(const XBot::FSM::Message& msg){
    std::cout << "ValveReach::entry()" << std::endl;

    shared_data().plugin_status->setStatus("VALVEREACH");
      

//    std::cout << "Select the End Effector you want to use." << std::endl;
    
    // blocking call: wait for a msg on topic hand_selection
    //    shared_data()._hand_selection = ros::topic::waitForMessage<std_msgs::String>("hand_selection");
    

//    std::string selectedHand = shared_data().selectedHand_;
//    std::cout << "Hand selected: " << selectedHand << std::endl;


//    // blocking call: wait for a pose on topic debris_pose
//    ADVR_ROS::im_pose_msg::ConstPtr tmp;
//    tmp = ros::topic::waitForMessage<ADVR_ROS::im_pose_msg>("valve_pose");
//    shared_data()._valve_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(tmp->pose_stamped));
//
//    std::cout << "_valve_pose: " << shared_data()._valve_pose << std::endl;


    //CALL SERVICE TO MOVE

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    if(shared_data().selectedHand_ == "RSoftHand")
      start_frame = shared_data().right_hand_pose_home_PoseStamped_;
    else if(shared_data().selectedHand_ == "LSoftHand")
      start_frame = shared_data().left_hand_pose_home_PoseStamped_;

    trajectory_utils::Cartesian start;
    start.distal_frame = shared_data().selectedHand_;
    start.frame = start_frame;    
    
    // define the intermediate frame
    geometry_msgs::PoseStamped intermediate_frame;
    intermediate_frame = *shared_data()._valve_pose;
//    intermediate_frame = shared_data().valve_left_pose_PoseStamped_;

    if(shared_data().selectedHand_ == "RSoftHand"){
    intermediate_frame.pose.position.y-= APPROCHING_SHIFT;
    }
    if(shared_data().selectedHand_ == "LSoftHand"){
    intermediate_frame.pose.position.y+= APPROCHING_SHIFT;
    }
    
    trajectory_utils::Cartesian intermediate;
    intermediate.distal_frame = shared_data().selectedHand_;
    intermediate.frame = intermediate_frame;
    
    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = TRAJ_DURATION;         // traj duration 5 second      
    s1.start = start;        // start pose
    s1.end = intermediate;            // intermediate pose     
    
    
    // define the end frame
    geometry_msgs::PoseStamped end_frame;
    end_frame = *shared_data()._valve_pose;
    
    
    trajectory_utils::Cartesian end;
    end.distal_frame = shared_data().selectedHand_;
    end.frame = end_frame;    

    shared_data()._last_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
    
    // define the second segment
    trajectory_utils::segment s2;
    s2.type.data = 0;        // min jerk traj
    s2.T.data = TRAJ_DURATION;         // traj duration 5 second      
    s2.start = intermediate;        // start pose
    s2.end = end;            // end pose 
    
    // only one segment in this example
    std::vector<trajectory_utils::segment> segments;
    segments.push_back(s1);
    segments.push_back(s2);
    
    // prepare the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = "world_odom";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);



    std::cout << "State Machine Current State: ValveReach" << std::endl;
    std::cout << "State Machine Transition: success->ValveTurn, fail->Homing" << std::endl;


}

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveReach::run(double time, double period){
  
  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // ValveReach failed
    if (!shared_data().current_command->str().compare("fail"))
      transit("Homing");
    
    // ValveReach Succeeded
    if (!shared_data().current_command->str().compare("success"))
      transit("ValveTurn");
  } 
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveReach::exit (){

}

/****************************** END ValveReach *******************************/




/****************************** BEGIN ValveTurn ******************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveTurn::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveTurn::entry(const XBot::FSM::Message& msg){
    std::cout << "ValveTurn::entry()" << std::endl;

    shared_data().plugin_status->setStatus("VALVETURN");
      

//    std_msgs::String message;
//    message = *shared_data()._hand_selection;
//    std::string selectedHand;
//    selectedHand = message.data;
    std::string selectedHand = shared_data().selectedHand_;

    //CALL SERVICE TO MOVE

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    start_frame = *shared_data()._valve_pose;

    trajectory_utils::Cartesian start;
    start.distal_frame = selectedHand;
    start.frame = start_frame;
    
    // define the end frame
    double rot = M_PI_2;
    if(selectedHand == "LSoftHand"){
            rot = -M_PI_2;
    }
    KDL::Frame end_frame_kdl;
    end_frame_kdl.Identity();
    end_frame_kdl.M = end_frame_kdl.M.Quaternion(start_frame.pose.orientation.x,
    start_frame.pose.orientation.y, start_frame.pose.orientation.z,
    start_frame.pose.orientation.w);
    end_frame_kdl.M.DoRotZ(rot);
    
    std_msgs::Float32 angle_rot;
    angle_rot.data = rot;
    
    geometry_msgs::PoseStamped end_frame;
    double qx,qy,qz,qw;
    end_frame_kdl.M.GetQuaternion(qx,qy,qz,qw);
    end_frame.pose.orientation.x = qx;
    end_frame.pose.orientation.y = qy;
    end_frame.pose.orientation.z = qz;
    end_frame.pose.orientation.w = qw;
    

    geometry_msgs::Vector3 plane_normal;
    plane_normal.x = -1;
    plane_normal.y = 0;
    plane_normal.z = 0;
//     if(selectedHand == "LSoftHand"){
//            plane_normal.x = 1;
//     }
    
    geometry_msgs::Vector3 circle_center;
    circle_center.x = start_frame.pose.position.x;
    circle_center.y = start_frame.pose.position.y;
    circle_center.z = start_frame.pose.position.z-VALVE_RADIUSE;
    
    
    
    trajectory_utils::Cartesian end;
    end.distal_frame = selectedHand;
    end.frame = end_frame;

    shared_data()._last_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
    
    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 1;        // arc traj
    s1.T.data = TRAJ_DURATION;         // traj duration 5 second      
    s1.start = start;        // start pose
    //s1.end = end;            // end pose 
    s1.end_rot = end.frame.pose.orientation;
    s1.angle_rot = angle_rot;
    s1.circle_center = circle_center;
    s1.plane_normal = plane_normal;
    
    // only one segment in this example
    std::vector<trajectory_utils::segment> segments;
    segments.push_back(s1);
    
    // prepare the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = "world_odom";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);
    
    //compute last pose
    geometry_msgs::PoseStamped last_frame;
    last_frame = *shared_data()._valve_pose;
    
    if(selectedHand == "RSoftHand"){
        last_frame.pose.position.y+=VALVE_RADIUSE;
	last_frame.pose.position.z-=VALVE_RADIUSE;
    }
    if(selectedHand == "LSoftHand"){
        last_frame.pose.position.y-=VALVE_RADIUSE;
	last_frame.pose.position.z-=VALVE_RADIUSE;
    }

    last_frame.pose.orientation = end_frame.pose.orientation;


    shared_data()._last_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(last_frame));
  

  

    std::cout << "State Machine Current State: ValveTurn" << std::endl;
    std::cout << "State Machine Transition: success->ValveGoBack, fail->Homing" << std::endl;

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveTurn::run(double time, double period){
  
  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // ValveTurn failed
    if (!shared_data().current_command->str().compare("fail"))
      transit("Homing");
    
    // ValveTurn Succeeded
    if (!shared_data().current_command->str().compare("success"))
      transit("ValveGoBack");
  } 
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveTurn::exit (){

}

/****************************** END ValveTurn *******************************/




/****************************** BEGIN ValveGoBack ******************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveGoBack::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveGoBack::entry(const XBot::FSM::Message& msg){
    std::cout << "ValveGoBack::entry()" << std::endl;

    shared_data().plugin_status->setStatus("VALVEGOBACK");
      
//    std::cout << "ValveGoBack_entry" << std::endl;

//    std_msgs::String message;
//    message = *shared_data()._hand_selection;
//    std::string selectedHand;
//    selectedHand = message.data;

    std::string selectedHand = shared_data().selectedHand_;

    //CALL SERVICE TO MOVE

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    start_frame = *shared_data()._last_pose;

    trajectory_utils::Cartesian start;
    start.distal_frame = selectedHand;
    start.frame = start_frame;    
    
    // define the intermediate frame
    geometry_msgs::PoseStamped intermediate_frame;
    intermediate_frame = *shared_data()._valve_pose;
//     intermediate_frame.pose.position.z += 0.2;
    
    
    trajectory_utils::Cartesian intermediate;
    intermediate.distal_frame = selectedHand;
    intermediate.frame = intermediate_frame;
    
    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = TRAJ_DURATION;         // traj duration 5 second      
    s1.start = start;        // start pose
    s1.end = intermediate;            // intermediate pose     
    
    
    // define the end frame
    geometry_msgs::PoseStamped end_frame;
    end_frame = *shared_data()._valve_pose;
    
    if(selectedHand == "RSoftHand"){
    end_frame.pose.position.y-= RETREAT_SHIFT;
    }
    if(selectedHand == "LSoftHand"){
    end_frame.pose.position.y+= RETREAT_SHIFT;
    }
    
    trajectory_utils::Cartesian end;
    end.distal_frame = selectedHand;
    end.frame = end_frame;    

    shared_data()._last_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
    
    // define the second segment
    trajectory_utils::segment s2;
    s2.type.data = 0;        // min jerk traj
    s2.T.data = TRAJ_DURATION;         // traj duration 5 second      
    s2.start = intermediate;        // start pose
    s2.end = end;            // end pose 
    
    // only one segment in this example
    std::vector<trajectory_utils::segment> segments;
    segments.push_back(s1);
    segments.push_back(s2);
    
    // prepare the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = "world_odom";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);



    std::cout << "State Machine Current State: ValveGoBack" << std::endl;
    std::cout << "State Machine Transition: success->Homing, fail->Homing" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveGoBack::run(double time, double period){
  
  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // ValveGoBack failed
    if (!shared_data().current_command->str().compare("fail"))
      transit("Homing");
    
    // ValveGoBack Succeeded
    if (!shared_data().current_command->str().compare("success"))
      transit("Homing");
  } 
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveGoBack::exit (){

}

/****************************** END ValveGoBack *******************************/


