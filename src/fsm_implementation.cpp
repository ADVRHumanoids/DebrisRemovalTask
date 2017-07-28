#include "fsm_definition.h"

#include <vector>
#include <string>

#include <eigen_conversions/eigen_msg.h>


/******************************** BEGIN Homing *******************************/
///////////////////////////////////////////////////////////////////////////////

void myfsm::Homing::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing::entry(const XBot::FSM::Message& msg){

    std::cout << "Homing_entry" << std::endl;
  
  
    //CALL SERVICE TO MOVE
    // send a trajectory for the end effector as a segment
    
    shared_data()._robot->sense(); 
    
    Eigen::Affine3d world_T_bl;
    std::string fb;  
    
    shared_data()._robot->model().getFloatingBaseLink(fb);
    tf.getTransformTf(fb, "world_odom", world_T_bl);
   
    shared_data()._robot->model().setFloatingBasePose(world_T_bl);
    shared_data()._robot->model().update();  
    
    
    // RIGHT HAND
    Eigen::Affine3d poseRightHand,poseLeftHand;
    geometry_msgs::Pose right_hand_pose,left_hand_pose;

    shared_data()._robot->model().getPose("RSoftHand", poseRightHand);
    shared_data()._robot->model().getPose("LSoftHand", poseLeftHand);
    
    tf::poseEigenToMsg (poseRightHand, right_hand_pose);
    tf::poseEigenToMsg (poseLeftHand, left_hand_pose);

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    start_frame.pose = right_hand_pose;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = "RSoftHand";
    start.frame = start_frame;
    
    // define the end frame - RIGHT HAND
    geometry_msgs::PoseStamped end_frame;
    
    end_frame.pose.position.x = 0.306;
    end_frame.pose.position.y = -0.393;
    end_frame.pose.position.z = 0.978;     
    
    end_frame.pose.orientation.x = -0.068;
    end_frame.pose.orientation.y = -0.534;
    end_frame.pose.orientation.z = 0.067;
    end_frame.pose.orientation.w = 0.840;   
    
    trajectory_utils::Cartesian end;
    end.distal_frame = "RSoftHand";
    end.frame = end_frame;
    
    //save the hand poses
    shared_data()._last_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));

    geometry_msgs::PoseStamped leftHandFrame;
    leftHandFrame.pose = left_hand_pose;
    shared_data()._initial_pose_right_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
    shared_data()._initial_pose_left_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(leftHandFrame));


    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = 25.0;         // traj duration 5 second      
    s1.start = start;        // start pose
    s1.end = end;            // end pose 
    
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
    
    std::cout << "Homing run. 'homing_fail'-> Homing\t\t'homing_sucess'->Reached\t\t" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing::run(double time, double period){
  
  // blocking reading: wait for a command
  if(shared_data().command.read(shared_data().current_command))
  {
    std::cout << "Command: " << shared_data().current_command.str() << std::endl;

    // Homing failed
    if (!shared_data().current_command.str().compare("homing_fail"))
      transit("Homing");
    
    // Homing Succeeded
    if (!shared_data().current_command.str().compare("homing_success"))
      transit("Reached");
  }

}


///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing::exit (){

}

/********************************* END Homing ********************************/


/******************************* BEGIN Reached *******************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::Reached::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Reached::entry(const XBot::FSM::Message& msg){

    std::cout << "Reached_entry" << std::endl;
    
    std::cout << "Select the End Effector you want to use." << std::endl;
    
    // blocking call: wait for a msg on topic hand_selection
    shared_data()._hand_selection = ros::topic::waitForMessage<std_msgs::String>("hand_selection");
    
    std_msgs::String message;
    message = *shared_data()._hand_selection;    
    std::string selectedHand;
    selectedHand = message.data;
    
    if(selectedHand.compare("RSoftHand") || selectedHand.compare("LSoftHand"))
      std::cout << "Select the pose where the debris is." << std::endl;
    else
      std::cout << "Incorrect input, you need to publish a different message" << std::endl;
      
    //in the future the position to reach the debris will be given by a ros message published on the rostopic "debris_pose"
    
    // blocking call: wait for a pose on topic debris_pose
    ADVR_ROS::im_pose_msg::ConstPtr tmp;
    tmp = ros::topic::waitForMessage<ADVR_ROS::im_pose_msg>("debris_pose");

    shared_data()._debris_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(tmp->pose_stamped));
    
//     geometry_msgs::PoseStamped poseDebris1;
//     poseDebris1.pose.position.x = 0.619;
//     poseDebris1.pose.position.y = -0.29;
//     poseDebris1.pose.position.z = 0.873;
//     
//     poseDebris1.pose.orientation.x = 0;
//     poseDebris1.pose.orientation.y = -0.5591931143131625;
//     poseDebris1.pose.orientation.z = 0;
//     poseDebris1.pose.orientation.w = 0.8290374303399975;
//     
//     shared_data()._debris_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(poseDebris1));
    
    
    //CALL SERVICE TO MOVE

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    if(selectedHand.compare("RSoftHand"))
      start_frame = *shared_data()._initial_pose_right_hand;
    else if(selectedHand.compare("LSoftHand"))
      start_frame = *shared_data()._initial_pose_left_hand;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = selectedHand;
    start.frame = start_frame;    
    
    // define the end frame
    geometry_msgs::PoseStamped end_frame;
    end_frame = *shared_data()._debris_pose;
    
    
    trajectory_utils::Cartesian end;
    end.distal_frame = selectedHand;
    end.frame = end_frame;

    shared_data()._last_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
    
    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = 25.0;         // traj duration 5 second      
    s1.start = start;        // start pose
    s1.end = end;            // end pose 
    
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

    std::cout << "Reached run. 'reached_fail'-> Homing\t\t'reached_sucess'->Grasped"<< std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Reached::run(double time, double period){

  // blocking reading: wait for a command
  if(shared_data().command.read(shared_data().current_command))
  {
    std::cout << "Command: " << shared_data().current_command.str() << std::endl;

    // Reached failed
    if (!shared_data().current_command.str().compare("reached_fail"))
      transit("Homing");
    
    // Reached Succeeded
    if (!shared_data().current_command.str().compare("reached_success"))
      transit("Grasped");
  } 

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Reached::exit (){

}

/******************************** END Reached ********************************/


/******************************* BEGIN Grasped *******************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::Grasped::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Grasped::entry(const XBot::FSM::Message& msg){

    std::cout << "Grasped_entry" << std::endl;
    
    //HAND
    //Hand selection
    std_msgs::String message;
    message = *shared_data()._hand_selection;    
    std::string selectedHand;
    selectedHand = message.data;
    
    std::cout << "SelectedHand: " << message.data << std::endl;
    
    //try to grasp
    
    ADVR_ROS::advr_grasp_control_srv srv;
  
    if(!selectedHand.compare("RSoftHand")){
      if(!shared_data()._hand_over_phase){
          srv.request.right_grasp = 1.0;
          srv.request.left_grasp = 0.0;
      }else{
          srv.request.right_grasp = 1.0;
          srv.request.left_grasp = 1.0;
          shared_data()._hand_over_phase = false;
      }
    }else if(!selectedHand.compare("LSoftHand")){
          srv.request.right_grasp = 0.0;
          srv.request.left_grasp = 1.0;
    }
    
    // call the service
    shared_data()._grasp_client.call(srv);    
      
    std::cout << "Grasped run. 'grasped_fail'-> Grasped\t\t'grasped_success'->Picked\t\t'move_away_after_ho'->MovedAway" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Grasped::run(double time, double period){

  //Wait for the trajectory to be completed
  if(!shared_data()._feedback){
    
    // blocking reading: wait for a command
    if(shared_data().command.read(shared_data().current_command))
    {
      std::cout << "Command: " << shared_data().current_command.str() << std::endl;

      // Grasped failed
      if (!shared_data().current_command.str().compare("grasped_fail"))
        transit("Grasped");
      
      // Grasped Succeeded
      if (!shared_data().current_command.str().compare("grasped_success"))
        transit("Picked");
      
      // Movedaway after handover
      if (!shared_data().current_command.str().compare("move_away_after_ho")){
        //Ungrasp left hand
        ADVR_ROS::advr_grasp_control_srv srv;
        srv.request.right_grasp = 1.0;
        srv.request.left_grasp = 0.0;
        // call the service
        shared_data()._grasp_client.call(srv);
        
        transit("MovedAway");    
      }
    } 
  }
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Grasped::exit (){

}

/********************************* END Grasped ******************************/


/******************************* BEGIN Picked *******************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::Picked::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Picked::entry(const XBot::FSM::Message& msg){

    std::cout << "Picked_entry" << std::endl;
  
    //CALL SERVICE TO MOVE

    //HAND
    //Hand selection
    std_msgs::String message;
    message = *shared_data()._hand_selection;    
    std::string selectedHand;
    selectedHand = message.data;
    
    std::cout << "SelectedHand: " << message.data << std::endl;

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    start_frame = *shared_data()._last_pose;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = selectedHand;
    start.frame = start_frame;
    
    // define the end frame
    geometry_msgs::PoseStamped end_frame;
    
    if(!selectedHand.compare("RSoftHand")){
      
      end_frame.pose.position.x = 0.352;
      end_frame.pose.position.y = -0.2;
      end_frame.pose.position.z = 1.00;   
      
      end_frame.pose.orientation.x = 0.225;
      end_frame.pose.orientation.y = -0.592;
      end_frame.pose.orientation.z = 0.432;
      end_frame.pose.orientation.w = 0.641;  
      
    }else if(!selectedHand.compare("LSoftHand")){

      end_frame.pose.position.x = 0.352;
      end_frame.pose.position.y = 0.03;
      end_frame.pose.position.z = 1.00;   

      end_frame.pose.orientation.x = -0.225;
      end_frame.pose.orientation.y = -0.592;
      end_frame.pose.orientation.z = -0.432;
      end_frame.pose.orientation.w = 0.641;      
      
    }
       

    trajectory_utils::Cartesian end;
    end.distal_frame = selectedHand;
    end.frame = end_frame;
    
    shared_data()._last_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));

    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = 25.0;         // traj duration 5 second      
    s1.start = start;        // start pose
    s1.end = end;            // end pose 
    
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
    
    std::cout << "Picked run. 'picked_fail'-> Homing\t\t'picked_success'->MovedAway\t\t'pick_second_hand'->PickSecondHand" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Picked::run(double time, double period){

  // blocking reading: wait for a command
  if(shared_data().command.read(shared_data().current_command))
  {
    std::cout << "Command: " << shared_data().current_command.str() << std::endl;

    // Picked failed
    if (!shared_data().current_command.str().compare("picked_fail"))
      transit("Homing");
    
    // Picked Succeeded
    if (!shared_data().current_command.str().compare("picked_success"))
      transit("MovedAway");
    
    // Pick second hand
    if (!shared_data().current_command.str().compare("pick_second_hand"))
      transit("PickSecondHand");
  } 
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Picked::exit (){

}

/********************************* END Picked *******************************/


/*************************** BEGIN PickSecondHand ***************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::PickSecondHand::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::PickSecondHand::entry(const XBot::FSM::Message& msg){

    std::cout << "PickSecondHand entry" << std::endl;
  
    //CALL SERVICE TO MOVE

    //HAND
    //Hand selection
    std_msgs::String message;
    message = *shared_data()._hand_selection;    
    std::string holdingHand;
    holdingHand = message.data;
    
    std::string secondHand;

    if(!holdingHand.compare("RSoftHand"))
      secondHand = "LSoftHand";
    else if(!holdingHand.compare("LSoftHand"))
      secondHand = "RSoftHand";
    
    std::cout << "holdingHand: " << holdingHand << std::endl;
    std::cout << "secondHand: " << secondHand << std::endl;

    
    Eigen::Affine3d poseSecondHand;
    geometry_msgs::Pose start_frame_pose;

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    start_frame = *shared_data()._initial_pose_right_hand;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = secondHand;
    start.frame = start_frame;
    
    // define the intermediate frame
    geometry_msgs::PoseStamped intermediate_frame;
    
    Eigen::Affine3d poseHoldingHand,poseSecondHandFinal,poseHoldingHand_Affine;
    geometry_msgs::Pose start_frame_pose_holding_hand;

    KDL::Frame poseHoldingHand_KDL;
    
    geometry_msgs::PoseStamped poseStampedHoldingHand;
    poseStampedHoldingHand = *shared_data()._last_pose;
    
    tf::poseMsgToEigen(poseStampedHoldingHand.pose,poseHoldingHand);
    
    poseHoldingHand_KDL.M = poseHoldingHand_KDL.M.Quaternion(poseStampedHoldingHand.pose.orientation.x,
      poseStampedHoldingHand.pose.orientation.y, poseStampedHoldingHand.pose.orientation.z,
      poseStampedHoldingHand.pose.orientation.w);
    
    poseHoldingHand_KDL.M.DoRotX(M_PI);
    poseHoldingHand_KDL.p.x(0.25);
    poseHoldingHand_KDL.p.y(0.10);
    poseHoldingHand_KDL.p.z(0);

    tf::transformKDLToEigen(poseHoldingHand_KDL,poseHoldingHand_Affine);
    
    poseSecondHandFinal = poseHoldingHand * poseHoldingHand_Affine;

    tf::poseEigenToMsg (poseSecondHandFinal, start_frame_pose_holding_hand);

    double qx, qy,qz,qw;
    poseHoldingHand_KDL.M.GetQuaternion(qx,qy,qz,qw);
    

    if(!secondHand.compare("RSoftHand")){
      
      intermediate_frame.pose.position.x = start_frame_pose_holding_hand.position.x;
      intermediate_frame.pose.position.y = start_frame_pose_holding_hand.position.y;
      intermediate_frame.pose.position.z = start_frame_pose_holding_hand.position.z;
      
      intermediate_frame.pose.orientation.x = qx;
      intermediate_frame.pose.orientation.y = qy;
      intermediate_frame.pose.orientation.z = qz;
      intermediate_frame.pose.orientation.w = qw;        
      
    }else if(!secondHand.compare("LSoftHand")){
      
      //TO BE IMPLEMENTED, IF NEEDED
      
    }

    trajectory_utils::Cartesian intermediate;
    intermediate.distal_frame = secondHand;
    intermediate.frame = intermediate_frame;

    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = 25.0;         // traj duration 5 second      
    s1.start = start;        // start pose
    s1.end = intermediate;            // end pose 
    
    std::vector<trajectory_utils::segment> segments;
    segments.push_back(s1);

    // define the end frame
    geometry_msgs::PoseStamped end_frame;

    geometry_msgs::Pose start_frame_pose_second_hand;
    Eigen::Affine3d poseHoldingHand_2,poseSecondHandFinal_2, poseSecondHand_Affine;    
    
    KDL::Frame poseHoldingHand_KDL_2;

    tf::poseMsgToEigen(poseStampedHoldingHand.pose,poseHoldingHand_2);
    
    poseHoldingHand_KDL_2.M = poseHoldingHand_KDL_2.M.Quaternion(poseStampedHoldingHand.pose.orientation.x,
    poseStampedHoldingHand.pose.orientation.y, poseStampedHoldingHand.pose.orientation.z,
    poseStampedHoldingHand.pose.orientation.w);
    
    poseHoldingHand_KDL_2.M.DoRotX(M_PI);
    poseHoldingHand_KDL_2.p.x(0.25);
    poseHoldingHand_KDL_2.p.y(-0.05);
    poseHoldingHand_KDL_2.p.z(0);
    
    tf::transformKDLToEigen(poseHoldingHand_KDL_2,poseSecondHand_Affine);
    
    poseSecondHandFinal_2 = poseHoldingHand_2 * poseSecondHand_Affine;

    tf::poseEigenToMsg (poseSecondHandFinal_2, start_frame_pose_second_hand);

    poseHoldingHand_KDL_2.M.GetQuaternion(qx,qy,qz,qw);
    

    if(!secondHand.compare("RSoftHand")){
      
      end_frame.pose.position.x = start_frame_pose_second_hand.position.x;
      end_frame.pose.position.y = start_frame_pose_second_hand.position.y;
      end_frame.pose.position.z = start_frame_pose_second_hand.position.z;

      end_frame.pose.orientation.x = qx;
      end_frame.pose.orientation.y = qy;
      end_frame.pose.orientation.z = qz;
      end_frame.pose.orientation.w = qw;        
      
    }else if(!secondHand.compare("LSoftHand")){
      
      //TO BE IMPLEMENTED, IF NEEDED
      
    }

    trajectory_utils::Cartesian end;
    end.distal_frame = secondHand;
    end.frame = end_frame;

    shared_data()._last_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
    
    // define the second segment
    trajectory_utils::segment s2;
    s2.type.data = 0;        // min jerk traj
    s2.T.data = 25.0;         // traj duration 5 second      
    s2.start = intermediate;        // start pose
    s2.end = end;            // end pose 
    
    segments.push_back(s2);
    
    // prepare the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = "world_odom";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);
    
    std::cout << "PickSecondHand run. 'homing'-> Homing\t\t'pick_second_hand_success'->Grasped" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::PickSecondHand::run(double time, double period){
  
  // blocking reading: wait for a command
  if(shared_data().command.read(shared_data().current_command))
  {
    std::cout << "Command: " << shared_data().current_command.str() << std::endl;

    // Picked failed
    if (!shared_data().current_command.str().compare("pick_second_hand_fail"))
      transit("Homing");
    
    // Picked Succeeded
    if (!shared_data().current_command.str().compare("pick_second_hand_success")){
      
      std::cout << "Select the End Effector you want to use." << std::endl;    
      shared_data()._hand_selection = ros::topic::waitForMessage<std_msgs::String>("hand_selection");
      std_msgs::String message;
      message = *shared_data()._hand_selection;    
      std::string selectedHand;
      selectedHand = message.data;
      
      shared_data()._hand_over_phase = true;
      
      transit("Grasped");

    }
  } 
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::PickSecondHand::exit (){

}

/**************************** END PickSecondHand ****************************/


/****************************** BEGIN MovedAway *****************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::MovedAway::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::MovedAway::entry(const XBot::FSM::Message& msg){

    std::cout << "MovedAway_entry" << std::endl;
  
    //CALL SERVICE TO MOVE

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    
    start_frame = *shared_data()._last_pose;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = "RSoftHand";
    start.frame = start_frame;
    
    
    // define the intermediate frame 
    geometry_msgs::PoseStamped intermediate_frame;

    intermediate_frame.pose.position.x = 0.50;
    intermediate_frame.pose.position.y = -0.393;
    intermediate_frame.pose.position.z = 1.09;     
    
    intermediate_frame.pose.orientation.x = 0.0;
    intermediate_frame.pose.orientation.y = -0.7071070192004544;
    intermediate_frame.pose.orientation.z = 0.0;
    intermediate_frame.pose.orientation.w = 0.7071070192004544;     
    
    trajectory_utils::Cartesian intermediate;
    intermediate.distal_frame = "RSoftHand";
    intermediate.frame = intermediate_frame;    
    
    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = 25.0;         // traj duration 5 second      
    s1.start = start;        // start pose
    s1.end = intermediate;            // end pose 
    
    std::vector<trajectory_utils::segment> segments;
    segments.push_back(s1);    
    
    // define the end frame - RIGHT HAND
    geometry_msgs::PoseStamped end_frame;
    
    end_frame.pose.position.x = 0.451;
    end_frame.pose.position.y = -0.940;
    end_frame.pose.position.z = 1.05;
    
    end_frame.pose.orientation.x = -0.386;
    end_frame.pose.orientation.y = -0.429;
    end_frame.pose.orientation.z = -0.452;
    end_frame.pose.orientation.w = 0.678;    
 
    shared_data()._last_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));

    trajectory_utils::Cartesian end;
    end.distal_frame = "RSoftHand";
    end.frame = end_frame;

    // define the second segment
    trajectory_utils::segment s2;
    s2.type.data = 0;        // min jerk traj
    s2.T.data = 25.0;         // traj duration 5 second      
    s2.start = intermediate;        // start pose
    s2.end = end;            // end pose 

    segments.push_back(s2);
    
    // prepare the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = "world_odom";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);     

    std::cout << "MovedAway run. 'movedaway_fail'-> Homing\t\t'movedaway_success'->PlacedDown" << std::endl;

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::MovedAway::run(double time, double period){
  
  // blocking reading: wait for a command
  if(shared_data().command.read(shared_data().current_command))
  {
    std::cout << "Command: " << shared_data().current_command.str() << std::endl;

    // MovedAway failed
    if (!shared_data().current_command.str().compare("movedaway_fail"))
      transit("Homing");
    
    // MovedAway Succeeded
    if (!shared_data().current_command.str().compare("movedaway_success")){
      
      //TEMPORARY
      transit("Ungrasped");
      
      std::cout << "PROVAAAAAAAAAAAAAAAAAA" << std::endl;
      
      //Hand Pose to get the initial wrench for the PlacedDown state
      shared_data()._robot->sense(); 
    
      Eigen::Affine3d world_T_bl;
      std::string fb;  
      
      shared_data()._robot->model().getFloatingBaseLink(fb);
      tf.getTransformTf(fb, "world_odom", world_T_bl);
    
      shared_data()._robot->model().setFloatingBasePose(world_T_bl);
      shared_data()._robot->model().update();     
      
      // RIGHT HAND
      
      Eigen::Affine3d poseRightHand;
      shared_data()._robot->model().getPose("RSoftHand", poseRightHand);
      
      //Reading initial wrench from ros topic
      double f_x,f_y,f_z,w_Fz_ft;
      
      shared_data()._ft_r_arm = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>("/xbotcore/bigman/ft/r_arm_ft");
      
      f_x = shared_data()._ft_r_arm->wrench.force.x;
      f_y = shared_data()._ft_r_arm->wrench.force.y;
      f_z = shared_data()._ft_r_arm->wrench.force.z;
  
      Eigen::Vector3d ft_F_ft, w_F_ft;
      ft_F_ft << f_x, f_y, f_z;
      w_F_ft = poseRightHand * ft_F_ft;
      shared_data()._w_F_ft_initial = w_F_ft(2);
      
      transit("PlacedDown");
    }
  } 
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::MovedAway::exit (){

}

/******************************* END MovedAway ******************************/


/****************************** BEGIN PlacedDown ****************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::PlacedDown::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::PlacedDown::entry(const XBot::FSM::Message& msg){

    std::cout << "PlacedDown_entry" << std::endl;
        
    shared_data()._robot->sense(); 
    
    Eigen::Affine3d world_T_bl;
    std::string fb;  
    
    shared_data()._robot->model().getFloatingBaseLink(fb);
    tf.getTransformTf(fb, "world_odom", world_T_bl);
   
    shared_data()._robot->model().setFloatingBasePose(world_T_bl);
    shared_data()._robot->model().update();     
    
    // RIGHT HAND
    
    Eigen::Affine3d poseRightHand;
    geometry_msgs::Pose start_frame_pose;

    shared_data()._robot->model().getPose("RSoftHand", poseRightHand);
    tf::poseEigenToMsg (poseRightHand, start_frame_pose);

    geometry_msgs::PoseStamped poseHandStamped;
    poseHandStamped.pose = start_frame_pose;
    poseHandStamped.pose.position.z-=0.000001;
    
    //publish ros message
    shared_data()._SoftHandPose_pub.publish (poseHandStamped);

}


///////////////////////////////////////////////////////////////////////////////
void myfsm::PlacedDown::run(double time, double period){
  
    shared_data()._robot->sense(); 
    
    Eigen::Affine3d world_T_bl;
    std::string fb;  
    
    shared_data()._robot->model().getFloatingBaseLink(fb);
    tf.getTransformTf(fb, "world_odom", world_T_bl);
   
    shared_data()._robot->model().setFloatingBasePose(world_T_bl);
    shared_data()._robot->model().update();     
    
    // RIGHT HAND
    
    Eigen::Affine3d poseRightHand;
    geometry_msgs::Pose start_frame_pose;

    shared_data()._robot->model().getPose("RSoftHand", poseRightHand);
    
    //Reading initial wrench from ros topic
    double f_x,f_y,f_z,w_Fz_ft;
    
    shared_data()._ft_r_arm = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>("/xbotcore/bigman/ft/r_arm_ft");
    
    f_x = shared_data()._ft_r_arm->wrench.force.x;
    f_y = shared_data()._ft_r_arm->wrench.force.y;
    f_z = shared_data()._ft_r_arm->wrench.force.z;

    Eigen::Vector3d ft_F_ft,w_F_ft;
    ft_F_ft << f_x, f_y, f_z;
    w_F_ft = poseRightHand * ft_F_ft;
    w_Fz_ft = w_F_ft(2);
      
//     w_Fz_ft = shared_data()._RH_Rot_Z.dot(ft_F_ft);
      
//     double k;
//     k = 0.7;

    std::cout << "w_Fz_ft: " << w_Fz_ft << std::endl;

    if(w_Fz_ft <= 50) //k * shared_data()._w_F_ft_initial)
      transit("PlacedDown");
    else
      transit("Ungrasped");
    
      // blocking reading: wait for a command
  if(shared_data().command.read(shared_data().current_command))
  {
    std::cout << "Command: " << shared_data().current_command.str() << std::endl;

    // Ungrasped failed
    if (!shared_data().current_command.str().compare("placeddown_success"))
      transit("Ungrasped");
    
    // Ungrasped Succeeded
    if (!shared_data().current_command.str().compare("placeddown_failed"))
      transit("Homing");
  }
  
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::PlacedDown::exit (){

}

/****************************** END PlacedDown ******************************/


/****************************** BEGIN Ungrasped *****************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::Ungrasped::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Ungrasped::entry(const XBot::FSM::Message& msg){

  std::cout << "Ungrasped_entry" << std::endl;
  
  //CALL SERVICE TO UNGRASP  

  ADVR_ROS::advr_grasp_control_srv srv;
  srv.request.right_grasp = 0.0;
  srv.request.left_grasp = 0.0;
  // call the service
  shared_data()._grasp_client.call(srv);
  
  std::cout << "Ungrasped run. 'ungrasped_fail'-> Ungrasped\t\t'ungrasped_success'->Homing" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Ungrasped::run(double time, double period){
  
  // blocking reading: wait for a command
  if(shared_data().command.read(shared_data().current_command))
  {
    std::cout << "Command: " << shared_data().current_command.str() << std::endl;

    // Ungrasped failed
    if (!shared_data().current_command.str().compare("ungrasped_fail"))
      transit("Ungrasped");
    
    // Ungrasped Succeeded
    if (!shared_data().current_command.str().compare("ungrasped_success"))
      transit("Homing");
  } 
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Ungrasped::exit (){

}

/****************************** END Ungrasped *******************************/
