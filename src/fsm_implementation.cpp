#include "fsm_definition.h"

#include <vector>
#include <string>

#include <eigen_conversions/eigen_msg.h>

#define TRAJ_DURATION 10


/******************************** BEGIN Homing_init *******************************/
///////////////////////////////////////////////////////////////////////////////

void myfsm::Homing_init::react(const XBot::FSM::Event& e) {
    
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing_init::entry(const XBot::FSM::Message& msg){

    shared_data().plugin_status->setStatus("HOMING");
    std::cout << "Homing_init_entry" << std::endl;
    shared_data()._robot->sense(); 
    
    Eigen::Affine3d world_T_bl;
    std::string fb;  
    shared_data()._robot->model().getFloatingBaseLink(fb);
    tf.getTransformTf(fb, "world_odom", world_T_bl);
    shared_data()._robot->model().setFloatingBasePose(world_T_bl);
    shared_data()._robot->model().update();  
    
    
    // SAVE INITIAL END EFFECTOR POSES
    Eigen::Affine3d poseLeftHand,poseRightHand;
    geometry_msgs::Pose left_hand_pose,right_hand_pose;

    shared_data()._robot->model().getPose("LSoftHand", poseLeftHand);
    shared_data()._robot->model().getPose("RSoftHand", poseRightHand);
    
    tf::poseEigenToMsg (poseLeftHand, left_hand_pose);
    tf::poseEigenToMsg (poseRightHand, right_hand_pose);

    geometry_msgs::PoseStamped leftHandFrame,rightHandFrame;
    leftHandFrame.pose = left_hand_pose;
    rightHandFrame.pose = right_hand_pose;
    shared_data()._initial_pose_left_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(leftHandFrame));
    shared_data()._initial_pose_right_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(rightHandFrame));
    shared_data()._last_pose_left_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(leftHandFrame));
    shared_data()._last_pose_right_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(rightHandFrame));

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing_init::run(double time, double period){
  
  transit("Homing_Ree");

}


///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing_init::exit (){

}

/********************************* END Homing_init ********************************/


/******************************** BEGIN Homing_Ree *******************************/
///////////////////////////////////////////////////////////////////////////////

void myfsm::Homing_Ree::react(const XBot::FSM::Event& e) {
    
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing_Ree::entry(const XBot::FSM::Message& msg){

    shared_data().plugin_status->setStatus("HOMING_REE");
    
    //CALL SERVICE TO MOVE - RIGHT HAND
    // send a trajectory for the end effector as a segment

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    start_frame = *shared_data()._last_pose_right_hand;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = "RSoftHand";
    start.frame = start_frame;
    
    // define the end frame
    geometry_msgs::PoseStamped end_frame;
//     end_frame = *shared_data()._initial_pose_right_hand;
    end_frame.pose.position.x = 0.248;
    end_frame.pose.position.y = -0.471;
    end_frame.pose.position.z = 0.969;     
    
    end_frame.pose.orientation.x = -0.091;
    end_frame.pose.orientation.y = -0.456;
    end_frame.pose.orientation.z = 0.19;
    end_frame.pose.orientation.w = 0.864;
    
    trajectory_utils::Cartesian end;
    end.distal_frame = "RSoftHand";
    end.frame = end_frame;
    
    //save the RIGHT HAND pose
    shared_data()._last_pose_right_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));

    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = TRAJ_DURATION;         // traj duration 5 second      
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
    
    std::cout << "Homing_Ree run. Homing_Ree--->Homing_Lee"<< std::endl;
    
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing_Ree::run(double time, double period){
  
  //Wait for the trajectory to be completed
  if(!shared_data()._feedback)
    transit("Homing_Lee");

}


///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing_Ree::exit (){

}

/********************************* END Homing_Ree ********************************/



/******************************** BEGIN Homing_Lee *******************************/
///////////////////////////////////////////////////////////////////////////////

void myfsm::Homing_Lee::react(const XBot::FSM::Event& e) {
    
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing_Lee::entry(const XBot::FSM::Message& msg){

    shared_data().plugin_status->setStatus("HOMING_LEE");
  
    //CALL SERVICE TO MOVE - LEFT HAND
    // send a trajectory for the end effector as a segment

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    start_frame = *shared_data()._last_pose_left_hand;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = "LSoftHand";
    start.frame = start_frame;
    
    // define the potential intermediate frame
    trajectory_utils::Cartesian intermediate;
    if(shared_data()._hand_over_phase){
      geometry_msgs::PoseStamped intermediate_frame;
      intermediate_frame = *shared_data()._last_pose_left_hand;
      intermediate_frame.pose.position.x+= 0.08;
      intermediate_frame.pose.position.y+= 0.08;
      
      intermediate.distal_frame = "LSoftHand";
      intermediate.frame = intermediate_frame;
    }
    
    // define the end frame
    geometry_msgs::PoseStamped end_frame;
//     end_frame = *shared_data()._initial_pose_left_hand;
    end_frame.pose.position.x = 0.248;
    end_frame.pose.position.y = 0.471;
    end_frame.pose.position.z = 0.969;     
    
    end_frame.pose.orientation.x = 0.091;
    end_frame.pose.orientation.y = -0.456;
    end_frame.pose.orientation.z = -0.19;
    end_frame.pose.orientation.w = 0.864;     
    
    trajectory_utils::Cartesian end;
    end.distal_frame = "LSoftHand";
    end.frame = end_frame;
    
    //save the LEFT HAND pose
    shared_data()._last_pose_left_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));

    // define the first segment
    trajectory_utils::segment s1,s2;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = TRAJ_DURATION;         // traj duration 5 second      
    s1.start = start;        // start pose
    if(shared_data()._hand_over_phase){
      s1.end = intermediate;
      s2.type.data = 0;        // min jerk traj
      s2.T.data = TRAJ_DURATION;         // traj duration 5 second      
      s2.start = intermediate;        // start pose
      s2.end = end;            // end pose 
    }else
      s1.end = end;            // end pose 
    
    std::vector<trajectory_utils::segment> segments;
    segments.push_back(s1);
    if(shared_data()._hand_over_phase)
      segments.push_back(s2);
    
    // prepare the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = "world_odom";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);    
    
    std::cout << "Homing_Lee run. 'success'->HandSelection\t'fail'->Homing\t'Handover_success'->MoveAway"<< std::endl;
    
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing_Lee::run(double time, double period){
  
  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // Homing_Lee failed
    if (!shared_data().current_command->str().compare("fail"))
      transit("Homing_Ree");
    
    // Homing_Lee succeeded
    if (!shared_data().current_command->str().compare("success"))
      transit("HandSelection");
    
    // Handover success
    if (!shared_data().current_command->str().compare("Handover_success"))
      transit("MoveAway");    
  }

}


///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing_Lee::exit (){

}

/********************************* END Homing_Lee ********************************/



/******************************* BEGIN HandSelection *******************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::HandSelection::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::HandSelection::entry(const XBot::FSM::Message& msg){

    shared_data().plugin_status->setStatus("HANDSELECTION");
      
    std::cout << "HandSelection_entry" << std::endl;
    
    std::cout << "Select the End Effector you want to use." << std::endl;


}

///////////////////////////////////////////////////////////////////////////////
void myfsm::HandSelection::run(double time, double period){

  std::string selectedHand;
  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // LSoftHand selected
    if (!shared_data().current_command->str().compare("LSoftHand")){
      std_msgs::String message;
      message.data = shared_data().current_command->str();
      shared_data()._hand_selection =  boost::shared_ptr<std_msgs::String>(new std_msgs::String(message));;
      transit("Reach");
    }
    // RSoftHand selected
    if (!shared_data().current_command->str().compare("RSoftHand")){
      std_msgs::String message;
      message.data = shared_data().current_command->str();
      shared_data()._hand_selection =  boost::shared_ptr<std_msgs::String>(new std_msgs::String(message));;
      transit("Reach");
    }
  } 

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::HandSelection::exit (){

}

/******************************** END HandSelection ********************************/


/******************************* BEGIN Reach *******************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::Reach::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Reach::entry(const XBot::FSM::Message& msg){

    shared_data().plugin_status->setStatus("REACH");
      
    std::cout << "Reach_entry" << std::endl;
    
    
    //HAND
    //Hand selection
    std_msgs::String message;
    message = *shared_data()._hand_selection;    
    std::string selectedHand;
    selectedHand = message.data;
    
    std::cout << "SelectedHand: " << message.data << std::endl;
    
    if(selectedHand.compare("RSoftHand") || selectedHand.compare("LSoftHand"))
      std::cout << "Select the pose where the debris is." << std::endl;
    else
      std::cout << "Incorrect input, you need to publish a different message" << std::endl;
      
    // blocking call: wait for a pose on topic debris_pose
    ADVR_ROS::im_pose_msg::ConstPtr tmp;
    tmp = ros::topic::waitForMessage<ADVR_ROS::im_pose_msg>("debris_pose");

    shared_data()._debris_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(tmp->pose_stamped));

    //CALL SERVICE TO MOVE


    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    if(!selectedHand.compare("RSoftHand"))
      start_frame = *shared_data()._last_pose_right_hand;
    else if(!selectedHand.compare("LSoftHand"))
      start_frame = *shared_data()._last_pose_left_hand;

    trajectory_utils::Cartesian start;
    start.distal_frame = selectedHand;
    start.frame = start_frame;    
    
    
    // define the intermediate frame 
    geometry_msgs::PoseStamped intermediate_frame;
    
    intermediate_frame = *shared_data()._debris_pose;

    if(!selectedHand.compare("RSoftHand")){
      intermediate_frame.pose.position.x-= 0.2;
      intermediate_frame.pose.position.y-= 0.2;
    }else if(!selectedHand.compare("LSoftHand")){
      intermediate_frame.pose.position.x-= 0.2;
      intermediate_frame.pose.position.y+= 0.2;
    }
    
    trajectory_utils::Cartesian intermediate;
    intermediate.distal_frame = selectedHand;
    intermediate.frame = intermediate_frame;    
    
    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = TRAJ_DURATION;         // traj duration 5 second      
    s1.start = start;        // start pose
    s1.end = intermediate;            // end pose 
    
    std::vector<trajectory_utils::segment> segments;
    segments.push_back(s1); 

    // define the end frame
    geometry_msgs::PoseStamped end_frame;
    end_frame = *shared_data()._debris_pose;
    
    
    trajectory_utils::Cartesian end;
    end.distal_frame = selectedHand;
    end.frame = end_frame;

    if(!selectedHand.compare("RSoftHand"))
      shared_data()._last_pose_right_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
    else if(!selectedHand.compare("LSoftHand"))
      shared_data()._last_pose_left_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
    
    
    // define the second segment
    trajectory_utils::segment s2;
    s2.type.data = 0;        // min jerk traj
    s2.T.data = TRAJ_DURATION;         // traj duration 5 second      
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

    std::cout << "Reach run. 'success'->Grasp\t'fail'->Homing_Ree\t'adjust'->Adjust" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Reach::run(double time, double period){

  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // Reach failed
    if (!shared_data().current_command->str().compare("fail"))
      transit("Homing_Ree");
    
    // Reach succeeded
    if (!shared_data().current_command->str().compare("success"))
      transit("Grasp");
    
    // Adjust
    if (!shared_data().current_command->str().compare("Adjust"))
      transit("Adjust");
  } 

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Reach::exit (){

}

/******************************** END Reach ********************************/


/******************************* BEGIN Grasp *******************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::Grasp::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Grasp::entry(const XBot::FSM::Message& msg){

    shared_data().plugin_status->setStatus("GRASP");
      
    std::cout << "Grasp_entry" << std::endl;
    
    //HAND
    //Hand selection
    std_msgs::String message;
    message = *shared_data()._hand_selection;    
    std::string selectedHand;
    selectedHand = message.data;
    
    std::cout << "SelectedHand: " << message.data << std::endl;
    
    ADVR_ROS::advr_grasp_control_srv srv;
  
    if(!selectedHand.compare("RSoftHand")){
      if(!shared_data()._hand_over_phase){
          srv.request.right_grasp = 1.0;
          srv.request.left_grasp = 0.0;
      }else{
          srv.request.right_grasp = 1.0;
          srv.request.left_grasp = 1.0;
      }
    }else if(!selectedHand.compare("LSoftHand")){
          srv.request.right_grasp = 0.0;
          srv.request.left_grasp = 1.0;
    }
    
    // call the service
    shared_data()._grasp_client.call(srv);    
      
    std::cout << "Grasp run. 'success'->Pick\t'fail'-> Grasp\t'After_handover'->Left homing" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Grasp::run(double time, double period){

    // blocking reading: wait for a command
    if(!shared_data().current_command->str().empty())
    {
      std::cout << "Command: " << shared_data().current_command->str() << std::endl;

      // Grasp failed
      if (!shared_data().current_command->str().compare("fail") && !shared_data()._hand_over_phase)
        transit("Grasp");
      
      // Grasp Succeeded
      if (!shared_data().current_command->str().compare("success") && !shared_data()._hand_over_phase)
        transit("Pick");
      
      // Movedaway after handover
      if (!shared_data().current_command->str().compare("After_handover") && shared_data()._hand_over_phase){

//         shared_data()._hand_over_phase = false;
  
        //Ungrasp left hand
        ADVR_ROS::advr_grasp_control_srv srv;
        srv.request.right_grasp = 1.0;
        srv.request.left_grasp = 0.0;
        // call the service
        shared_data()._grasp_client.call(srv);
        
        transit("Homing_Lee");    
      }
    } 
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Grasp::exit (){

}

/********************************* END Grasp ******************************/


/******************************* BEGIN Pick *******************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::Pick::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Pick::entry(const XBot::FSM::Message& msg){

    shared_data().plugin_status->setStatus("PICK");
    
    std::cout << "Pick_entry" << std::endl;
  
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
    if(!selectedHand.compare("RSoftHand"))
      start_frame = *shared_data()._last_pose_right_hand;
    else if(!selectedHand.compare("LSoftHand"))
      start_frame = *shared_data()._last_pose_left_hand;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = selectedHand;
    start.frame = start_frame;
    
    // define the end frame
    geometry_msgs::PoseStamped end_frame;
    
    if(!selectedHand.compare("RSoftHand")){
      
      end_frame.pose.position.x = 0.5;
      end_frame.pose.position.y = -0.2;
      end_frame.pose.position.z = 1.00;   
      
      end_frame.pose.orientation.x = 0.225;
      end_frame.pose.orientation.y = -0.592;
      end_frame.pose.orientation.z = 0.432;
      end_frame.pose.orientation.w = 0.641; 
      
      shared_data()._last_pose_right_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
      
    }else if(!selectedHand.compare("LSoftHand")){

//       end_frame.pose.position.x = 0.5;
//       end_frame.pose.position.y = 0.03;
//       end_frame.pose.position.z = 1.00;   
// 
//       end_frame.pose.orientation.x = -0.225;
//       end_frame.pose.orientation.y = -0.592;
//       end_frame.pose.orientation.z = -0.432;
//       end_frame.pose.orientation.w = 0.641;
      
      end_frame.pose.position.x = 0.4;
      end_frame.pose.position.y = 0.0;
      end_frame.pose.position.z = 1.00;   

      end_frame.pose.orientation.x = -0.5;
      end_frame.pose.orientation.y = -0.5;
      end_frame.pose.orientation.z = -0.5;
      end_frame.pose.orientation.w = 0.5;
      
      shared_data()._last_pose_left_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
      
    }
       

    trajectory_utils::Cartesian end;
    end.distal_frame = selectedHand;
    end.frame = end_frame;
    
    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = TRAJ_DURATION;         // traj duration 5 second      
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
    
    std::cout << "Pick run. 'success'->MoveAway\t'fail'->Homing\t'Handover'->PickSecondHand" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Pick::run(double time, double period){

  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // Pick failed
    if (!shared_data().current_command->str().compare("fail"))
      transit("Homing_Ree");
    
    // Pick Succeeded
    if (!shared_data().current_command->str().compare("success"))
      transit("MoveAway");
    
    // Pick second hand
    if (!shared_data().current_command->str().compare("Handover"))
      transit("PickSecondHand");
  } 
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Pick::exit (){

}

/********************************* END Pick *******************************/


/*************************** BEGIN PickSecondHand ***************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::PickSecondHand::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::PickSecondHand::entry(const XBot::FSM::Message& msg){

    shared_data().plugin_status->setStatus("PICKSECONDHAND");
    
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
    if(!holdingHand.compare("RSoftHand"))
      start_frame = *shared_data()._last_pose_left_hand;
    else if(!holdingHand.compare("LSoftHand"))
      start_frame = *shared_data()._last_pose_right_hand;    
    
    trajectory_utils::Cartesian start;
    start.distal_frame = secondHand;
    start.frame = start_frame;
    
    // define the intermediate1 frame
    geometry_msgs::PoseStamped intermediate_frame1;
    intermediate_frame1 = start_frame;
    intermediate_frame1.pose.position.x+= 0.3;

    
    trajectory_utils::Cartesian intermediate1;
    intermediate1.distal_frame = secondHand;
    intermediate1.frame = intermediate_frame1;

    // define the first segment
    trajectory_utils::segment s0;
    s0.type.data = 0;        // min jerk traj
    s0.T.data = TRAJ_DURATION;         // traj duration 5 second      
    s0.start = start;        // start pose
    s0.end = intermediate1;            // end pose 
    
    std::vector<trajectory_utils::segment> segments;
    segments.push_back(s0);
    
    
    
    // define the intermediate frame
    geometry_msgs::PoseStamped intermediate_frame;
    
    Eigen::Affine3d poseHoldingHand,poseSecondHandFinal,poseHoldingHand_Affine;
    geometry_msgs::Pose start_frame_pose_holding_hand;

    KDL::Frame poseHoldingHand_KDL;
    
    geometry_msgs::PoseStamped poseStampedHoldingHand;
    poseStampedHoldingHand = *shared_data()._last_pose_left_hand;
    
    tf::poseMsgToEigen(poseStampedHoldingHand.pose,poseHoldingHand);
    
    poseHoldingHand_KDL.M = poseHoldingHand_KDL.M.Quaternion(poseStampedHoldingHand.pose.orientation.x,
      poseStampedHoldingHand.pose.orientation.y, poseStampedHoldingHand.pose.orientation.z,
      poseStampedHoldingHand.pose.orientation.w);
    
    poseHoldingHand_KDL.M.DoRotX(M_PI);
    poseHoldingHand_KDL.p.x(0.25);
//     poseHoldingHand_KDL.p.y(0.10);
//     poseHoldingHand_KDL.p.z(-0.05);
    poseHoldingHand_KDL.p.y(0.10);
    poseHoldingHand_KDL.p.z(0.05);

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
    s1.T.data = TRAJ_DURATION;         // traj duration 5 second      
    s1.start = intermediate1;        // start pose
    s1.end = intermediate;            // end pose 
    
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
//     poseHoldingHand_KDL_2.p.y(-0.05);
//     poseHoldingHand_KDL_2.p.z(0.05);
    poseHoldingHand_KDL_2.p.y(-0.02);
    poseHoldingHand_KDL_2.p.z(0.0); //0.05
    
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

   
    if(!secondHand.compare("RSoftHand"))
      shared_data()._last_pose_right_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
    else if(!secondHand.compare("LSoftHand"))
      shared_data()._last_pose_left_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));    
    
    // define the second segment
    trajectory_utils::segment s2;
    s2.type.data = 0;        // min jerk traj
    s2.T.data = TRAJ_DURATION;         // traj duration 5 second      
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
    
    std::cout << "PickSecondHand run. 'success'->Grasp\t'fail'-> Homing" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::PickSecondHand::run(double time, double period){
  
  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // Pick failed
    if (!shared_data().current_command->str().compare("fail"))
      transit("Homing_Ree");
    
    // Pick Succeeded
    if (!shared_data().current_command->str().compare("success")){
      
      std::cout << "Changing selected Hand" << std::endl;

      std::string selectedHand;
      selectedHand = shared_data()._hand_selection->data;
      
      std_msgs::String rightHand,leftHand;
      rightHand.data = "RSoftHand";   
      leftHand.data = "LSoftHand";       
      
      if(!selectedHand.compare("RSoftHand"))
        shared_data()._hand_selection = boost::shared_ptr<std_msgs::String>(new std_msgs::String(leftHand));
      else if(!selectedHand.compare("LSoftHand"))
        shared_data()._hand_selection = boost::shared_ptr<std_msgs::String>(new std_msgs::String(rightHand));

      //Activate Handover phase      
      shared_data()._hand_over_phase = true;
      
      transit("Grasp");

    }
  } 
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::PickSecondHand::exit (){

}

/**************************** END PickSecondHand ****************************/


/****************************** BEGIN MoveAway *****************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::MoveAway::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::MoveAway::entry(const XBot::FSM::Message& msg){

    shared_data().plugin_status->setStatus("MOVEAWAY");
    
    std::cout << "MoveAway_entry" << std::endl;
  
    //CALL SERVICE TO MOVE
    
    //Hand selection
    std_msgs::String message;
    message = *shared_data()._hand_selection;    
    std::string selectedHand;
    selectedHand = message.data;
    
    std::cout << "SelectedHand: " << message.data << std::endl;

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    if(!selectedHand.compare("RSoftHand"))
      start_frame = *shared_data()._last_pose_right_hand;
    else if(!selectedHand.compare("LSoftHand"))
      start_frame = *shared_data()._last_pose_left_hand;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = selectedHand;
    start.frame = start_frame;
    
    
    // define the intermediate frame 
    geometry_msgs::PoseStamped intermediate_frame;

    intermediate_frame.pose.position.x = 0.50;
    
    if(!selectedHand.compare("RSoftHand"))
      intermediate_frame.pose.position.y = -0.393;
    else if(!selectedHand.compare("LSoftHand"))
      intermediate_frame.pose.position.y = 0.393;
    
    intermediate_frame.pose.position.z = 1.09;
    if(shared_data()._hand_over_phase)
      intermediate_frame.pose.position.z+= 0.15;
    
    intermediate_frame.pose.orientation.x = 0.0;
    intermediate_frame.pose.orientation.y = -0.7071070192004544;
    intermediate_frame.pose.orientation.z = 0.0;
    intermediate_frame.pose.orientation.w = 0.7071070192004544;     
    
    trajectory_utils::Cartesian intermediate;
    intermediate.distal_frame = selectedHand;
    intermediate.frame = intermediate_frame;    
    
    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = TRAJ_DURATION;         // traj duration 5 second      
    s1.start = start;        // start pose
    s1.end = intermediate;            // end pose 
    
    std::vector<trajectory_utils::segment> segments;
    segments.push_back(s1);    
    
    // define the end frame
    geometry_msgs::PoseStamped end_frame;
    
    if(!selectedHand.compare("RSoftHand")){
      
      end_frame.pose.position.x = 0.3; //0.4
      end_frame.pose.position.y = -0.75; //-0.68
      end_frame.pose.position.z = 1.05;
      
      if(shared_data()._hand_over_phase){
        end_frame.pose.position.z+= 0.15;
        shared_data()._hand_over_phase = false;
      }
    
      end_frame.pose.orientation.x = -0.386;
      end_frame.pose.orientation.y = -0.429;
      end_frame.pose.orientation.z = -0.452;
      end_frame.pose.orientation.w = 0.678;
      
    }
    else if(!selectedHand.compare("LSoftHand")){
      
      //TO BE IMPLEMENTED, IF NEEDED
      
    }
 
    
    if(!selectedHand.compare("RSoftHand"))
      shared_data()._last_pose_right_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
    else if(!selectedHand.compare("LSoftHand"))
      shared_data()._last_pose_left_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));    

    trajectory_utils::Cartesian end;
    end.distal_frame = selectedHand;
    end.frame = end_frame;

    // define the second segment
    trajectory_utils::segment s2;
    s2.type.data = 0;        // min jerk traj
    s2.T.data = TRAJ_DURATION;         // traj duration 5 second      
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

    std::cout << "MoveAway run. 'success'->PlaceDown\t'fail'-> Homing" << std::endl;

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::MoveAway::run(double time, double period){
  
  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // MoveAway failed
    if (!shared_data().current_command->str().compare("fail"))
      transit("Homing_Ree");
    
    // MoveAway Succeeded
    if (!shared_data().current_command->str().compare("success")){
      
      //TEMPORARY
      transit("PlaceDown");
      
//       std::cout << "PROVAAAAAAAAAAAAAAAAAA" << std::endl;
//       
//       //Hand Pose to get the initial wrench for the PlaceDown state
//       shared_data()._robot->sense(); 
//     
//       Eigen::Affine3d world_T_bl;
//       std::string fb;  
//       
//       shared_data()._robot->model().getFloatingBaseLink(fb);
//       tf.getTransformTf(fb, "world_odom", world_T_bl);
//     
//       shared_data()._robot->model().setFloatingBasePose(world_T_bl);
//       shared_data()._robot->model().update();     
//       
//       // RIGHT HAND
//       
//       Eigen::Affine3d poseRightHand;
//       shared_data()._robot->model().getPose("RSoftHand", poseRightHand);
//       
//       //Reading initial wrench from ros topic
//       double f_x,f_y,f_z,w_Fz_ft;
//       
//       shared_data()._ft_r_arm = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>("/xbotcore/bigman/ft/r_arm_ft");
//       
//       f_x = shared_data()._ft_r_arm->wrench.force.x;
//       f_y = shared_data()._ft_r_arm->wrench.force.y;
//       f_z = shared_data()._ft_r_arm->wrench.force.z;
//   
//       Eigen::Vector3d ft_F_ft, w_F_ft;
//       ft_F_ft << f_x, f_y, f_z;
//       w_F_ft = poseRightHand * ft_F_ft;
//       shared_data()._w_F_ft_initial = w_F_ft(2);
//       
//       transit("PlaceDown");
    }
  } 
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::MoveAway::exit (){

}

/******************************* END MoveAway ******************************/


/****************************** BEGIN PlaceDown ****************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::PlaceDown::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////

void myfsm::PlaceDown::entry(const XBot::FSM::Message& msg){

    shared_data().plugin_status->setStatus("PLACEDOWN");
      
    
    //CALL SERVICE TO MOVE
    
    //Hand selection
    std_msgs::String message;
    message = *shared_data()._hand_selection;    
    std::string selectedHand;
    selectedHand = message.data;
    
    std::cout << "SelectedHand: " << message.data << std::endl;

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    if(!selectedHand.compare("RSoftHand"))
      start_frame = *shared_data()._last_pose_right_hand;
    else if(!selectedHand.compare("LSoftHand"))
      start_frame = *shared_data()._last_pose_left_hand;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = selectedHand;
    start.frame = start_frame;
    
    // define the end frame
    geometry_msgs::PoseStamped end_frame;
    end_frame = start_frame;
    
    end_frame.pose.position.z-= 0.13;
    
    
    trajectory_utils::Cartesian end;
    end.distal_frame = selectedHand;
    end.frame = end_frame;

    if(!selectedHand.compare("RSoftHand"))
      shared_data()._last_pose_right_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
    else if(!selectedHand.compare("LSoftHand"))
      shared_data()._last_pose_left_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));    

    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = TRAJ_DURATION;         // traj duration 5 second      
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

    std::cout << "PlaceDown run. 'success'->Ungrasp\t'fail'->PlaceDown"<< std::endl;
    
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::PlaceDown::run(double time, double period){

  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // Reach failed
    if (!shared_data().current_command->str().compare("fail"))
      transit("PlaceDown");
    
    // Reach succeeded
    if (!shared_data().current_command->str().compare("success"))
      transit("Ungrasp");
  } 

}

///////////////////////////////////////////////////////////////////////////////

    // void myfsm::PlaceDown::entry(const XBot::FSM::Message& msg){
// 
//     shared_data().plugin_status->setStatus("PLACEDDOWN");
//     
//     std::cout << "PlaceDown_entry" << std::endl;
//         
//     shared_data()._robot->sense(); 
//     
//     Eigen::Affine3d world_T_bl;
//     std::string fb;  
//     
//     shared_data()._robot->model().getFloatingBaseLink(fb);
//     tf.getTransformTf(fb, "world_odom", world_T_bl);
//    
//     shared_data()._robot->model().setFloatingBasePose(world_T_bl);
//     shared_data()._robot->model().update();     
//     
//     // RIGHT HAND
//     
//     Eigen::Affine3d poseRightHand;
//     geometry_msgs::Pose start_frame_pose;
// 
//     shared_data()._robot->model().getPose("RSoftHand", poseRightHand);
//     tf::poseEigenToMsg (poseRightHand, start_frame_pose);
// 
//     geometry_msgs::PoseStamped poseHandStamped;
//     poseHandStamped.pose = start_frame_pose;
//     poseHandStamped.pose.position.z-=0.000001;
//     
//     //publish ros message
//     shared_data()._SoftHandPose_pub.publish (poseHandStamped);
// 
// }
// 
// 
// ///////////////////////////////////////////////////////////////////////////////
// void myfsm::PlaceDown::run(double time, double period){
//   
//     shared_data()._robot->sense(); 
//     
//     Eigen::Affine3d world_T_bl;
//     std::string fb;  
//     
//     shared_data()._robot->model().getFloatingBaseLink(fb);
//     tf.getTransformTf(fb, "world_odom", world_T_bl);
//    
//     shared_data()._robot->model().setFloatingBasePose(world_T_bl);
//     shared_data()._robot->model().update();     
//     
//     // RIGHT HAND
//     
//     Eigen::Affine3d poseRightHand;
//     geometry_msgs::Pose start_frame_pose;
// 
//     shared_data()._robot->model().getPose("RSoftHand", poseRightHand);
//     
//     //Reading initial wrench from ros topic
//     double f_x,f_y,f_z,w_Fz_ft;
//     
//     shared_data()._ft_r_arm = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>("/xbotcore/bigman/ft/r_arm_ft");
//     
//     f_x = shared_data()._ft_r_arm->wrench.force.x;
//     f_y = shared_data()._ft_r_arm->wrench.force.y;
//     f_z = shared_data()._ft_r_arm->wrench.force.z;
// 
//     Eigen::Vector3d ft_F_ft,w_F_ft;
//     ft_F_ft << f_x, f_y, f_z;
//     w_F_ft = poseRightHand * ft_F_ft;
//     w_Fz_ft = w_F_ft(2);
//       
// //     w_Fz_ft = shared_data()._RH_Rot_Z.dot(ft_F_ft);
//       
// //     double k;
// //     k = 0.7;
// 
//     std::cout << "w_Fz_ft: " << w_Fz_ft << std::endl;
// 
//     if(w_Fz_ft <= 50) //k * shared_data()._w_F_ft_initial)
//       transit("PlaceDown");
//     else
//       transit("Ungrasp");
//     
//       // blocking reading: wait for a command
//   if(shared_data().command.read(shared_data().current_command))
//   {
//     std::cout << "Command: " << shared_data().current_command->str() << std::endl;
// 
//     // Ungrasp failed
//     if (!shared_data().current_command->str().compare("placeddown_success"))
//       transit("Ungrasp");
//     
//     // Ungrasp Succeeded
//     if (!shared_data().current_command->str().compare("placeddown_failed"))
//       transit("Homing");
//   }
//   
// }

///////////////////////////////////////////////////////////////////////////////
void myfsm::PlaceDown::exit (){

}

/****************************** END PlaceDown ******************************/


/****************************** BEGIN Ungrasp *****************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::Ungrasp::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Ungrasp::entry(const XBot::FSM::Message& msg){

  shared_data().plugin_status->setStatus("UNGRASP");
      
  std::cout << "Ungrasp_entry" << std::endl;
  
  //CALL SERVICE TO UNGRASP  

  ADVR_ROS::advr_grasp_control_srv srv;
  srv.request.right_grasp = 0.0;
  srv.request.left_grasp = 0.0;
  // call the service
  shared_data()._grasp_client.call(srv);
  
  std::cout << "Ungrasp run. 'success'->Homing\t'fail'->Ungrasp" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Ungrasp::run(double time, double period){
  
  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // Ungrasp failed
    if (!shared_data().current_command->str().compare("fail"))
      transit("Ungrasp");
    
    // Ungrasp Succeeded
    if (!shared_data().current_command->str().compare("success"))
      transit("Homing_Ree");
  } 
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Ungrasp::exit (){

}

/****************************** END Ungrasp *******************************/


/******************************* BEGIN Adjust *******************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::Adjust::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Adjust::entry(const XBot::FSM::Message& msg){

    shared_data().plugin_status->setStatus("ADJUST");
    
    std::cout << "Adjust_entry" << std::endl;
  
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
    if(!selectedHand.compare("RSoftHand"))
      start_frame = *shared_data()._last_pose_right_hand;
    else if(!selectedHand.compare("LSoftHand"))
      start_frame = *shared_data()._last_pose_left_hand;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = selectedHand;
    start.frame = start_frame;
    
    // define the end frame
    geometry_msgs::PoseStamped end_frame;
    
    end_frame = start_frame;
    
    if(!selectedHand.compare("RSoftHand")){
      
      end_frame.pose.position.y+=0.01;
      
      shared_data()._last_pose_right_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
      
    }else if(!selectedHand.compare("LSoftHand")){

      end_frame.pose.position.y-= 0.01;
      
      shared_data()._last_pose_left_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
      
    }
       

    trajectory_utils::Cartesian end;
    end.distal_frame = selectedHand;
    end.frame = end_frame;
    
    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = 3;         // traj duration 5 second      
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
    
    std::cout << "Adjust run. 'success'->Grasp\t'fail'-> Adjust" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Adjust::run(double time, double period){

  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // Adjust failed
    if (!shared_data().current_command->str().compare("fail"))
      transit("Adjust");
    
    // Adjust Succeeded
    if (!shared_data().current_command->str().compare("success"))
      transit("Grasp");
    
  } 
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Adjust::exit (){

}

/********************************* END Adjust *******************************/
