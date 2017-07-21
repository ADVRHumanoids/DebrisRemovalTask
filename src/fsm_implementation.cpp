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
    Eigen::Affine3d poseRightHand;
    geometry_msgs::Pose start_frame_pose;

    shared_data()._robot->model().getPose("RSoftHand", poseRightHand);
    tf::poseEigenToMsg (poseRightHand, start_frame_pose);
    

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    start_frame.pose = start_frame_pose;
    
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
    
    shared_data()._last_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));

    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = 5.0;         // traj duration 5 second      
    s1.start = start;        // start pose
    s1.end = end;            // end pose 
    
    // only one segment in this example
    std::vector<trajectory_utils::segment> segments;
    segments.push_back(s1);
    
    // prapere the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = "world_odom";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);    
    
    
    // at the moment not possible to move both hands in the same state
    
//     /******************************************************************/
//     
//     // LEFT HAND
//     
//     Eigen::Affine3d poseLeftHand;
//     geometry_msgs::Pose start_frame_pose2;
// 
//     shared_data()._robot->model().getPose("LSoftHand",poseLeftHand);
//     tf::poseEigenToMsg (poseLeftHand, start_frame_pose2);
// 
//     // define the start frame 
//     geometry_msgs::PoseStamped start_frame2;
//     start_frame2.pose = start_frame_pose2;
// 
//     trajectory_utils::Cartesian start2;
//     start2.distal_frame = "LSoftHand";
//     start2.frame = start_frame2;
//     
//     // define the end frame - LEFT HAND
//     geometry_msgs::PoseStamped end_frame2;
//     end_frame2.pose.position.x = 0.732;
//     end_frame2.pose.position.y = 0.250;
//     end_frame2.pose.position.z = 0.029;
//     
//     end_frame2.pose.orientation.x = -0.669;
//     end_frame2.pose.orientation.y = -0.464;
//     end_frame2.pose.orientation.z = 0.120;
//     end_frame2.pose.orientation.w = 0.568;
//     
//     trajectory_utils::Cartesian end2;
//     end2.distal_frame = "LSoftHand";
//     end2.frame = end_frame2;
// 
// 
//     // define the first segment
//     trajectory_utils::segment s2;
//     s2.type.data = 0;        // min jerk traj
//     s2.T.data = 5.0;         // traj duration 5 second      
//     s2.start = start2;        // start pose
//     s2.end = end2;            // end pose 
//     
//     // only one segment in this example
//     std::vector<trajectory_utils::segment> segments2;
//     segments2.push_back(s2);
//     
//     // prapere the advr_segment_control
//     ADVR_ROS::advr_segment_control srv2;
//     srv2.request.segment_trj.header.frame_id = "Waist";
//     srv2.request.segment_trj.header.stamp = ros::Time::now();
//     srv2.request.segment_trj.segments = segments2;
//     
//     // call the service
//     shared_data()._client.call(srv2);
//     
//     
//     /******************************************************************/
    
    
      std::cout << "Homing run" << std::endl;


}

///////////////////////////////////////////////////////////////////////////////

void myfsm::Homing::run(double time, double period){
  
//   std::cout << "Homing run" << std::endl;
  
  //TBD: Check if the RH has reached the homing_pose
  
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
    
    //CHANGED TRIAL REAL ROBOT
    // blocking call: wait for a msg on topic hand_selection
//     shared_data()._hand_selection = ros::topic::waitForMessage<std_msgs::String>("hand_selection");
    std_msgs::String rh;
    rh.data = "RSoftHand";
    shared_data()._hand_selection = boost::shared_ptr<std_msgs::String>(new std_msgs::String(rh));
    std_msgs::String message;
    message = *shared_data()._hand_selection;    
    std::string selectedHand;
    selectedHand = message.data;
    
    if(selectedHand.compare("RSoftHand") || selectedHand.compare("LSoftHand"))
      std::cout << "Select the pose where the debris is." << std::endl;
    else
      std::cout << "Incorrect input, you need to publish a different message" << std::endl;
      
    //in the future the position to reach the debris will be given by a ros message published on the rostopic "debris_pose"
    
    //COMMENTED FOR FIRST TRIALS ON THE REAL ROBOT
    // blocking call: wait for a pose on topic debris_pose
//     shared_data()._debris_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("debris_pose");
    geometry_msgs::PoseStamped poseDebris1;
    poseDebris1.pose.position.x = 0.619;
    poseDebris1.pose.position.y = -0.29;
    poseDebris1.pose.position.z = 0.873;
    
    poseDebris1.pose.orientation.x = 0;
    poseDebris1.pose.orientation.y = -0.5591931143131625;
    poseDebris1.pose.orientation.z = 0;
    poseDebris1.pose.orientation.w = 0.8290374303399975;
    
    shared_data()._debris_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(poseDebris1));
    
    
    //CALL SERVICE TO MOVE
    // send a trajectory for the end effector as a segment
    
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
    
    //HAND
    
//     Eigen::Affine3d poseHand;
//     geometry_msgs::Pose start_frame_pose;
    
//     shared_data()._robot->model().getPose(selectedHand, poseHand);
//     tf::poseEigenToMsg (poseHand, start_frame_pose);


    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
//     start_frame.pose = start_frame_pose;
    start_frame = *shared_data()._last_pose;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = selectedHand;
    start.frame = start_frame;    
    
    // define the end frame
    geometry_msgs::PoseStamped end_frame;
    
//     end_frame.pose = start_frame_pose;    
    
    end_frame = *shared_data()._debris_pose;
    
    
    trajectory_utils::Cartesian end;
    end.distal_frame = selectedHand;
    end.frame = end_frame;

    shared_data()._last_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
    
    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = 5.0;         // traj duration 5 second      
    s1.start = start;        // start pose
    s1.end = end;            // end pose 
    
    // only one segment in this example
    std::vector<trajectory_utils::segment> segments;
    segments.push_back(s1);
    
    // prapere the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = "world_odom";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);

    std::cout << "Reached run. 'reached_fail'-> Homing	'reached_sucess'->Grasped	'gotoreach'->Reached" << std::endl;

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Reached::run(double time, double period){

//   std::cout << "Reached run. 'reached_fail'-> Homing	'reached_sucess'->Grasped	'gotoreach'->Reached" << std::endl;
  
  //TBD: Check if the RH has reached the reached_pose
  
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
    
    // to be canceled
    if (!shared_data().current_command.str().compare("gotoreach"))
      transit("Reached");      
  } 

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Reached::exit (){

 

}

/********************************* END Reached *******************************/


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
    
    std::string handJoint;
    
    if(!selectedHand.compare("RSoftHand"))
      handJoint = "r_handj";
    else if(!selectedHand.compare("LSoftHand")){
      handJoint = "l_handj"; 
    }
    
    
    int hand_id = shared_data()._robot->getHand()[handJoint]->getHandId();
    XBot::Hand::Ptr hand = shared_data()._robot->getHand(hand_id);
    //MAYBE ON THE REAL ROBOT HERE PUT AN IF AND A WAITFORMESSAGE TO BE SURE YOU WANT TO GRASP, YOU DON'T TRY EVERY TIME AS IN SIMULATION
    hand->grasp(1);
   
    
    //COMMENTED FOR FIRST TRIALS ON THE REAL ROBOT
    
    
//     //IF is not grasping perform a movement to the left/right
//     //READ GRASP STATE
//     double grasped;
//     grasped = hand->getGraspState();
//       
//     if(!grasped){
//       
//       //move 1 cm to the left/right
//       std::cout << "Not grasped, moving 1 cm to the left/right" << std::endl;
//       
//       //CALL SERVICE TO MOVE
//       // send a trajectory for the end effector as a segment
//       
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
//       
//       Eigen::Affine3d poseHand;
//       geometry_msgs::Pose start_frame_pose;
// 
//       shared_data()._robot->model().getPose(selectedHand, poseHand);
//       tf::poseEigenToMsg (poseHand, start_frame_pose);
// 
// 
//       // define the start frame 
//       geometry_msgs::PoseStamped start_frame;
//       start_frame.pose = start_frame_pose;
//       
//       trajectory_utils::Cartesian start;
//       start.distal_frame = selectedHand;
//       start.frame = start_frame;
//       
//       // define the end frame - RIGHT HAND
//       geometry_msgs::PoseStamped end_frame;
//       
//       end_frame.pose = start_frame_pose; 
//       
//       if(!selectedHand.compare("RSoftHand")){
// 	
// 	end_frame.pose.position.y+= -0.01;
// 
//       }else if(!selectedHand.compare("LSoftHand")){
// 	
// 	end_frame.pose.position.y-= 0.01;   
// 
//       }
// 
//       trajectory_utils::Cartesian end;
//       end.distal_frame = selectedHand;
//       end.frame = end_frame;
// 
//       // define the first segment
//       trajectory_utils::segment s1;
//       s1.type.data = 0;        // min jerk traj
//       s1.T.data = 3.0;         // traj duration 3 second      
//       s1.start = start;        // start pose
//       s1.end = end;            // end pose 
//       
//       // only one segment in this example
//       std::vector<trajectory_utils::segment> segments;
//       segments.push_back(s1);
//       
//       // prapere the advr_segment_control
//       ADVR_ROS::advr_segment_control srv;
//       srv.request.segment_trj.header.frame_id = "world_odom";
//       srv.request.segment_trj.header.stamp = ros::Time::now();
//       srv.request.segment_trj.segments = segments;
//       
//       // call the service
//       shared_data()._client.call(srv);
//       
//       
//       
//     }
  
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Grasped::run(double time, double period){

  //Wait for the trajectory to be completed
  if(!shared_data()._feedback){
    
    //COMMENTED FOR FIRST TRIALS ON THE REAL ROBOT
//     //HAND
//     //Hand selection
//     std_msgs::String message;
//     message = *shared_data()._hand_selection;    
//     std::string selectedHand;
//     selectedHand = message.data;
//     
//     std::cout << "SelectedHand: " << message.data << std::endl;
//     
//     //try to grasp
//     
//     std::string handJoint;
//     
//     if(!selectedHand.compare("RSoftHand"))
//       handJoint = "r_handj";
//     else if(!selectedHand.compare("LSoftHand")){
//       handJoint = "l_handj"; 
//     }
//     
//     
//     int hand_id = shared_data()._robot->getHand()[handJoint]->getHandId();
//     XBot::Hand::Ptr hand = shared_data()._robot->getHand(hand_id);
//     hand->grasp(1);
//     
//     //IF is not grasping perform a movement to the left/right
//     //READ GRASP STATE
//     double grasped;
//     grasped = hand->getGraspState();
//     
//     if(!grasped){
//       std::cout << "Grasped fail, trying again" << std::endl;
//       transit("Grasped");
//     }
    
  
    std::cout << "Grasped run. 'grasped_fail'-> Grasped	'grasped_success'->Picked	'move_away_after_ho'->MovedAway" << std::endl;
    
    // blocking reading: wait for a command
    if(shared_data().command.read(shared_data().current_command))
    {
      std::cout << "Command: " << shared_data().current_command.str() << std::endl;

      //UNCOMMENTED FOR FIRST TRIALS ON THE REAL ROBOT
      // Grasped failed
      if (!shared_data().current_command.str().compare("grasped_fail"))
	transit("Grasped");
      
      // Grasped Succeeded
      if (!shared_data().current_command.str().compare("grasped_success"))
	transit("Picked");
      
      // new: moveaway after handover
      if (!shared_data().current_command.str().compare("move_away_after_ho")){
	
	//Ungrasp left hand
	int hand_id = shared_data()._robot->getHand()["l_handj"]->getHandId();
	XBot::Hand::Ptr hand = shared_data()._robot->getHand(hand_id);
	hand->grasp(0);

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
    // send a trajectory for the end effector as a segment
/*    
    shared_data()._robot->sense(); 
    
    Eigen::Affine3d world_T_bl;
    std::string fb;  
    
    shared_data()._robot->model().getFloatingBaseLink(fb);
    tf.getTransformTf(fb, "world_odom", world_T_bl);
   
    shared_data()._robot->model().setFloatingBasePose(world_T_bl);
    shared_data()._robot->model().update();  */
    
    //HAND
    //Hand selection
    std_msgs::String message;
    message = *shared_data()._hand_selection;    
    std::string selectedHand;
    selectedHand = message.data;
    
    std::cout << "SelectedHand: " << message.data << std::endl;

    
//     Eigen::Affine3d poseHand;
//     geometry_msgs::Pose start_frame_pose;
// 
//     shared_data()._robot->model().getPose(selectedHand, poseHand);
//     tf::poseEigenToMsg (poseHand, start_frame_pose);


    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
//     start_frame.pose = start_frame_pose;
    start_frame = *shared_data()._last_pose;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = selectedHand;
    start.frame = start_frame;
    
    // define the end frame - RIGHT HAND
    geometry_msgs::PoseStamped end_frame;
    
//     end_frame.pose = start_frame_pose; 
    
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
//       end_frame.pose.position.y = 0.2; //tmp_ft_prova
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
    s1.T.data = 5.0;         // traj duration 5 second      
    s1.start = start;        // start pose
    s1.end = end;            // end pose 
    
    // only one segment in this example
    std::vector<trajectory_utils::segment> segments;
    segments.push_back(s1);
    
    // prapere the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = "world_odom";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);
    
    std::cout << "Picked run. 'picked_fail'-> Homing	'picked_success'->MovedAway	'pick_second_hand'->PickSecondHand" << std::endl;
    
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Picked::run(double time, double period){

//   std::cout << "Picked run. 'picked_fail'-> Homing	'picked_success'->MovedAway	'pick_second_hand'->PickSecondHand" << std::endl;
  
  //TBD: Check if the RH has reached the picked_pose
  
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
    
    // Picked Succeeded
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
    // send a trajectory for the end effector as a segment
    
    shared_data()._robot->sense(); 
    
    Eigen::Affine3d world_T_bl;
    std::string fb;  
    
    shared_data()._robot->model().getFloatingBaseLink(fb);
    tf.getTransformTf(fb, "world_odom", world_T_bl);
   
    shared_data()._robot->model().setFloatingBasePose(world_T_bl);
    shared_data()._robot->model().update();   
    
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

    
    shared_data()._robot->model().getPose(secondHand, poseSecondHand);
    tf::poseEigenToMsg (poseSecondHand, start_frame_pose);
    
    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    start_frame.pose = start_frame_pose;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = secondHand;
    start.frame = start_frame;
    
    // define the intermediate frame
    geometry_msgs::PoseStamped intermediate_frame;
    
    Eigen::Affine3d poseHoldingHand,poseSecondHandFinal,poseHoldingHand_Affine;
    geometry_msgs::Pose start_frame_pose_holding_hand;

    KDL::Frame poseHoldingHand_KDL;
    shared_data()._robot->model().getPose(holdingHand, poseHoldingHand_KDL);
   
    shared_data()._robot->model().getPose(holdingHand, poseHoldingHand);
    

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
    s1.T.data = 5.0;         // traj duration 5 second      
    s1.start = start;        // start pose
    s1.end = intermediate;            // end pose 
    
    std::vector<trajectory_utils::segment> segments;
    segments.push_back(s1);

    shared_data()._robot->sense();        
    
    // define the end frame
    geometry_msgs::PoseStamped end_frame;
    

    geometry_msgs::Pose start_frame_pose_second_hand;
    Eigen::Affine3d poseHoldingHand_2,poseSecondHandFinal_2, poseSecondHand_Affine;    
    
    KDL::Frame poseHoldingHand_KDL_2;

    
    shared_data()._robot->model().getPose(holdingHand, poseHoldingHand_KDL_2);

    shared_data()._robot->model().getPose(holdingHand, poseHoldingHand_2);    
    
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

    // define the second segment
    trajectory_utils::segment s2;
    s2.type.data = 0;        // min jerk traj
    s2.T.data = 5.0;         // traj duration 5 second      
    s2.start = intermediate;        // start pose
    s2.end = end;            // end pose 
    
    segments.push_back(s2);
    
    // prapere the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = "world_odom";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);     
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::PickSecondHand::run(double time, double period){

  std::cout << "PickSecondHand run. 'homing'-> Homing	'pick_second_hand_success'->Grasped" << std::endl;
  
  
  //TBD: Check if the RH has reached the PickSecondHand_pose
  
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
      
      transit("Grasped");
      
    }
  } 


}

///////////////////////////////////////////////////////////////////////////////
void myfsm::PickSecondHand::exit (){

 

}

/********************************* END PickSecondHand *******************************/



/****************************** BEGIN MovedAway *****************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::MovedAway::react(const XBot::FSM::Event& e) {

 

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::MovedAway::entry(const XBot::FSM::Message& msg){

    std::cout << "MovedAway_entry" << std::endl;
  
    //CALL SERVICE TO MOVE
    // send a trajectory for the end effector as a segment
    
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
    
    // RIGHT HAND
    
//     Eigen::Affine3d poseRightHand;
//     geometry_msgs::Pose start_frame_pose;
// 
//     shared_data()._robot->model().getPose("RSoftHand", poseRightHand);
// //     shared_data()._robot->model().getPose("LSoftHand", poseRightHand); // tmp_ft_prova
//     tf::poseEigenToMsg (poseRightHand, start_frame_pose);

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
//     start_frame.pose = start_frame_pose;
    
    start_frame = *shared_data()._last_pose;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = "RSoftHand";
//     start.distal_frame = "LSoftHand"; //tmp_ft_prova
    start.frame = start_frame;
    
    
    // define the intermediate frame 
    geometry_msgs::PoseStamped intermediate_frame;

    intermediate_frame.pose.position.x = 0.50;
    intermediate_frame.pose.position.y = -0.393;
//     intermediate_frame.pose.position.y = 0.393; //tmp_ft_prova
    intermediate_frame.pose.position.z = 1.09;     
    
    intermediate_frame.pose.orientation.x = 0.0;
    intermediate_frame.pose.orientation.y = -0.7071070192004544;
    intermediate_frame.pose.orientation.z = 0.0;
    intermediate_frame.pose.orientation.w = 0.7071070192004544;     
    
    trajectory_utils::Cartesian intermediate;
    intermediate.distal_frame = "RSoftHand";
//     intermediate.distal_frame = "LSoftHand"; //tmp_ft_prova
    intermediate.frame = intermediate_frame;    
    
    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = 5.0;         // traj duration 5 second      
    s1.start = start;        // start pose
    s1.end = intermediate;            // end pose 
    
    std::vector<trajectory_utils::segment> segments;
    segments.push_back(s1);    
    
    // define the end frame - RIGHT HAND
    geometry_msgs::PoseStamped end_frame;
    
    end_frame.pose.position.x = 0.451;
    end_frame.pose.position.y = -0.940;
//     end_frame.pose.position.y = 0.940; //tmp_ft_prova
    end_frame.pose.position.z = 1.05;
    
    end_frame.pose.orientation.x = -0.386;
//     end_frame.pose.orientation.x = 0.386;
    end_frame.pose.orientation.y = -0.429;
    end_frame.pose.orientation.z = -0.452;
//     end_frame.pose.orientation.z = 0.452;
    end_frame.pose.orientation.w = 0.678;    
 
    shared_data()._last_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));

    trajectory_utils::Cartesian end;
    end.distal_frame = "RSoftHand";
//     end.distal_frame = "LSoftHand"; //tmp_ft_prova
    end.frame = end_frame;

    // define the second segment
    trajectory_utils::segment s2;
    s2.type.data = 0;        // min jerk traj
    s2.T.data = 5.0;         // traj duration 5 second      
    s2.start = intermediate;        // start pose
    s2.end = end;            // end pose 

    segments.push_back(s2);
    
    // prapere the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = "world_odom";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);     

    std::cout << "MovedAway run. 'movedaway_fail'-> Homing	'movedaway_success'->PlacedDown" << std::endl;

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::MovedAway::run(double time, double period){

//   std::cout << "MovedAway run. 'movedaway_fail'-> Homing	'movedaway_success'->PlacedDown" << std::endl;
  
  
  //TBD: Check if the RH has reached the movedaway_pose
  
  // blocking reading: wait for a command
  if(shared_data().command.read(shared_data().current_command))
  {
    std::cout << "Command: " << shared_data().current_command.str() << std::endl;

    // MovedAway failed
    if (!shared_data().current_command.str().compare("movedaway_fail"))
      transit("Homing");
    
    // MovedAway Succeeded
    if (!shared_data().current_command.str().compare("movedaway_success")){
      
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
  //     shared_data()._ft_r_arm = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>("/xbotcore/bigman/ft/l_arm_ft"); //tmp_ft_prova
      
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
    poseHandStamped.pose.position.z-=0.01;
    
    //publish ros message
    shared_data()._SoftHandPose_pub.publish (poseHandStamped);
//     usleep(10);

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
    //shared_data()._ft_r_arm = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>("/xbotcore/bigman/ft/l_arm_ft"); //tmp_ft_prova
    
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
//   std_msgs::String message;
//   message.data = "";
//   shared_data()._grasp_mag_pub_RSoftHand.publish (message);
  
  //     //l hand moving
    int r_hand_id = shared_data()._robot->getHand()["r_handj"]->getHandId();
    XBot::Hand::Ptr r_hand = shared_data()._robot->getHand(r_hand_id);
    r_hand->grasp(0);
    

  
  std::cout << "Ungrasped run. 'ungrasped_fail'-> Ungrasped	'ungrasped_success'->Homing" << std::endl;


}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Ungrasped::run(double time, double period){

//   std::cout << "Ungrasped run. 'ungrasped_fail'-> Ungrasped	'ungrasped_success'->Homing" << std::endl;
  
  
  //TBD: Check if the RH has reached the ungrasped_pose
  
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
