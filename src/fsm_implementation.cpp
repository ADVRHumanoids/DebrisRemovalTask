#include "fsm_definition.h"

#include <vector>




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
    
    // RIGHT HAND
    
    Eigen::Affine3d poseRightHand;
    geometry_msgs::Pose start_frame_pose;

    shared_data()._robot->model().getPose("RSoftHand", "Waist", poseRightHand);
    tf::poseEigenToMsg (poseRightHand, start_frame_pose);

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    start_frame.pose = start_frame_pose;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = "RSoftHand";
    start.frame = start_frame;
    
    // define the end frame - RIGHT HAND
    geometry_msgs::PoseStamped end_frame;
    
    end_frame.pose = start_frame_pose;    

    end_frame.pose.position.x = 0.679;
    end_frame.pose.position.y = -0.504;
    end_frame.pose.position.z = -0.049;    
    
    end_frame.pose.orientation.x = -0.017;
    end_frame.pose.orientation.y = -0.698;
    end_frame.pose.orientation.z = -0.012;
    end_frame.pose.orientation.w = 0.715;    
    trajectory_utils::Cartesian end;
    end.distal_frame = "RSoftHand";
    end.frame = end_frame;

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
    srv.request.segment_trj.header.frame_id = "Waist";
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

}

///////////////////////////////////////////////////////////////////////////////

void myfsm::Homing::run(double time, double period){
  
  std::cout << "Homing run" << std::endl;
  
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
  
    //in the future the position to reach the debris will be given by a ros message published on the rostopic "debris_pose"
    
  // blocking call: wait for a pose on topic debris_pose
//   shared_data()._debris_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("debris_pose");
  
  // Debug msg
//   std::cout << "pose x : " << shared_data()._debris_pose->pose.position.x << std::endl;

  
    //CALL SERVICE TO MOVE
    // send a trajectory for the end effector as a segment
    
    shared_data()._robot->sense();    
    
    // RIGHT HAND
    
    Eigen::Affine3d poseRightHand;
    geometry_msgs::Pose start_frame_pose;

    shared_data()._robot->model().getPose("RSoftHand", "Waist", poseRightHand);
    tf::poseEigenToMsg (poseRightHand, start_frame_pose);

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    start_frame.pose = start_frame_pose;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = "RSoftHand";
    start.frame = start_frame;
    
    // define the end frame - RIGHT HAND
    geometry_msgs::PoseStamped end_frame;
    
    end_frame.pose = start_frame_pose;    

    end_frame.pose.position.x = 0.841;
    end_frame.pose.position.y = 0.051;
    end_frame.pose.position.z = 0.134;    
    
    end_frame.pose.orientation.x = -0.110;
    end_frame.pose.orientation.y = 0.739;
    end_frame.pose.orientation.z = -0.481;
    end_frame.pose.orientation.w = -0.459;    
    trajectory_utils::Cartesian end;
    end.distal_frame = "RSoftHand";
    end.frame = end_frame;

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
    srv.request.segment_trj.header.frame_id = "Waist";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);     

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Reached::run(double time, double period){
   
  std::cout << "Reached run" << std::endl;
  
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
  
  //CALL SERVICE TO GRASP
  

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Grasped::run(double time, double period){

   
  std::cout << "Grasped run" << std::endl;
  
  //TBD: Check if the RH has reached the grasped_pose
  
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
    
    shared_data()._robot->sense();    
    
    // RIGHT HAND
    
    Eigen::Affine3d poseRightHand;
    geometry_msgs::Pose start_frame_pose;

    shared_data()._robot->model().getPose("RSoftHand", "Waist", poseRightHand);
    tf::poseEigenToMsg (poseRightHand, start_frame_pose);

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    start_frame.pose = start_frame_pose;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = "RSoftHand";
    start.frame = start_frame;
    
    // define the end frame - RIGHT HAND
    geometry_msgs::PoseStamped end_frame;
    
    end_frame.pose = start_frame_pose;    

    end_frame.pose.position.x = 0.627;
    end_frame.pose.position.y = -0.049;
    end_frame.pose.position.z = 0.054;    
    
    end_frame.pose.orientation.x = -0.258;
    end_frame.pose.orientation.y = 0.691;
    end_frame.pose.orientation.z = -0.527;
    end_frame.pose.orientation.w = -0.423;    
    trajectory_utils::Cartesian end;
    end.distal_frame = "RSoftHand";
    end.frame = end_frame;

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
    srv.request.segment_trj.header.frame_id = "Waist";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);     
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Picked::run(double time, double period){

   
  std::cout << "Picked run" << std::endl;
  
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
  } 


}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Picked::exit (){

 

}

/********************************* END Picked *******************************/


/****************************** BEGIN MovedAway *****************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::MovedAway::react(const XBot::FSM::Event& e) {

 

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::MovedAway::entry(const XBot::FSM::Message& msg){

    std::cout << "MovedAway_entry" << std::endl;
  
    //CALL SERVICE TO MOVE
    // send a trajectory for the end effector as a segment
    
    shared_data()._robot->sense();    
    
    // RIGHT HAND
    
    Eigen::Affine3d poseRightHand;
    geometry_msgs::Pose start_frame_pose;

    shared_data()._robot->model().getPose("RSoftHand", "Waist", poseRightHand);
    tf::poseEigenToMsg (poseRightHand, start_frame_pose);

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    start_frame.pose = start_frame_pose;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = "RSoftHand";
    start.frame = start_frame;
    
    // define the end frame - RIGHT HAND
    geometry_msgs::PoseStamped end_frame;
    
    end_frame.pose = start_frame_pose;    

    end_frame.pose.position.x = 0.451;
    end_frame.pose.position.y = -0.940;
    end_frame.pose.position.z = -0.040;    
    
    end_frame.pose.orientation.x = -0.396;
    end_frame.pose.orientation.y = -0.525;
    end_frame.pose.orientation.z = -0.428;
    end_frame.pose.orientation.w = 0.620;    
    trajectory_utils::Cartesian end;
    end.distal_frame = "RSoftHand";
    end.frame = end_frame;

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
    srv.request.segment_trj.header.frame_id = "Waist";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);     

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::MovedAway::run(double time, double period){

   
  std::cout << "MovedAway run" << std::endl;
  
  //TBD: Check if the RH has reached the movedaway_pose
  
  // blocking reading: wait for a command
  if(shared_data().command.read(shared_data().current_command))
  {
    std::cout << "Command: " << shared_data().current_command.str() << std::endl;

    // MovedAway failed
    if (!shared_data().current_command.str().compare("movedaway_fail"))
      transit("Homing");
    
    // MovedAway Succeeded
    if (!shared_data().current_command.str().compare("movedaway_success"))
      transit("PlacedDown");
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

  //READ RIGHT ARM FT ("/xbotcore/bigman/ft/r_arm_ft")
  //ASSIGN THE NORM  OF IT TO FT_i (AND FT_f)
  
  //until the debris is not touching the ground, lower it a bit
  while( /*FT_i <= k1 * FT_f && FT_i >= k2 * FT_f*/ 0){
    
    //READ CURRENT POSE
    
    //DESIRED POSE: CURRENTPOSE->z = CURRENTPOSE->z - 0.05;
    
    //CALL SERVICE TO MOVE
  
    //WAIT FOR DESIRED POSE TO BE REACHED
    
    //READ RIGHT ARM FT ("/xbotcore/bigman/ft/r_arm_ft")
    //ASSIGN THE NORM  OF IT TO AND FT_f
    
  }

  // I now end the loop here  
  std::cout << "THE END" << std::endl;
  return;  

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::PlacedDown::run(double time, double period){

  std::cout << "PlacedDown run" << std::endl;
  
  //TBD: Check if the RH has reached the placeddown_pose
  
  // blocking reading: wait for a command
  if(shared_data().command.read(shared_data().current_command))
  {
    std::cout << "Command: " << shared_data().current_command.str() << std::endl;

    // PlacedDown failed
    if (!shared_data().current_command.str().compare("placeddown_fail"))
      transit("Homing");
    
    // PlacedDown Succeeded
    if (!shared_data().current_command.str().compare("placeddown_success"))
      transit("Ungrasped");
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

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Ungrasped::run(double time, double period){

  std::cout << "Ungrasped run" << std::endl;
  
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
