#include "fsm_definition.h"

#include <vector>



 /*BEGIN Homing*/

void myfsm::Homing::react(const XBot::FSM::Event& e) {

 

}

void myfsm::Homing::entry(const XBot::FSM::Message& msg){

  std::cout << "Homing_entry" << std::endl;
  sleep(1);

}


void myfsm::Homing::run(double time, double period){
  
  std::cout << "Homing" << std::endl;

  //DESIRED POSE: HOMING POSITION
  
  //   world_T_RIGHTee_homingPose:
  //   0.0310332   0.040294  -0.998706   0.467211
  //     0.011248    0.99911  0.0406599  -0.525973
  //     0.999455 -0.0124953  0.0305523 -0.0448229
  //          0          0          0          1
//   x: 0.467211
//   y: -0.525973
//   z: -0.0448229
//   
//   qx: 0.01851436567264015
//   qy: 0.6959748703195231
//   qz: 0.010116945573104904
//   qw: 0.7177561184411692

  
  //   world_T_LEFTee_homingPose:  
  //   -0.105681    0.282662    -0.95338    0.564943
  //   0.0390022     0.95919    0.280061    0.457868
  //    0.993635 -0.00758672   -0.112393 -0.00638944
  //           0           0           0           1
//   x: 0.564943
//   y: 0.457868
//   z: -0.00638944
//   
//   qx: 0.1089976629095435
//   qy: 0.7377777431777746
//   qz: 0.09232939109468996
//   qw: 0.6597566177704103  

    /******************************************************************/
    
    // send a trajectory for the end effector as a segment
    
    shared_data()._robot->sense();     
    

    // RIGHT HAND
    
    Eigen::Affine3d poseRightHand;
    geometry_msgs::Pose start_frame_pose;

    shared_data()._robot->model().getPose("RSoftHand", "Waist", poseRightHand);
    tf::poseEigenToMsg (poseRightHand, start_frame_pose);

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    
    start_frame.pose.position.x = start_frame_pose.position.x;
    start_frame.pose.position.y = start_frame_pose.position.y;
    start_frame.pose.position.z = start_frame_pose.position.z;
      
    start_frame.pose.orientation.x = start_frame_pose.orientation.x;
    start_frame.pose.orientation.y = start_frame_pose.orientation.y;
    start_frame.pose.orientation.z = start_frame_pose.orientation.z;
    start_frame.pose.orientation.w = start_frame_pose.orientation.w;    
    
    
    trajectory_utils::Cartesian start;
    start.distal_frame = "RSoftHand";
    start.frame = start_frame;
    
    // define the end frame - RIGHT HAND
    geometry_msgs::PoseStamped end_frame;
    end_frame.pose.position.x = 0.467211;
    end_frame.pose.position.y = -0.525973;
    end_frame.pose.position.z = -0.0448229;
    
    end_frame.pose.orientation.x = 0.01851436567264015;
    end_frame.pose.orientation.y = 0.6959748703195231;
    end_frame.pose.orientation.z = 0.010116945573104904;
    end_frame.pose.orientation.w = 0.7177561184411692;
    
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
    
    
    /******************************************************************/
    
    // LEFT HAND
    
    Eigen::Affine3d poseLeftHand;
    geometry_msgs::Pose start_frame_pose;

    shared_data()._robot->model().getPose("LSoftHand", "Waist", poseLeftHand);
    tf::poseEigenToMsg (poseLeftHand, start_frame_pose);

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    
    start_frame.pose.position.x = start_frame_pose.position.x;
    start_frame.pose.position.y = start_frame_pose.position.y;
    start_frame.pose.position.z = start_frame_pose.position.z;
      
    start_frame.pose.orientation.x = start_frame_pose.orientation.x;
    start_frame.pose.orientation.y = start_frame_pose.orientation.y;
    start_frame.pose.orientation.z = start_frame_pose.orientation.z;
    start_frame.pose.orientation.w = start_frame_pose.orientation.w;    
    
    
    trajectory_utils::Cartesian start;
    start.distal_frame = "LSoftHand";
    start.frame = start_frame;
    
    // define the end frame - RIGHT HAND
    geometry_msgs::PoseStamped end_frame;
    end_frame.pose.position.x = 0.564943;
    end_frame.pose.position.y = 0.457868;
    end_frame.pose.position.z = -0.00638944;
    
    end_frame.pose.orientation.x = 0.1089976629095435;
    end_frame.pose.orientation.y = 0.7377777431777746;
    end_frame.pose.orientation.z = 0.09232939109468996;
    end_frame.pose.orientation.w = 0.6597566177704103;
    
    trajectory_utils::Cartesian end;
    end.distal_frame = "LSoftHand";
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
    
    
    /******************************************************************/    
  
  //CALL SERVICE TO MOVE
  
  bool home;
//   home = move(pose,time);
  home = true;
  
  sleep(5);
  
  if(home){
    transit("Reached");
  }else{
    transit("Homing");
  }

}

void myfsm::Homing::exit (){

 

}

/*END Homing*/


 /*BEGIN Reached*/

void myfsm::Reached::react(const XBot::FSM::Event& e) {

 

}

void myfsm::Reached::entry(const XBot::FSM::Message& msg){

  std::cout << "Reached_entry" << std::endl;
  sleep(1);  

}


void myfsm::Reached::run(double time, double period){
   
  // blocking call: wait for a pose on topic handle_pose
  shared_data()._debris_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("debris_pose");
   
  std::cout << "pose x : " << shared_data()._debris_pose->pose.position.x << std::endl;

  //DESIRED POSE: DEBRIS POSITION
  //   world_T_ee_reachedPose:
  //   -0.55177 -0.603929 -0.575169  0.739059
  //   0.277916   0.51709 -0.809556 -0.136363
  //   0.786329 -0.606537 -0.117473  0.126653
  // 	  0         0         0         1

  
  //CALL SERVICE TO MOVE
  
  //WAIT FOR DEBRIS POSITION TO BE REACHED

  
  std::cout << "Reached" << std::endl;
  sleep(1);
  transit("Grasped");

}

void myfsm::Reached::exit (){

 

}

/*END Reached*/


 /*BEGIN Grasped*/

void myfsm::Grasped::react(const XBot::FSM::Event& e) {

 

}

void myfsm::Grasped::entry(const XBot::FSM::Message& msg){

  std::cout << "Grasped_entry" << std::endl;
  sleep(1); 
  

}


void myfsm::Grasped::run(double time, double period){

  //CALL SERVICE TO GRASP
  
  //WAIT FOR GRASPING TO END

    
  std::cout << "Grasped" << std::endl;
  sleep(1);
  transit("Picked");

}

void myfsm::Grasped::exit (){

 

}

/*END Grasped*/


 /*BEGIN Picked*/

void myfsm::Picked::react(const XBot::FSM::Event& e) {

 

}

void myfsm::Picked::entry(const XBot::FSM::Message& msg){

  std::cout << "Picked_entry" << std::endl;
  sleep(1);   

}


void myfsm::Picked::run(double time, double period){

  //DESIRED POSE: PICKED POSITION
  //   world_T_ee_pickedPose:
  //   -0.510232  -0.800641  -0.314065    0.58673
  //   0.0906155   0.313097  -0.945388  -0.256443
  //     0.85525  -0.510827 -0.0872019   0.051406
  // 	  0          0          0          1

  
  //CALL SERVICE TO MOVE
  
  //WAIT FOR PICKED POSITION TO BE REACHED

    
  std::cout << "Picked" << std::endl;
  sleep(1);
  transit("MovedAway");

}

void myfsm::Picked::exit (){

 

}

/*END Picked*/


 /*BEGIN MovedAway*/

void myfsm::MovedAway::react(const XBot::FSM::Event& e) {

 

}

void myfsm::MovedAway::entry(const XBot::FSM::Message& msg){

  std::cout << "MovedAway_entry" << std::endl;
  sleep(1);   

}


void myfsm::MovedAway::run(double time, double period){

  //DESIRED POSE: MOVEDAWAY POSITION
  //   world_T_ee_MovedAwayPose::
  //     0.0840183    0.943207   -0.321405    0.358676
  //     -0.10957    0.329334    0.937834   -0.752365
  //     0.990422  -0.0435789    0.131017 -0.00966471
  // 	    0           0           0           1

  
  //CALL SERVICE TO MOVE
  
  //WAIT FOR MOVEDAWAY POSITION TO BE REACHED

    
  std::cout << "MovedAway" << std::endl;
  sleep(1);
  transit("PlacedDown");  
 

}

void myfsm::MovedAway::exit (){

 

}

/*END MovedAway*/


 /*BEGIN PlacedDown*/

void myfsm::PlacedDown::react(const XBot::FSM::Event& e) {

 

}

void myfsm::PlacedDown::entry(const XBot::FSM::Message& msg){

  std::cout << "PlacedDown_entry" << std::endl;
  sleep(1);   

}


void myfsm::PlacedDown::run(double time, double period){

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


     
  std::cout << "PlacedDown" << std::endl;
  sleep(1);
  transit("Ungrasped");  

}

void myfsm::PlacedDown::exit (){

 

}

/*END PlacedDown*/


 /*BEGIN Ungrasped*/

void myfsm::Ungrasped::react(const XBot::FSM::Event& e) {

 

}

void myfsm::Ungrasped::entry(const XBot::FSM::Message& msg){

  //CALL SERVICE TO UNGRASP
  
  //WAIT FOR UNGRASPING TO END

      
  std::cout << "Ungrasped_entry" << std::endl;
  sleep(1);   

}


void myfsm::Ungrasped::run(double time, double period){

  std::cout << "Ungrasped" << std::endl;
  sleep(1);
  transit("Homing");   

}

void myfsm::Ungrasped::exit (){

 

}

/*END Ungrasped*/



