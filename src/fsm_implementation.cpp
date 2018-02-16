#include "fsm_definition.h"

#include <vector>
#include <string>

#include <eigen_conversions/eigen_msg.h>

#define TRAJ_DURATION 10
#define WAITING_TIME 5
#define AUTONOMOUS 0


/******************************** BEGIN Homing_init *******************************/
///////////////////////////////////////////////////////////////////////////////

void myfsm::Homing_init::react(const XBot::FSM::Event& e) {
    
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing_init::entry(const XBot::FSM::Message& msg){

    shared_data().plugin_status->setStatus("HOMING");
    std::cout << "Homing_init_entry" << std::endl;
    shared_data()._robot->sense(); 

    // SAVE INITIAL END EFFECTOR POSES
    Eigen::Affine3d poseLeftHand,poseRightHand;
    geometry_msgs::Pose left_hand_pose,right_hand_pose;
    
//     shared_data()._robot->model().getPose("arm2_8", "torso_2", poseRightHand);
//     
//     tf::poseEigenToMsg (poseRightHand, right_hand_pose);

//     std::cout << "Pose right arm2_8:\n" << right_hand_pose.orientation << "\n\n" << right_hand_pose.position << std::endl;
    
    shared_data()._robot->model().getPose("arm1_8", "torso_2", poseLeftHand);
    shared_data()._robot->model().getPose("arm2_8", "torso_2", poseRightHand);
    
    tf::poseEigenToMsg (poseLeftHand, left_hand_pose);
    tf::poseEigenToMsg (poseRightHand, right_hand_pose);

    std::cout << "Pose right arm2_8:\n" << right_hand_pose.orientation << "\n\n" << right_hand_pose.position << std::endl;
    
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
    start.distal_frame = "arm2_8";
    start.frame = start_frame;
    
    // define the end frame
    geometry_msgs::PoseStamped end_frame;
//     end_frame = *shared_data()._initial_pose_right_hand;
    
    end_frame.pose.position.x = 0.546;
    end_frame.pose.position.y = -0.51;
    end_frame.pose.position.z = -0.08;
    
    end_frame.pose.orientation.x = 0.306;
    end_frame.pose.orientation.y = 0.343;
    end_frame.pose.orientation.z = 0.490;
    end_frame.pose.orientation.w = 0.741;  
    
    trajectory_utils::Cartesian end;
    end.distal_frame = "arm2_8";
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
    srv.request.segment_trj.header.frame_id = "torso_2";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);
    std::cout << "\n\n" << 
                  "\033[1m*****Homing_Ree state******\033[0m\n" <<
                 "\033[92m 'success' ---> Homing_Lee \033[0m\n" <<
                 "\033[91m   'fail'  ---> Homing_Ree \033[0m\n" <<
                  "\033[1m***************************\033[0m\n" << std::endl;

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing_Ree::run(double time, double period){
  
  //Wait for the trajectory to be completed
//   if(!shared_data()._feedback)
//     transit("Homing_Lee");
//   std::cout << "feedback: " << shared_data()._feedback << std::endl; //test on real robot
  
  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // Homing_Ree failed
    if (!shared_data().current_command->str().compare("fail"))
      transit("Homing_Ree");
    
    // Homing_Ree succeeded
    if (!shared_data().current_command->str().compare("success"))
      transit("Homing_Lee");
   
  }

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
    start.distal_frame = "arm1_8";
    start.frame = start_frame;
    
    // define the potential intermediate frame
    trajectory_utils::Cartesian intermediate;
    if(shared_data()._hand_over_phase){
      geometry_msgs::PoseStamped intermediate_frame;
      intermediate_frame = *shared_data()._last_pose_left_hand;
      intermediate_frame.pose.position.x+= 0.08;
      intermediate_frame.pose.position.y+= 0.08;
      
      intermediate.distal_frame = "arm1_8";
      intermediate.frame = intermediate_frame;
    }
    
    // define the end frame
    geometry_msgs::PoseStamped end_frame;
//     end_frame = *shared_data()._initial_pose_left_hand;
    
    end_frame.pose.position.x = 0.546;
    end_frame.pose.position.y = 0.51;
    end_frame.pose.position.z = -0.08;
    
    end_frame.pose.orientation.x = 0.343;
    end_frame.pose.orientation.y = 0.306;
    end_frame.pose.orientation.z = 0.741;
    end_frame.pose.orientation.w = 0.490;    
    
    trajectory_utils::Cartesian end;
    end.distal_frame = "arm1_8";
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
    srv.request.segment_trj.header.frame_id = "torso_2";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);    
    
    std::cout << "\n\n" << 
                  "\033[1m***********Homing_Lee state************\033[0m\n" <<
                 "\033[92m    'success'       ---> HandSelection \033[0m\n" <<
                 "\033[91m      'fail'        ---> Homing_Ree \033[0m\n" <<
                 "\033[93m 'Handover_success' ---> MoveAway \033[0m\n" <<
                  "\033[1m***************************************\033[0m\n" << std::endl;
    
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing_Lee::run(double time, double period){
  //Wait for the trajectory to be completed
  if(!shared_data()._feedback){
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
    }else if(AUTONOMOUS){
      shared_data()._time+= period;
      if(shared_data()._time > WAITING_TIME && shared_data()._first){
        transit("HandSelection");
    //     shared_data().current_command = std::shared_ptr<XBot::Command>(new XBot::Command("HandSelection"));
        shared_data()._first = false;
      }     
    }
  }
}


///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing_Lee::exit (){
  shared_data()._time = 0;
}

/********************************* END Homing_Lee ********************************/



/******************************* BEGIN HandSelection *******************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::HandSelection::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::HandSelection::entry(const XBot::FSM::Message& msg){

    shared_data().plugin_status->setStatus("HANDSELECTION");
      
    std::cout << "\n\n" << 
                 "\033[1m**********HandSelection state***********\033[0m\n" <<
                 "\033[1mSelect the End Effector you want to use.\033[0m\n" <<
                 "\033[1m   -> Right Hand\033[0m\n" <<
                 "\033[1m   ->  Left Hand\033[0m\n" <<
                 "\033[1m****************************************\033[0m\n" << std::endl;
    


}

///////////////////////////////////////////////////////////////////////////////
void myfsm::HandSelection::run(double time, double period){

  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // arm1_8 selected
    if (!shared_data().current_command->str().compare("arm1_8")){
      std_msgs::String message;
      message.data = shared_data().current_command->str();
      shared_data()._hand_selection =  boost::shared_ptr<std_msgs::String>(new std_msgs::String(message));;
      transit("Reach");
    }
    // arm2_8 selected
    if (!shared_data().current_command->str().compare("arm2_8")){
      std_msgs::String message;
      message.data = shared_data().current_command->str();
      shared_data()._hand_selection =  boost::shared_ptr<std_msgs::String>(new std_msgs::String(message));;
      transit("Reach");
    }
  }else if(AUTONOMOUS){
      shared_data()._time+= period;
      if(shared_data()._time > WAITING_TIME){
        std_msgs::String message;
        message.data = "arm2_8";
        shared_data()._hand_selection =  boost::shared_ptr<std_msgs::String>(new std_msgs::String(message));;
        transit("Reach");
    //     shared_data().current_command = std::shared_ptr<XBot::Command>(new XBot::Command("arm2_8"));
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
    
    if(selectedHand.compare("arm2_8") || selectedHand.compare("arm1_8")){
        std::cout << "\n\n" << 
              "\033[1m**************Reach state***************\033[0m\n" <<
              "\033[1mSelect the pose where the debris is. \033[0m" << std::endl;
    }else
      std::cout << "Incorrect input, you need to publish a different message" << std::endl;
      
    // blocking call: wait for a pose on topic debris_pose
    ADVR_ROS::im_pose_msg::ConstPtr tmp;
    tmp = ros::topic::waitForMessage<ADVR_ROS::im_pose_msg>("debris_pose");

    shared_data()._debris_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(tmp->pose_stamped));

    //CALL SERVICE TO MOVE


    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    if(!selectedHand.compare("arm2_8"))
      start_frame = *shared_data()._last_pose_right_hand;
    else if(!selectedHand.compare("arm1_8"))
      start_frame = *shared_data()._last_pose_left_hand;

    trajectory_utils::Cartesian start;
    start.distal_frame = selectedHand;
    start.frame = start_frame;    
    
    
    // define the intermediate frame 
    geometry_msgs::PoseStamped intermediate_frame;
    
    intermediate_frame = *shared_data()._debris_pose;

    if(!selectedHand.compare("arm2_8")){
      intermediate_frame.pose.position.x-= 0.2;
//       intermediate_frame.pose.position.y-= 0.2;
    }else if(!selectedHand.compare("arm1_8")){
      intermediate_frame.pose.position.x-= 0.2;
//       intermediate_frame.pose.position.y+= 0.2;
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

    if(!selectedHand.compare("arm2_8"))
      shared_data()._last_pose_right_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
    else if(!selectedHand.compare("arm1_8"))
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
    srv.request.segment_trj.header.frame_id = "torso_2";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
//     std::cout << "forces: " << ros::topic::waitForMessage<geometry_msgs::WrenchStamped>("/xbotcore/walkman/ft/r_arm_ft/wrench") << std::endl;

    // call the service
    shared_data()._client.call(srv);

    std::cout <<  "\033[1mPose selected.\033[0m\n" <<
                 "\033[92m     'success'     ---> Grasp \033[0m\n" <<
                 "\033[91m       'fail'      ---> Homing_Ree \033[0m\n" <<
                 "\033[93m 'AdjustLaterally' ---> AdjustLaterally \033[0m\n" <<
                 "\033[93m  'AdjustForward'  ---> AdjustForward \033[0m\n" <<
                  "\033[1m****************************************\033[0m\n" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Reach::run(double time, double period){
  //Wait for the trajectory to be completed
  if(!shared_data()._feedback){
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
      
      // AdjustLaterally
      if (!shared_data().current_command->str().compare("AdjustLaterally"))
        transit("AdjustLaterally");
      
      // AdjustForward
      if (!shared_data().current_command->str().compare("AdjustForward"))
        transit("AdjustForward");    
    }else if(AUTONOMOUS){
      shared_data()._time+= period;
      if(shared_data()._time > WAITING_TIME/3){
        transit("AdjustLaterally");
    //     shared_data().current_command = std::shared_ptr<XBot::Command>(new XBot::Command("AdjustLaterally"));
      }     
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Reach::exit (){
  shared_data()._time = 0;
}

/******************************** END Reach ********************************/


/******************************* BEGIN Grasp *******************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::Grasp::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Grasp::entry(const XBot::FSM::Message& msg){

    shared_data().plugin_status->setStatus("GRASP");
    
    //HAND
    //Hand selection
    std_msgs::String message;
    message = *shared_data()._hand_selection;    
    std::string selectedHand;
    selectedHand = message.data;
    
    std::cout << "SelectedHand: " << message.data << std::endl;
    
    ADVR_ROS::advr_grasp_control_srv srv;
  
    if(!selectedHand.compare("arm2_8")){
      if(!shared_data()._hand_over_phase){
          srv.request.right_grasp = 1.0;
          srv.request.left_grasp = 0.0;
      }else{
          srv.request.right_grasp = 1.0;
          srv.request.left_grasp = 1.0;
      }
    }else if(!selectedHand.compare("arm1_8")){
          srv.request.right_grasp = 0.0;
          srv.request.left_grasp = 1.0;
    }
    
    // call the service
    shared_data()._grasp_client.call(srv);    
      
    std::cout << "\n\n" << 
                "\033[1m************Grasp state************\033[0m\n" <<
                "\033[92m      'success'   ---> Pick \033[0m\n" <<
                "\033[91m       'fail'     ---> Grasp \033[0m\n" <<
                "\033[93m 'After_handover' ---> Homing_Lee \033[0m\n" <<
                "\033[1m***********************************\033[0m\n" << std::endl;
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
    }else if(AUTONOMOUS){
      shared_data()._time+= period;
      if(shared_data()._time > WAITING_TIME){
        transit("Pick");
    //     shared_data().current_command = std::shared_ptr<XBot::Command>(new XBot::Command("success"));
      }     
    }
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Grasp::exit (){
  shared_data()._time = 0;
}

/********************************* END Grasp ******************************/


/******************************* BEGIN Pick *******************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::Pick::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Pick::entry(const XBot::FSM::Message& msg){

    shared_data().plugin_status->setStatus("PICK");
  
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
    if(!selectedHand.compare("arm2_8"))
      start_frame = *shared_data()._last_pose_right_hand;
    else if(!selectedHand.compare("arm1_8"))
      start_frame = *shared_data()._last_pose_left_hand;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = selectedHand;
    start.frame = start_frame;
    
    // define the end frame
    geometry_msgs::PoseStamped end_frame;
    
    if(!selectedHand.compare("arm2_8")){
      
      end_frame.pose.position.x = 0.54;
      end_frame.pose.position.y = -0.14;
      end_frame.pose.position.z = -0.16;   
      
      end_frame.pose.orientation.x = 0.149;
      end_frame.pose.orientation.y = 0.761;
      end_frame.pose.orientation.z = 0.577;
      end_frame.pose.orientation.w = 0.259; 
      
      shared_data()._last_pose_right_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
      
    }else if(!selectedHand.compare("arm1_8")){
      
      //NOT OPTIMIZED FOR HANDOVER
      end_frame.pose.position.x = 0.54;
      end_frame.pose.position.y = 0.14;
      end_frame.pose.position.z = -0.16;   
      
      end_frame.pose.orientation.x = 0.761;
      end_frame.pose.orientation.y = 0.149;
      end_frame.pose.orientation.z = 0.259;
      end_frame.pose.orientation.w = 0.577; 
      
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
    srv.request.segment_trj.header.frame_id = "torso_2";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);
    
    std::cout << "\n\n" << 
              "\033[1m***********Pick state************\033[0m\n" <<
             "\033[92m 'success'  ---> MoveAway \033[0m\n" <<
             "\033[91m  'fail'    ---> Homing_Ree \033[0m\n" <<
             "\033[93m 'Handover' ---> PickSecondHand \033[0m\n" <<
              "\033[1m*********************************\033[0m\n" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Pick::run(double time, double period){
  //Wait for the trajectory to be completed
  if(!shared_data()._feedback){
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
    }else if(AUTONOMOUS){
      shared_data()._time+= period;
      if(shared_data()._time > WAITING_TIME){
        transit("MoveAway");
    //     shared_data().current_command = std::shared_ptr<XBot::Command>(new XBot::Command("success"));
      }     
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Pick::exit (){
  shared_data()._time = 0;
}

/********************************* END Pick *******************************/


/*************************** BEGIN PickSecondHand ***************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::PickSecondHand::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::PickSecondHand::entry(const XBot::FSM::Message& msg){

    shared_data().plugin_status->setStatus("PICKSECONDHAND");
  
    //CALL SERVICE TO MOVE

    //HAND
    //Hand selection
    std_msgs::String message;
    message = *shared_data()._hand_selection;    
    std::string holdingHand;
    holdingHand = message.data;
    
    std::string secondHand;

    if(!holdingHand.compare("arm2_8"))
      secondHand = "arm1_8";
    else if(!holdingHand.compare("arm1_8"))
      secondHand = "arm2_8";
    
//     std::cout << "holdingHand: " << holdingHand << std::endl;
//     std::cout << "secondHand: " << secondHand << std::endl;

    
    Eigen::Affine3d poseSecondHand;
    geometry_msgs::Pose start_frame_pose;

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    if(!holdingHand.compare("arm2_8"))
      start_frame = *shared_data()._last_pose_left_hand;
    else if(!holdingHand.compare("arm1_8"))
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
    poseHoldingHand_KDL.p.y(0.10);
    poseHoldingHand_KDL.p.z(-0.10);

    tf::transformKDLToEigen(poseHoldingHand_KDL,poseHoldingHand_Affine);
    
    poseSecondHandFinal = poseHoldingHand * poseHoldingHand_Affine;

    tf::poseEigenToMsg (poseSecondHandFinal, start_frame_pose_holding_hand);

    double qx, qy,qz,qw;
    poseHoldingHand_KDL.M.GetQuaternion(qx,qy,qz,qw);
    

    if(!secondHand.compare("arm2_8")){
      
      intermediate_frame.pose.position.x = start_frame_pose_holding_hand.position.x;
      intermediate_frame.pose.position.y = start_frame_pose_holding_hand.position.y;
      intermediate_frame.pose.position.z = start_frame_pose_holding_hand.position.z;
      
      intermediate_frame.pose.orientation.x = qx;
      intermediate_frame.pose.orientation.y = qy;
      intermediate_frame.pose.orientation.z = qz;
      intermediate_frame.pose.orientation.w = qw;        
      
    }else if(!secondHand.compare("arm1_8")){
      
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
//     poseHoldingHand_KDL_2.p.y(0.0); ///real robot
//     poseHoldingHand_KDL_2.p.z(-0.10); ///real robot
    poseHoldingHand_KDL_2.p.y(-0.03);
    poseHoldingHand_KDL_2.p.z(-0.17);
    
    tf::transformKDLToEigen(poseHoldingHand_KDL_2,poseSecondHand_Affine);
    
    poseSecondHandFinal_2 = poseHoldingHand_2 * poseSecondHand_Affine;

    tf::poseEigenToMsg (poseSecondHandFinal_2, start_frame_pose_second_hand);

    poseHoldingHand_KDL_2.M.GetQuaternion(qx,qy,qz,qw);
    

    if(!secondHand.compare("arm2_8")){
      
      end_frame.pose.position.x = start_frame_pose_second_hand.position.x;
      end_frame.pose.position.y = start_frame_pose_second_hand.position.y;
      end_frame.pose.position.z = start_frame_pose_second_hand.position.z;

      end_frame.pose.orientation.x = qx;
      end_frame.pose.orientation.y = qy;
      end_frame.pose.orientation.z = qz;
      end_frame.pose.orientation.w = qw;        
      
    }else if(!secondHand.compare("arm1_8")){
      
      //TO BE IMPLEMENTED, IF NEEDED
      
    }

    trajectory_utils::Cartesian end;
    end.distal_frame = secondHand;
    end.frame = end_frame;

   
    if(!secondHand.compare("arm2_8"))
      shared_data()._last_pose_right_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
    else if(!secondHand.compare("arm1_8"))
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
    srv.request.segment_trj.header.frame_id = "torso_2";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);
    
    std::cout << "\n\n" << 
              "\033[1m***PickSecondHand state****\033[0m\n" <<
             "\033[92m 'success' ---> Grasp \033[0m\n" <<
             "\033[91m   'fail'  ---> Homing_Ree \033[0m\n" <<
              "\033[1m***************************\033[0m\n" << std::endl;
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
      rightHand.data = "arm2_8";   
      leftHand.data = "arm1_8";       
      
      if(!selectedHand.compare("arm2_8"))
        shared_data()._hand_selection = boost::shared_ptr<std_msgs::String>(new std_msgs::String(leftHand));
      else if(!selectedHand.compare("arm1_8"))
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
    
    //CALL SERVICE TO MOVE
    
    //Hand selection
    std_msgs::String message;
    message = *shared_data()._hand_selection;    
    std::string selectedHand;
    selectedHand = message.data;
    
    std::cout << "SelectedHand: " << message.data << std::endl;

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    if(!selectedHand.compare("arm2_8"))
      start_frame = *shared_data()._last_pose_right_hand;
    else if(!selectedHand.compare("arm1_8"))
      start_frame = *shared_data()._last_pose_left_hand;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = selectedHand;
    start.frame = start_frame;
    
    
    // define the intermediate frame 
    geometry_msgs::PoseStamped intermediate_frame;

    intermediate_frame.pose.position.x = 0.56;
    
    if(!selectedHand.compare("arm2_8"))
      intermediate_frame.pose.position.y = -0.36;
    else if(!selectedHand.compare("arm1_8"))
      intermediate_frame.pose.position.y = 0.36;
    
    intermediate_frame.pose.position.z = -0.05;
    if(shared_data()._hand_over_phase)
      intermediate_frame.pose.position.z+= 0.15;
    
    intermediate_frame.pose.orientation.x = 0.504;
    intermediate_frame.pose.orientation.y = 0.499;
    intermediate_frame.pose.orientation.z = 0.497;
    intermediate_frame.pose.orientation.w = 0.499;
    
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
    
    if(!selectedHand.compare("arm2_8")){
      
      end_frame.pose.position.x = 0.33;
      end_frame.pose.position.y = -0.76;
      end_frame.pose.position.z = -0.22;
      
      if(shared_data()._hand_over_phase){
        end_frame.pose.position.z+= 0.22;
        shared_data()._hand_over_phase = false;
      }
    
      end_frame.pose.orientation.x = 0.802;
      end_frame.pose.orientation.y = 0.161;
      end_frame.pose.orientation.z = 0.031;
      end_frame.pose.orientation.w = 0.575;
      
    }
    else if(!selectedHand.compare("arm1_8")){
      
      end_frame.pose.position.x = 0.33;
      end_frame.pose.position.y = 0.76;
      end_frame.pose.position.z = -0.22;
      
      end_frame.pose.orientation.x = 0.161;
      end_frame.pose.orientation.y = 0.802;
      end_frame.pose.orientation.z = 0.575;
      end_frame.pose.orientation.w = 0.031;
      
    }
 
    
    if(!selectedHand.compare("arm2_8"))
      shared_data()._last_pose_right_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
    else if(!selectedHand.compare("arm1_8"))
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
    srv.request.segment_trj.header.frame_id = "torso_2";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);     

    std::cout << "\n\n" << 
              "\033[1m******MoveAway state*******\033[0m\n" <<
             "\033[92m 'success' ---> PlaceDown \033[0m\n" <<
             "\033[91m   'fail'  ---> Homing_Ree \033[0m\n" <<
              "\033[1m***************************\033[0m\n" << std::endl;

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::MoveAway::run(double time, double period){
  
  //Wait for the trajectory to be completed
  if(!shared_data()._feedback && AUTONOMOUS){
    transit("PlaceDown");
  }
  
  
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
    if(!selectedHand.compare("arm2_8"))
      start_frame = *shared_data()._last_pose_right_hand;
    else if(!selectedHand.compare("arm1_8"))
      start_frame = *shared_data()._last_pose_left_hand;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = selectedHand;
    start.frame = start_frame;
    
    // define the end frame
    geometry_msgs::PoseStamped end_frame;
    end_frame = start_frame;
    
    if(AUTONOMOUS)
      end_frame.pose.position.z-= 0.01; //0.08 - if you change this, also change traj duration to TRAJ_DURATION
    else
      end_frame.pose.position.z-= 0.02; // was 0.08
    
    trajectory_utils::Cartesian end;
    end.distal_frame = selectedHand;
    end.frame = end_frame;

    if(!selectedHand.compare("arm2_8"))
      shared_data()._last_pose_right_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
    else if(!selectedHand.compare("arm1_8"))
      shared_data()._last_pose_left_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));    

    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    if(AUTONOMOUS)
      s1.T.data = 2;
    else
      s1.T.data = TRAJ_DURATION;
    
    s1.start = start;        // start pose
    s1.end = end;            // end pose 
    
    // only one segment in this example
    std::vector<trajectory_utils::segment> segments;
    segments.push_back(s1);
    
    // prepare the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = "torso_2";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);

    std::cout << "\n\n" << 
              "\033[1m*****PlaceDown state******\033[0m\n" <<
             "\033[92m 'success' ---> Ungrasp \033[0m\n" <<
             "\033[91m   'fail'  ---> PlaceDown \033[0m\n" <<
              "\033[1m**************************\033[0m\n" << std::endl;
    
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::PlaceDown::run(double time, double period){
  
  //Wait for the trajectory to be completed
  if(!shared_data()._feedback){
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
    
    // METHOD
    if(AUTONOMOUS){
      Eigen::Affine3d poseRightHand;
      geometry_msgs::Pose tmp;
      tmp = shared_data()._last_pose_right_hand->pose;
      tf::poseMsgToEigen(tmp,poseRightHand);
      double f_x,f_y,f_z,w_Fz_ft;
      Eigen::Vector6d aux_ft;
      shared_data()._robot->getForceTorque().at("r_arm_ft")->getWrench(aux_ft);
      f_x = aux_ft(0);
      f_y = aux_ft(1);
      f_z = aux_ft(2);
      
      Eigen::Vector3d ft_F_ft,w_F_ft;
      ft_F_ft << f_x, f_y, f_z;
      w_F_ft = poseRightHand.linear() * ft_F_ft;
      w_Fz_ft = w_F_ft(2);

      std::cout << "w_Fz_ft: " << w_Fz_ft << std::endl;

      if(w_Fz_ft <= 50)
        transit("PlaceDown");
      else
        transit("Ungrasp");
    }
  }

}

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
  
  //CALL SERVICE TO UNGRASP  

  ADVR_ROS::advr_grasp_control_srv srv;
  srv.request.right_grasp = 0.0;
  srv.request.left_grasp = 0.0;
  // call the service
  shared_data()._grasp_client.call(srv);
  
  std::cout << "\n\n" << 
              "\033[1m*******Ungrasp state*******\033[0m\n" <<
             "\033[92m 'success' ---> Homing_Ree \033[0m\n" <<
             "\033[91m   'fail'  ---> Ungrasp \033[0m\n" <<
              "\033[1m***************************\033[0m\n" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Ungrasp::run(double time, double period){
  
  shared_data()._time+= period;
  if(shared_data()._time > WAITING_TIME && AUTONOMOUS){
    transit("Homing_Ree");
//     shared_data().current_command = std::shared_ptr<XBot::Command>(new XBot::Command("success"));
  }
  
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
  shared_data()._time = 0;
}

/****************************** END Ungrasp *******************************/


/******************************* BEGIN AdjustLaterally *******************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::AdjustLaterally::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::AdjustLaterally::entry(const XBot::FSM::Message& msg){

    shared_data().plugin_status->setStatus("ADJUSTLATERALLY");
    
    //CALL SERVICE TO MOVE

    //HAND
    //Hand selection
    std_msgs::String message;
    message = *shared_data()._hand_selection;    
    std::string selectedHand;
    selectedHand = message.data;
    
//     std::cout << "SelectedHand: " << message.data << std::endl;

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    if(!selectedHand.compare("arm2_8"))
      start_frame = *shared_data()._last_pose_right_hand;
    else if(!selectedHand.compare("arm1_8"))
      start_frame = *shared_data()._last_pose_left_hand;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = selectedHand;
    start.frame = start_frame;
    
    // define the end frame
    geometry_msgs::PoseStamped end_frame;
    
    end_frame = start_frame;
    
    if(!selectedHand.compare("arm2_8")){
      
      end_frame.pose.position.y+=0.01;
      
      shared_data()._last_pose_right_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
      
    }else if(!selectedHand.compare("arm1_8")){

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
    srv.request.segment_trj.header.frame_id = "torso_2";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);
    
    std::cout << "\n\n" << 
              "\033[1m*********AdjustLaterally state**********\033[0m\n" <<
             "\033[92m      'success'    ---> Grasp \033[0m\n" <<
             "\033[91m        'fail'     ---> Homing_Ree \033[0m\n" <<
             "\033[93m 'AdjustLaterally' ---> AdjustLaterally \033[0m\n" <<
             "\033[93m  'AdjustForward'  ---> AdjustForward \033[0m\n" <<
              "\033[1m****************************************\033[0m\n" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::AdjustLaterally::run(double time, double period){
  //Wait for the trajectory to be completed
  if(!shared_data()._feedback){
    
    bool contact = false;
    if(AUTONOMOUS){
      Eigen::Vector6d aux_ft;
      shared_data()._robot->getForceTorque().at("r_arm_ft")->getWrench(aux_ft);
      double force_y = aux_ft(1);
      std::cout << "Force(y): " << force_y << std::endl;
      if(force_y < -10)
        contact = true;
    }
    
    // blocking reading: wait for a command
    if(!shared_data().current_command->str().empty() || contact)
    {
      std::cout << "Command: " << shared_data().current_command->str() << std::endl;

      // AdjustLaterally
      if (!shared_data().current_command->str().compare("AdjustLaterally"))
        transit("AdjustLaterally");
      
      // AdjustForward
      if (!shared_data().current_command->str().compare("AdjustForward"))
        transit("AdjustForward");
      
      // AdjustLaterally Succeeded
      if (!shared_data().current_command->str().compare("success") || contact)
        transit("Grasp");
      
      // AdjustLaterally failed
      if (!shared_data().current_command->str().compare("fail"))
        transit("Homing_Ree");
      
    }else if(AUTONOMOUS){
      shared_data()._time+= period;
      if(shared_data()._time > WAITING_TIME/3){
        transit("AdjustLaterally");
    //     shared_data().current_command = std::shared_ptr<XBot::Command>(new XBot::Command("AdjustLaterally"));
      }     
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::AdjustLaterally::exit (){
  shared_data()._time = 0;
}

/********************************* END AdjustLaterally *******************************/


/******************************* BEGIN AdjustForward *******************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::AdjustForward::react(const XBot::FSM::Event& e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::AdjustForward::entry(const XBot::FSM::Message& msg){

    shared_data().plugin_status->setStatus("ADJUSTFORWARD");
    
    //CALL SERVICE TO MOVE

    //HAND
    //Hand selection
    std_msgs::String message;
    message = *shared_data()._hand_selection;    
    std::string selectedHand;
    selectedHand = message.data;
    
//     std::cout << "SelectedHand: " << message.data << std::endl;

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    if(!selectedHand.compare("arm2_8"))
      start_frame = *shared_data()._last_pose_right_hand;
    else if(!selectedHand.compare("arm1_8"))
      start_frame = *shared_data()._last_pose_left_hand;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = selectedHand;
    start.frame = start_frame;
    
    // define the end frame
    geometry_msgs::PoseStamped end_frame;
    
    end_frame = start_frame;
    end_frame.pose.position.x+=0.01;
    
    if(!selectedHand.compare("arm2_8"))
      shared_data()._last_pose_right_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
    else if(!selectedHand.compare("arm1_8"))
      shared_data()._last_pose_left_hand = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_frame));
       

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
    srv.request.segment_trj.header.frame_id = "torso_2";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);
    
    std::cout << "\n\n" << 
              "\033[1m**********AdjustForward state***********\033[0m\n" <<
             "\033[92m      'success'    ---> Grasp \033[0m\n" <<
             "\033[91m        'fail'     ---> Homing_Ree \033[0m\n" <<
             "\033[93m 'AdjustLaterally' ---> AdjustLaterally \033[0m\n" <<
             "\033[93m  'AdjustForward'  ---> AdjustForward \033[0m\n" <<
              "\033[1m****************************************\033[0m\n" << std::endl;              
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::AdjustForward::run(double time, double period){

  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // AdjustLaterally
    if (!shared_data().current_command->str().compare("AdjustLaterally"))
      transit("AdjustLaterally");
    
    // AdjustForward
    if (!shared_data().current_command->str().compare("AdjustForward"))
      transit("AdjustForward");
    
    // AdjustForward Succeeded
    if (!shared_data().current_command->str().compare("success"))
      transit("Grasp");
    
    // AdjustForward failed
    if (!shared_data().current_command->str().compare("fail"))
      transit("Homing_Ree");    
    
  } 
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::AdjustForward::exit (){

}

/********************************* END AdjustForward *******************************/
