#include "fsm_definition.h"

#include <vector>
#include <string>
#include <cmath>

#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Float64.h>

#define TRAJ_DURATION 10
#define WAITING_TIME 5
#define AUTONOMOUS 0
#define COMPLIANCE 0
#define K_COMPLIANT 250
#define K_STIFF 850


/******************************** BEGIN Homing *******************************/
///////////////////////////////////////////////////////////////////////////////

void myfsm::Homing::react(const XBot::FSM::Event& e) {
    
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing::entry(const XBot::FSM::Message& msg){

  shared_data().plugin_status->setStatus("HOMING");
  
  geometry_msgs::PoseStamped pose;
      
  pose.pose.position.x = 0.63;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.46;
  pose.pose.orientation.x = 0.037;
  pose.pose.orientation.y = 0.727;
  pose.pose.orientation.z = 0.023;
  pose.pose.orientation.w = 0.686;
  
  shared_data()._hand_pose = pose;
  
  std::cout << "\n\n" << 
               "\033[1m******Homing state******\033[0m\n" <<
              "\033[92m 'success' ---> Reach\033[0m\n" <<
              "\033[91m   'fail'  ---> Homing\033[0m\n" <<
               "\033[1m************************\033[0m\n" << std::endl;

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing::run(double time, double period){

  shared_data()._pose_pub.publish(shared_data()._hand_pose);  
  
  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;
    
    // Homing success
    if (!shared_data().current_command->str().compare("success"))
      transit("Reach");
    
    // Homing fail
    if (!shared_data().current_command->str().compare("fail"))
      transit("Homing");
   
  }

}


///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing::exit (){
  
}

/********************************* END Homing ********************************/


/******************************** BEGIN Reach *******************************/
///////////////////////////////////////////////////////////////////////////////

void myfsm::Reach::react(const XBot::FSM::Event& e) {
    
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Reach::entry(const XBot::FSM::Message& msg){

  shared_data().plugin_status->setStatus("REACH");
  
  std::cout << "\n\n" << 
               "\033[1m******Reach state******\033[0m\n" <<
               "\033[1mSelect the pose where the debris is. \033[0m" << std::endl;
  
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0.83; //pseudo random values, orientation not changed
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.26;
  pose.pose.orientation.x = 0.037;
  pose.pose.orientation.y = 0.727;
  pose.pose.orientation.z = 0.023;
  pose.pose.orientation.w = 0.686;
  shared_data()._debris_pose = pose;
  
//   ADVR_ROS::im_pose_msg::ConstPtr tmp;
//   tmp = ros::topic::waitForMessage<ADVR_ROS::im_pose_msg>("debris_pose");
//   shared_data()._debris_pose = tmp->pose_stamped;
  
  std::cout << "\033[1mPose selected.\033[0m\n" <<
              "\033[92m 'success' ---> Grasp\033[0m\n" <<
              "\033[91m   'fail'  ---> Homing\033[0m\n" <<
               "\033[1m************************\033[0m\n" << std::endl;

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Reach::run(double time, double period){

  shared_data()._pose_pub.publish(shared_data()._debris_pose);  
  
  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;
    
    // Reach success
    if (!shared_data().current_command->str().compare("success"))
      transit("Grasp");
    
    // Reach fail
    if (!shared_data().current_command->str().compare("fail"))
      transit("Homing");
   
  }

}


///////////////////////////////////////////////////////////////////////////////
void myfsm::Reach::exit (){
  
}

/********************************* END Reach ********************************/


/******************************** BEGIN Grasp *******************************/
///////////////////////////////////////////////////////////////////////////////

void myfsm::Grasp::react(const XBot::FSM::Event& e) {
    
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Grasp::entry(const XBot::FSM::Message& msg){

  shared_data().plugin_status->setStatus("GRASP");
  
  //here a manual command is sent to grasp
  
  std::cout << "\n\n" << 
               "\033[1m******Grasp state******\033[0m\n" <<
              "\033[92m 'success' ---> Pick\033[0m\n" <<
              "\033[91m   'fail'  ---> Grasp\033[0m\n" <<
               "\033[1m************************\033[0m\n" << std::endl;

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Grasp::run(double time, double period){

  //future implementation
//   shared_data()._grasp_pub.publish(true); 
  
  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;
    
    // Grasp success
    if (!shared_data().current_command->str().compare("success"))
      transit("Pick");
    
    // Grasp fail
    if (!shared_data().current_command->str().compare("fail"))
      transit("Grasp");
   
  }

}


///////////////////////////////////////////////////////////////////////////////
void myfsm::Grasp::exit (){
  
}

/********************************* END Grasp ********************************/


/******************************** BEGIN Pick *******************************/
///////////////////////////////////////////////////////////////////////////////

void myfsm::Pick::react(const XBot::FSM::Event& e) {
    
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Pick::entry(const XBot::FSM::Message& msg){

  shared_data().plugin_status->setStatus("PICK");
  
  geometry_msgs::PoseStamped pose;
      
  pose.pose.position.x = 0.63; //same as Homing for now
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.46;
  pose.pose.orientation.x = 0.037;
  pose.pose.orientation.y = 0.727;
  pose.pose.orientation.z = 0.023;
  pose.pose.orientation.w = 0.686;
  
  shared_data()._hand_pose = pose;
  
  std::cout << "\n\n" << 
               "\033[1m*******Pick state********\033[0m\n" <<
              "\033[92m 'success' ---> MoveAway\033[0m\n" <<
              "\033[91m   'fail'  ---> Homing\033[0m\n" <<
               "\033[1m*************************\033[0m\n" << std::endl;

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Pick::run(double time, double period){

  shared_data()._pose_pub.publish(shared_data()._hand_pose);  
  
  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;
    
    // Pick success
    if (!shared_data().current_command->str().compare("success"))
      transit("MoveAway");
    
    // Pick fail
    if (!shared_data().current_command->str().compare("fail"))
      transit("Homing");
   
  }

}


///////////////////////////////////////////////////////////////////////////////
void myfsm::Pick::exit (){
  
}

/********************************* END Pick ********************************/


/******************************** BEGIN MoveAway *******************************/
///////////////////////////////////////////////////////////////////////////////

void myfsm::MoveAway::react(const XBot::FSM::Event& e) {
    
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::MoveAway::entry(const XBot::FSM::Message& msg){

  shared_data().plugin_status->setStatus("MOVEAWAY");
  
  geometry_msgs::PoseStamped pose;
      
  pose.pose.position.x = 0.63; //pseudo random values, orientation not changed
  pose.pose.position.y = -0.40;
  pose.pose.position.z = 0.46;
  pose.pose.orientation.x = 0.037;
  pose.pose.orientation.y = 0.727;
  pose.pose.orientation.z = 0.023;
  pose.pose.orientation.w = 0.686;
  
  shared_data()._hand_pose = pose;
  
  std::cout << "\n\n" << 
               "\033[1m*******MoveAway state*****\033[0m\n" <<
              "\033[92m 'success' ---> PlaceDown\033[0m\n" <<
              "\033[91m   'fail'  ---> Homing\033[0m\n" <<
               "\033[1m**************************\033[0m\n" << std::endl;

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::MoveAway::run(double time, double period){

  shared_data()._pose_pub.publish(shared_data()._hand_pose);  
  
  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;
    
    // MoveAway success
    if (!shared_data().current_command->str().compare("success"))
      transit("PlaceDown");
    
    // MoveAway fail
    if (!shared_data().current_command->str().compare("fail"))
      transit("Homing");
   
  }

}


///////////////////////////////////////////////////////////////////////////////
void myfsm::MoveAway::exit (){
  
}

/********************************* END MoveAway ********************************/


/******************************** BEGIN PlaceDown *******************************/
///////////////////////////////////////////////////////////////////////////////

void myfsm::PlaceDown::react(const XBot::FSM::Event& e) {
    
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::PlaceDown::entry(const XBot::FSM::Message& msg){

  shared_data().plugin_status->setStatus("PLACEDOWN");
  
  geometry_msgs::PoseStamped pose;
  
  pose = shared_data()._hand_pose;
  pose.pose.position.z-= 0.01;
  
  shared_data()._hand_pose = pose;
  
  std::cout << "\n\n" << 
               "\033[1m*******PlaceDown state*****\033[0m\n" <<
              "\033[92m 'success' ---> Ungrasp\033[0m\n" <<
              "\033[91m   'fail'  ---> PlaceDown\033[0m\n" <<
               "\033[1m**************************\033[0m\n" << std::endl;

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::PlaceDown::run(double time, double period){

  shared_data()._pose_pub.publish(shared_data()._hand_pose);  
  
  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;
    
    // PlaceDown success
    if (!shared_data().current_command->str().compare("success"))
      transit("Ungrasp");
    
    // PlaceDown fail
    if (!shared_data().current_command->str().compare("fail"))
      transit("PlaceDown");
   
  }

}


///////////////////////////////////////////////////////////////////////////////
void myfsm::PlaceDown::exit (){
  
}

/********************************* END PlaceDown ********************************/


/******************************** BEGIN Ungrasp *******************************/
///////////////////////////////////////////////////////////////////////////////

void myfsm::Ungrasp::react(const XBot::FSM::Event& e) {
    
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Ungrasp::entry(const XBot::FSM::Message& msg){

  shared_data().plugin_status->setStatus("UNGRASP");
  
  //here a manual command is sent to ungrasp
  
  std::cout << "\n\n" << 
               "\033[1m******Ungrasp state******\033[0m\n" <<
              "\033[92m 'success' ---> Homing\033[0m\n" <<
              "\033[91m   'fail'  ---> Ungrasp\033[0m\n" <<
               "\033[1m************************\033[0m\n" << std::endl;

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Ungrasp::run(double time, double period){

  //future implementation
//   shared_data()._grasp_pub.publish(false); 
  
  // blocking reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;
    
    // Ungrasp success
    if (!shared_data().current_command->str().compare("success"))
      transit("Homing");
    
    // Ungrasp fail
    if (!shared_data().current_command->str().compare("fail"))
      transit("Ungrasp");
   
  }

}


///////////////////////////////////////////////////////////////////////////////
void myfsm::Ungrasp::exit (){
  
}

/********************************* END Ungrasp ********************************/



//example how to set stiffness
//     geometry_msgs::Vector3 stiffV;
//       
//     stiffV.x = 100;
//     stiffV.y = 100;
//     stiffV.z = 800;
//     shared_data()._stiffnessVector_pub.publish(stiffV);

//example how to read ext forces
//     shared_data()._end_effector = shared_data()._robot->model().chain("arm1").getTipLinkName();
    
//     Eigen::MatrixXd J,J_pinv;
//     shared_data()._robot->model().getJacobian(shared_data()._end_effector,J);
//     J_pinv = J.transpose() * (J*J.transpose()).inverse();
//       
//     //Reading external forces
//     Eigen::VectorXd extTor(7),extWrench(6);
//     shared_data()._robot->model().getJointEffort(extTor);
//     extWrench = J_pinv.transpose() * extTor;
//     std::cout << extWrench.transpose() << std::endl;
    


// PlaceDown old method for reference
// void myfsm::PlaceDown::run(double time, double period){
//   
//   //Wait for the trajectory to be completed
//   if(!shared_data()._feedback){
//     if(!shared_data().current_command->str().empty())
//     {
//       std::cout << "Command: " << shared_data().current_command->str() << std::endl;
// 
//       // Reach failed
//       if (!shared_data().current_command->str().compare("fail"))
//         transit("PlaceDown");
//       
//       // Reach succeeded
//       if (!shared_data().current_command->str().compare("success"))
//         transit("Ungrasp");
//     }
//     
//     // METHOD
//     if(AUTONOMOUS){
//       Eigen::Affine3d poseRightHand;
//       geometry_msgs::Pose tmp;
//       tmp = shared_data()._last_pose_right_hand->pose;
//       tf::poseMsgToEigen(tmp,poseRightHand);
//       double f_x,f_y,f_z,w_Fz_ft;
//       Eigen::Vector6d aux_ft;
//       shared_data()._robot->getForceTorque().at("r_arm_ft")->getWrench(aux_ft);
//       f_x = aux_ft(0);
//       f_y = aux_ft(1);
//       f_z = aux_ft(2);
//       
//       Eigen::Vector3d ft_F_ft,w_F_ft;
//       ft_F_ft << f_x, f_y, f_z;
//       w_F_ft = poseRightHand.linear() * ft_F_ft;
//       w_Fz_ft = w_F_ft(2);
// 
//       std::cout << "w_Fz_ft: " << w_Fz_ft << std::endl;
// 
//       if(w_Fz_ft <= 50)
//         transit("PlaceDown");
//       else
//         transit("Ungrasp");
//     }
//   }
// 
// }
// 


//AdjustLaterally old method for reference
// void myfsm::AdjustLaterally::run(double time, double period){
//   
//   //Wait for the trajectory to be completed
//   if(!shared_data()._feedback){
//     
//     bool contact = false;
//     if(AUTONOMOUS){
//       Eigen::Vector6d aux_ft;
//       shared_data()._robot->getForceTorque().at("r_arm_ft")->getWrench(aux_ft);
//       double force_y = aux_ft(1);
//       std::cout << "Force(y): " << force_y << std::endl;
//       if(force_y < -10)
//         contact = true;
//     }
//     
//     // blocking reading: wait for a command
//     if(!shared_data().current_command->str().empty() || contact)
//     {
//       std::cout << "Command: " << shared_data().current_command->str() << std::endl;
// 
//       // AdjustLaterally
//       if (!shared_data().current_command->str().compare("AdjustLaterally"))
//         transit("AdjustLaterally");
//       
//       // AdjustForward
//       if (!shared_data().current_command->str().compare("AdjustForward"))
//         transit("AdjustForward");
//       
//       // AdjustLaterally Succeeded
//       if (!shared_data().current_command->str().compare("success") || contact)
//         transit("Grasp");
//       
//       // AdjustLaterally failed
//       if (!shared_data().current_command->str().compare("fail"))
//         transit("Homing_Ree");
//       
//     }else if(AUTONOMOUS){
//       shared_data()._time+= period;
//       if(shared_data()._time > WAITING_TIME/3){
//         transit("AdjustLaterally");
//     //     shared_data().current_command = std::shared_ptr<XBot::Command>(new XBot::Command("AdjustLaterally"));
//       }     
//     }
//   }
// }