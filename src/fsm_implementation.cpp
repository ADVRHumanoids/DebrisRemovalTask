#include "fsm_definition.h"


 /*BEGIN Homing*/

void myfsm::Homing::react(const XBot::FSM::Event& e) {

 

}

void myfsm::Homing::entry(const XBot::FSM::Message& msg){

  std::cout << "Homing_entry" << std::endl;
  sleep(1);

}


void myfsm::Homing::run(double time, double period){

  //DESIRED POSE: HOMING POSITION
  //   world_T_ee_homingPose:
  //   0.0310332   0.040294  -0.998706   0.467211
  //     0.011248    0.99911  0.0406599  -0.525973
  //     0.999455 -0.0124953  0.0305523 -0.0448229
  //          0          0          0          1


  
  //CALL SERVICE TO MOVE
  
  //WAIT FOR HOMING POSITION TO BE REACHED
  
  
  std::cout << "Homing" << std::endl;
  sleep(1);
  transit("Reached");

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



