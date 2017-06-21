#include "fsm_definition.h"


 /*BEGIN Homing*/

void myfsm::Homing::react(const XBot::FSM::Event& e) {

 

}

void myfsm::Homing::entry(const XBot::FSM::Message& msg){

  std::cout << "Homing_entry" << std::endl;
  sleep(1);

}


void myfsm::Homing::run(double time, double period){

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



