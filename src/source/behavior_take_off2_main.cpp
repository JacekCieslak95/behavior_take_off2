#include "../include/behavior_take_off2.h"

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());

  std::cout << ros::this_node::getName() << std::endl;

  BehaviorTakeOff2 behavior;
  behavior.setUp();
  ros::Rate rate(10);
  while(ros::ok()){
    behavior.run();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}