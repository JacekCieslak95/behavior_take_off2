#ifndef TAKE_OFF2_H
#define TAKE_OFF2_H

//System
#include <string>
#include <tuple>
// ROS
#include <ros/ros.h>
#include <droneMsgsROS/droneSpeeds.h>
#include "std_srvs/Empty.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <yaml-cpp/yaml.h>
#include <droneMsgsROS/dronePositionRefCommandStamped.h>
#include <droneMsgsROS/dronePositionTrajectoryRefCommand.h>
#include <droneMsgsROS/droneYawRefCommand.h>
#include <droneMsgsROS/droneTrajectoryControllerControlMode.h>
#include <droneMsgsROS/setControlMode.h>

// Aerostack msgs
#include <droneMsgsROS/BehaviorEvent.h>
#include <droneMsgsROS/dronePose.h>
#include <droneMsgsROS/droneCommand.h>
#include <droneMsgsROS/setInitDroneYaw_srv_type.h>
#include<droneMsgsROS/ConsultBelief.h>
#include <droneMsgsROS/dronePitchRollCmd.h>
#include <droneMsgsROS/droneDAltitudeCmd.h>
#include <droneMsgsROS/droneDYawCmd.h>


//Aerostack libraries
#include <behavior_process.h>


class BehaviorTakeOff2:public BehaviorProcess
{
  //Constructor
public:
  BehaviorTakeOff2();
  ~BehaviorTakeOff2();

private:
  ros::NodeHandle node_handle;

  //Congfig variables
  std::string drone_id;
  std::string drone_id_namespace;
  std::string my_stack_directory;
  std::string behavior_name_str;
  std::string estimated_pose_str;
  std::string controllers_str;
  std::string rotation_angles_str;
  std::string initialize_yaw_str;
  std::string execute_query_srv;
  std::string estimated_speed_str;
  std::string drone_control_mode_str;
  std::string yaw_controller_str;
  std::string service_topic_str;
  std::string drone_position_str;
  std::string mission_point_str;
  std::string speed_topic;
  std::string d_altitude_str;
  std::string trajectory_ref_command_str;


  //Subscriber---
  ros::Subscriber estimated_pose_sub;
  ros::Subscriber estimated_speed_sub;
  ros::Subscriber rotation_angles_sub;
  ros::Publisher controllers_pub;
  ros::Publisher yaw_controller_pub;
  ros::Publisher drone_position_pub;
  ros::Publisher  speed_topic_pub;
  ros::Publisher  d_altitude_pub;
  ros::Publisher  mission_point_pub;
  ros::Publisher reference_trajectory_pub;
  ros::ServiceClient mode_service;
  ros::ServiceClient initialize_yaw_cli;
  ros::ServiceClient query_client;

//Message
  droneMsgsROS::dronePose estimated_pose_msg;
  droneMsgsROS::dronePose static_pose;
  droneMsgsROS::droneSpeeds target_position;
  droneMsgsROS::droneSpeeds estimated_speed_msg;
  geometry_msgs::Vector3Stamped rotation_angles_msg;
  //Timer staticity_timer;
  bool first_position;
  bool taken_off;
  bool is_finished;
  double sp_altitude;
private: //BehaviorProcess
  void ownSetUp();
  void ownStart();
  void ownRun();
  void ownStop();
  std::tuple<bool, std::string> ownCheckSituation();

private:
  void endingImplementation();
  bool monitor();

public: //Callbacks
  void estimatedPoseCallBack(const droneMsgsROS::dronePose&);
  void rotationAnglesCallback(const geometry_msgs::Vector3Stamped&);
  void estimatedSpeedCallback(const droneMsgsROS::droneSpeeds&);

};


#endif
