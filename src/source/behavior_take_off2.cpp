#include "../include/behavior_take_off2.h"

BehaviorTakeOff2::BehaviorTakeOff2() : BehaviorProcess()
{

}

BehaviorTakeOff2::~BehaviorTakeOff2()
{

}


void BehaviorTakeOff2::ownSetUp()
{
  node_handle.param<std::string>("drone_id", drone_id, "1");
  node_handle.param<std::string>("drone_id_namespace", drone_id_namespace, "drone"+drone_id);
  node_handle.param<std::string>("my_stack_directory", my_stack_directory,
             "~/workspace/ros/quadrotor_stack_catkin/src/quadrotor_stack");

  estimated_pose_str = "estimated_pose";
  node_handle.param<std::string>("estimated_pose_topic", estimated_pose_str, "estimated_pose");
  node_handle.param<std::string>("estimated_speed_topic",estimated_speed_str,"estimated_speed");
  node_handle.param<std::string>("yaw_controller_str",yaw_controller_str , "droneControllerYawRefCommand");
  node_handle.param<std::string>("service_topic_str",service_topic_str , "droneTrajectoryController/setControlMode");
  node_handle.param<std::string>("drone_position_str",drone_position_str , "dronePositionRefs");
  node_handle.param<std::string>("speed_topic",speed_topic , "droneSpeedsRefs");
  node_handle.param<std::string>("drone_control_mode",drone_control_mode_str,"droneTrajectoryController/controlMode");
  node_handle.param<std::string>("d_altitude",d_altitude_str,"command/dAltitude");
  node_handle.param<std::string>("controllers_topic", controllers_str, "command/high_level");
  node_handle.param<std::string>("rotation_angles_topic", rotation_angles_str, "rotation_angles");
  node_handle.param<std::string>("initialize_yaw_srv", initialize_yaw_str, 
  "droneOdometryStateEstimator/setInitDroneYaw");
  node_handle.param<std::string>("consult_belief",execute_query_srv,"consult_belief");
}

void BehaviorTakeOff2::ownStart()
{
  /*Initialize topics*/
  estimated_pose_sub = node_handle.subscribe(estimated_pose_str, 1000, &BehaviorTakeOff2::estimatedPoseCallBack, this);
  rotation_angles_sub = node_handle.subscribe(rotation_angles_str, 1000, &BehaviorTakeOff2::rotationAnglesCallback, this);
  estimated_speed_sub = node_handle.subscribe(estimated_speed_str, 1000, &BehaviorTakeOff2::estimatedSpeedCallback, this);
  yaw_controller_pub = node_handle.advertise<droneMsgsROS::droneYawRefCommand>(yaw_controller_str,1000);
  mode_service = node_handle.serviceClient<droneMsgsROS::setControlMode>(service_topic_str);
  drone_position_pub = node_handle.advertise< droneMsgsROS::dronePositionRefCommandStamped>(drone_position_str,1000);
  speed_topic_pub = node_handle.advertise<droneMsgsROS::droneSpeeds>(speed_topic,1000);
  d_altitude_pub = node_handle.advertise<droneMsgsROS::droneDAltitudeCmd>(d_altitude_str,1);
  controllers_pub = node_handle.advertise<droneMsgsROS::droneCommand>(controllers_str, 1, true);
  initialize_yaw_cli = node_handle.serviceClient<droneMsgsROS::setInitDroneYaw_srv_type>(initialize_yaw_str);
  query_client = node_handle.serviceClient <droneMsgsROS::ConsultBelief> (execute_query_srv);

/*behavior implementation*/
  
  std::string arguments=getArguments();
  YAML::Node config_file = YAML::Load(arguments);
  if(config_file["altitude"].IsDefined())
  {
    double altitude=config_file["altitude"].as<double>();
    std::cout<<"Altitude found: "<< altitude <<std::endl;
    sp_altitude=altitude;
  }
  else
  {
    std::cout<<"Could not read altitude!"<<std::endl;
    sp_altitude=0.7;
  }
  
  std_msgs::Header header;
  header.frame_id = "behavior_take_off2";

  droneMsgsROS::droneCommand msg;
  msg.header = header;
  msg.command = droneMsgsROS::droneCommand::TAKE_OFF;
  controllers_pub.publish(msg);
  estimated_pose_msg.z = 0;
  first_position = false;
  is_finished = false;
  taken_off=false;

}

void BehaviorTakeOff2::ownRun()
{
  if(!is_finished){
    monitor();
  }
}

//ownRun
void BehaviorTakeOff2::ownStop()
{
  std::cout << "ownStop" << std::endl;
  std_msgs::Header header;
  header.frame_id = "behavior_land";

  droneMsgsROS::droneCommand msg;
  msg.header = header;
  msg.command = droneMsgsROS::droneCommand::HOVER;
  controllers_pub.publish(msg);

  speed_topic_pub.shutdown();
  d_altitude_pub.shutdown();
  estimated_pose_sub.shutdown();
  rotation_angles_sub.shutdown();
  controllers_pub.shutdown();
  initialize_yaw_cli.shutdown();
  estimated_speed_sub.shutdown();
  yaw_controller_pub.shutdown();
  drone_position_pub.shutdown();
}

std::tuple<bool,std::string> BehaviorTakeOff2::ownCheckSituation()
{
  droneMsgsROS::ConsultBelief query_service;
  std::ostringstream capturador;
  capturador << "battery_level(self,LOW)";
  std::string query(capturador.str());
  query_service.request.query = query;
  query_client.call(query_service);
  if(query_service.response.success)
  {
    return std::make_tuple(false,"Error: Battery low, unable to perform action");
  }
  std::ostringstream capturador2;
  capturador2<<"flight_state(self,FLYING)";
  std::string query2(capturador2.str());
  query_service.request.query = query2;
  query_client.call(query_service);
  if(query_service.response.success)
  {
    return std::make_tuple(false,"Error: Already flying");
  }

  return std::make_tuple(true,"");
}

//Private functions
bool BehaviorTakeOff2::monitor()
{
  double precision_take_off = 0.1;

  //Check achievement
  if(!taken_off&&(std::abs(std::abs(estimated_pose_msg.z) - 0.6) < precision_take_off)){
    taken_off=true;
    estimated_speed_msg = *ros::topic::waitForMessage<droneMsgsROS::droneSpeeds>(estimated_speed_str, node_handle, ros::Duration(2));
    estimated_pose_msg = *ros::topic::waitForMessage<droneMsgsROS::dronePose>(estimated_pose_str, node_handle, ros::Duration(2));
    droneMsgsROS::setControlMode mode;
    mode.request.controlMode.command=mode.request.controlMode.SPEED_CONTROL;
    mode_service.call(mode);

    droneMsgsROS::droneSpeeds point;
    ros::topic::waitForMessage<droneMsgsROS::droneTrajectoryControllerControlMode>(
      drone_control_mode_str, node_handle);
    point.dx=0;
    point.dy=0;
    point.dz=5;
    speed_topic_pub.publish(point);
    droneMsgsROS::droneCommand msg;
    msg.command = droneMsgsROS::droneCommand::MOVE;
    controllers_pub.publish(msg);

    estimated_speed_msg = *ros::topic::waitForMessage<droneMsgsROS::droneSpeeds>(estimated_speed_str, node_handle, ros::Duration(2));
    droneMsgsROS::droneDAltitudeCmd d_altitude_msg;
    d_altitude_msg.dAltitudeCmd = 0.25;
    d_altitude_pub.publish(d_altitude_msg);
  }
  if(taken_off && (std::abs(std::abs(estimated_pose_msg.z) - sp_altitude) < precision_take_off))
  {
    droneMsgsROS::droneDAltitudeCmd d_altitude_msg;
    d_altitude_msg.dAltitudeCmd = 0.0;
    d_altitude_pub.publish(d_altitude_msg);
    BehaviorProcess::setFinishEvent(droneMsgsROS::BehaviorEvent::GOAL_ACHIEVED);
    BehaviorProcess::setFinishConditionSatisfied(true);
    endingImplementation();
    printf("done!\n");
    is_finished = true;
    return true;
  }
  
  //Check timeout
  if(timerIsFinished()){
    BehaviorProcess::setFinishEvent(droneMsgsROS::BehaviorEvent::TIME_OUT);
    BehaviorProcess::setFinishConditionSatisfied(true);
    endingImplementation();
    is_finished = true;
    return true;
  }
  return false;
}

void BehaviorTakeOff2::endingImplementation()
{
  droneMsgsROS::setInitDroneYaw_srv_type init_yaw_msg;
  init_yaw_msg.request.yaw_droneLMrT_telemetry_rad = (rotation_angles_msg.vector.z)*(M_PI/180.0);
  initialize_yaw_cli.call(init_yaw_msg);

  droneMsgsROS::droneCommand msg;
  msg.command = droneMsgsROS::droneCommand::HOVER;
  controllers_pub.publish(msg);
}

//Custom topic Callbacks
void BehaviorTakeOff2::estimatedSpeedCallback(const droneMsgsROS::droneSpeeds& msg)
{
    estimated_speed_msg=msg;
}

void BehaviorTakeOff2::estimatedPoseCallBack(const droneMsgsROS::dronePose& message)
{
  if(!first_position)
    static_pose = message;
  estimated_pose_msg = message;
}

void BehaviorTakeOff2::rotationAnglesCallback(const geometry_msgs::Vector3Stamped& message)
{
  rotation_angles_msg = message;
}
