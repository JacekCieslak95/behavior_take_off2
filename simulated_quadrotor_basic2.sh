#!/bin/bash

#---------------------------------------------------------------------------------------------
# FILE:   simulated_quadrotor.sh
#
# BRIEF:  Launches the execution of processes for a generic simulated quadrotor
#
# DETAIL: Includes the following configuration of processes:
#         - Basic Aerostack processes
#         - Simulation of a generic quadrotor
#         - Recognizes aruco markers
#         - Human machine interface
#---------------------------------------------------------------------------------------------

#---------------------------------------------------------------------------------------------
# Input arguments
#---------------------------------------------------------------------------------------------
NUMID_DRONE=$1
NETWORK_ROSCORE=$2
DRONE_IP=$3
DRONE_WCHANNEL=$4

#---------------------------------------------------------------------------------------------
# Default values for arguments
#
# This code checks the existence of arguments as it is explained at:
#      http://stackoverflow.com/questions/6482377/bash-shell-script-check-input-argument
#---------------------------------------------------------------------------------------------
if [ -z $NETWORK_ROSCORE ] # Check if NETWORK_ROSCORE is NULL
  then
    # Argument 2 is empty
    . ${AEROSTACK_STACK}/setup.sh
    OPEN_ROSCORE=1
  else
   . ${AEROSTACK_STACK}/setup.sh $2
fi

if [ -z $NUMID_DRONE ] # Check if NUMID_DRONE is NULL
  then
    # Argument 1 empty
    echo "-Setting droneId = 1"
    NUMID_DRONE=1
  else
    echo "-Setting droneId = $1"
fi

if [ -z $DRONE_IP ] # Check if NUMID_DRONE is NULL
  then
    # Argument 3 is empty
    echo "-Setting droneIp = 192.168.1.1"
    DRONE_IP=192.168.1.1
  else
    echo "-Setting droneIp = $3"
  fi

if [ -z $DRONE_WCHANNEL ] # Check if NUMID_DRONE is NULL
  then
    # Argument 4 is empty
    echo "-Setting droneChannel = 6"
    DRONE_WCHANNEL=6
  else
    echo "-Setting droneChannel = $4"
  fi

#---------------------------------------------------------------------------------------------
# INTERNAL PROCESSES
#---------------------------------------------------------------------------------------------
gnome-terminal  \
`#---------------------------------------------------------------------------------------------` \
`# Quadrotor simulator                                                                         ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Simulator"	--command "bash -c \"
roslaunch droneSimulatorROSModule droneSimulatorROSModule.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id_int:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Self localizer (State estimator)                                                            ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "State Estimator"	--command "bash -c \"
roslaunch droneEKFStateEstimatorROSModule droneEKFStateEstimatorROSModule.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id_int:=$NUMID_DRONE my_stack_directory:=${AEROSTACK_STACK};
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Trajectory controller                                                                       ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Trajectory Controller"	--command "bash -c \"
roslaunch droneTrajectoryControllerROSModule droneTrajectoryControllerROSModule.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id_int:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK} \
    drone_estimated_pose_topic_name:=estimated_pose \
    drone_estimated_speeds_topic_name:=estimated_speed;
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Obstacle distance calculator                                                                ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Obstacle Distance Calculator" --command "bash -c \"
roslaunch droneObstacleDistanceCalculatorROSModule droneObstacleDistanceCalculationROSModule.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id_int:=$NUMID_DRONE my_stack_directory:=${AEROSTACK_STACK} \
    drone_pose_topic_name:=estimated_pose;
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Obstacle Processor                                                                          ` \
`# Identifies obstacles according to the Visual Markers                                        ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Obstacle Processor" --command "bash -c \"
roslaunch droneObstacleProcessorVisualMarksROSModule droneObstacleProcessor2dVisualMarksROSModule.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id_int:=$NUMID_DRONE \
    my_stack_directory:=${DRONE_STACK};
exec bash\""   \
`#---------------------------------------------------------------------------------------------` \
`# Aruco Eye                                                                                   ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "ArucoEye" --command "bash -c \"
roslaunch drone_aruco_eye_ros_module droneArucoEyeROSModule.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id_int:=$NUMID_DRONE \
    my_stack_directory:=${DRONE_STACK};
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Visual Markers Localizer                                                                    ` \
`# Finds and recognizes Visual Markers                                                         ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Visual Marker Localizer" --command "bash -c \"
roslaunch droneVisualMarkersLocalizerROSModule droneVisualMarkersLocalizerROSModule.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id_int:=$NUMID_DRONE \
    my_stack_directory:=${DRONE_STACK};
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Trajectory planner                                                                          ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Trajectory Planner" --command "bash -c \"
roslaunch droneTrajectoryPlannerROSModule droneTrajectoryPlanner2dROSModule.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id_int:=$NUMID_DRONE my_stack_directory:=${AEROSTACK_STACK} \
    drone_pose_topic_name:=estimated_pose;
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Yaw commander                                                                               ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Yaw Commander" --command "bash -c \"
roslaunch droneYawCommanderROSModule droneYawCommanderROSModule.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id_int:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK} \
    drone_pose_topic_name:=estimated_pose;
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Communication manager                                                                       ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Communication Manager" --command "bash -c \"
roslaunch droneCommunicationManagerROSModule droneCommunicationManagerROSModule.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id_int:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK} \
    estimated_pose_topic_name:=estimated_pose;
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Process monitor                                                                             ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Process Monitor"	--command "bash -c \"
roslaunch process_monitor_process process_monitor.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id_int:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Behavior Coordinator                                                                        ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior Coordinator" --command "bash -c \"
roslaunch behavior_coordinator_process behavior_coordinator_process.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Behavior Specialist                                                                         ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior Specialist" --command "bash -c \"
roslaunch behavior_specialist_process behavior_specialist_process.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Resource Manager                                                                            ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Resource Manager" --command "bash -c \"
roslaunch resource_manager_process resource_manager_process.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Belief Manager                                                                              ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Belief Manager" --command "bash -c \"
roslaunch belief_manager_process belief_manager_process.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Belief Updater                                                                              ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Belief Updater" --command "bash -c \"
roslaunch belief_updater_process belief_updater_process.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Python based mission interpreter                                                            ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Python based mission interpreter" --command "bash -c \"
roslaunch python_based_mission_interpreter_process python_based_mission_interpreter_process.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    mission:=mission.py \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Task based mission planner                                                                  ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Task based mission planner" --command "bash -c \"
roslaunch task_based_mission_planner_process task_based_mission_planner_process.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Self Localization Selector Process                                                          ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Self Localization Selector" --command "bash -c \"
roslaunch self_localization_selector_process self_localization_selector_process.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Behaviors                                                                                   ` \
`#---------------------------------------------------------------------------------------------` \
`#---------------------------------------------------------------------------------------------` \
`# Behavior TakeOff                                                                            ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior TakeOff" --command "bash -c \"
roslaunch behavior_take_off behavior_take_off.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Behavior Land                                                                               ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior Land" --command "bash -c \"
roslaunch behavior_land behavior_land.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Behavior GoToPoint                                                                          ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior GoToPoint" --command "bash -c \"
roslaunch behavior_go_to_point behavior_go_to_point.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Behavior FollowObjectImage                                                                  ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior Follow Object Image" --command "bash -c \"
roslaunch behavior_follow_object_image behavior_follow_object_image.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Behavior Keep Hovering                                                                       ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior Keep Hovering" --command "bash -c \"
roslaunch behavior_keep_hovering behavior_keep_hovering.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Behavior Keep Moving                                                                        ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior Keep Moving" --command "bash -c \"
roslaunch behavior_keep_moving behavior_keep_moving.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Behavior Pay Attention to visual markers                                                    ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior Pay Attention to Visual Markers" --command "bash -c \"
roslaunch behavior_pay_attention_to_visual_markers behavior_pay_attention_to_visual_markers.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Behavior Rotate                                                                             ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior Rotate" --command "bash -c \"
roslaunch behavior_rotate behavior_rotate.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Behavior Self Localize by odometry                                                          ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior Self Localize by odometry" --command "bash -c \"
roslaunch behavior_self_localize_by_odometry behavior_self_localize_by_odometry.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Behavior Self Localize by Visual markers                                                                 ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior Self Localize by Visual Marker" --command "bash -c \"
roslaunch behavior_self_localize_by_visual_marker behavior_self_localize_by_visual_marker.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Behavior TakeOff2                                                                           ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior TakeOff2" --command "bash -c \"
roslaunch behavior_take_off2 behavior_take_off2.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Behavior LookAtPoint                                                                           ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior LookAtPoint" --command "bash -c \"
roslaunch behavior_look_at_point behavior_look_at_point.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Behavior GoToPointAngle                                                                           ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior GoToPointAngle" --command "bash -c \"
roslaunch behavior_go_to_point_angle behavior_go_to_point_angle.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Behavior Wait                                                                 ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior Wait" --command "bash -c \"
roslaunch behavior_wait behavior_wait.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\"" &
#---------------------------------------------------------------------------------------------
# USER INTERFACE PROCESSES
#---------------------------------------------------------------------------------------------
gnome-terminal  \
\
`#---------------------------------------------------------------------------------------------` \
`# Shell Interface                                                                             ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Shell User Interface"	--command "bash -c \"
roslaunch droneInterfaceROSModule droneInterface_jp_ROSModule.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id_int:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# RViz Interface                                                                              ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "RViz Interface"  --command "bash -c \"
roslaunch droneArchitectureRvizROSModule droneArchitectureRvizInterfaceROSModule.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id_int:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# HMI Interface                                                                               ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "HMI"  --command "bash -c \"
roslaunch graphical_user_interface gui.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id_int:=$NUMID_DRONE  \
    drone_pose_subscription:=estimated_pose \
    drone_speeds_subscription:=estimated_speeds \
    my_stack_directory:=${AEROSTACK_STACK};
exec bash\"" &
