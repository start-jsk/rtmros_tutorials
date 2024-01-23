#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")

(hironxjsk-init)

(setq pub-rate (ros::get-param "~publish_rate" 5))

(ros::advertise "joint_states" sensor_msgs::JointState 1)
(ros::rate pub-rate)
(while (ros::ok)
  (setq msg (instance sensor_msgs::JointState :init))
  (send msg :header :stamp (ros::time-now))
  (send msg :name (list "RHAND_JOINT0" "RHAND_JOINT1" "RHAND_JOINT2" "RHAND_JOINT3" "LHAND_JOINT0" "LHAND_JOINT1" "LHAND_JOINT2" "LHAND_JOINT3"))
  (send msg :position (map float-vector #'deg2rad (send *ri* :hand-angle-vector :hands)))
  (ros::publish "joint_states" msg)
  (ros::sleep)
  )
