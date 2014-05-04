(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :hrp2jsknt "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknt.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-utils.l"))
  (require :hrp2jsknt-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-utils.l"))
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2-common-interface.l")
(if (ros::resolve-ros-path "package://jsk_hrp2_ros_bridge")
    (ros::load-ros-manifest "jsk_hrp2_ros_bridge"))

(defclass hrp2jsknt-interface
  :super hrp2-common-interface
  :slots ())
(defmethod hrp2jsknt-interface
  (:init (&rest args)
         (send-super :init :robot hrp2jsknt-robot)))

(defun hrp2jsknt-init ()
  (if (not (boundp '*ri*))
      (setq *ri* (instance hrp2jsknt-interface :init)))
  (if (not (boundp '*hrp2jsknt*))
      (setq *hrp2jsknt* (instance hrp2jsknt-robot :init))))
