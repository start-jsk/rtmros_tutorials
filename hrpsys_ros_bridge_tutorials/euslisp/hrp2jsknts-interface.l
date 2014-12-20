(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :hrp2jsknts "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknts.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-utils.l"))
  (require :hrp2jsknts-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-utils.l"))
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2-common-interface.l")
(if (ros::resolve-ros-path "package://jsk_hrp2_ros_bridge")
    (ros::load-ros-manifest "jsk_hrp2_ros_bridge"))

(defclass hrp2jsknts-interface
  :super hrp2-common-interface
  :slots ())
(defmethod hrp2jsknts-interface
  (:init (&rest args)
         (send-super* :init :robot hrp2jsknts-robot args)))

(defun hrp2jsknts-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* hrp2jsknts-interface :init args)))
  (if (not (boundp '*hrp2jsknts*))
      (setq *hrp2jsknts* (instance hrp2jsknts-robot :init))))
