(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :hrp2jsk "package://hrpsys_ros_bridge_tutorials/models/hrp2jsk.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-utils.l"))
  (require :hrp2jsk-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-utils.l"))

(defclass hrp2jsk-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod hrp2jsk-interface
  (:init (&rest args)
         (send-super :init :robot hrp2jsk-robot)))

(defun hrp2jsk-init ()
  (if (not (boundp '*ri*))
      (setq *ri* (instance hrp2jsk-interface :init)))
  (if (not (boundp '*hrp2jsk*))
      (setq *hrp2jsk* (instance hrp2jsk-robot :init))))
