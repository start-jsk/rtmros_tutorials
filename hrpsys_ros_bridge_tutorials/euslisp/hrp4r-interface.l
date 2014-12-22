(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :hrp4r "package://hrpsys_ros_bridge_tutorials/models/hrp4r.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hrp4r-utils.l"))
  (require :hrp4r-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hrp4r-utils.l"))

(defclass hrp4r-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod hrp4r-interface
  (:init (&rest args)
         (send-super* :init :robot hrp4r-robot args)))

(defun hrp4r-init ()
  (if (not (boundp '*ri*))
      (setq *ri* (instance hrp4r-interface :init)))
  (if (not (boundp '*hrp4r*))
      (setq *hrp4r* (instance hrp4r-robot :init))))
