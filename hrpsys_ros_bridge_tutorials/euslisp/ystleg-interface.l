(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :ystleg "package://hrpsys_ros_bridge_tutorials/models/ystleg.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/ystleg-utils.l"))
  (require :ystleg-utils "package://hrpsys_ros_bridge_tutorials/euslisp/ystleg-utils.l"))

(defclass ystleg-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod ystleg-interface
  (:init (&rest args)
         (send-super* :init :robot ystleg-robot args)))

(defun ystleg-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* ystleg-interface :init args)))
  (if (not (boundp '*ystleg*))
      (setq *ystleg* (instance ystleg-robot :init))))
