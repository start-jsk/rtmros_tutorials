(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :jaxon "package://hrpsys_ros_bridge_tutorials/models/jaxon.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-utils.l"))
  (require :jaxon-utils "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-utils.l"))

(defclass jaxon-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod jaxon-interface
  (:init (&rest args)
         (send-super* :init :robot jaxon-robot args)))

(defun jaxon-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* jaxon-interface :init args)))
  (if (not (boundp '*jaxon*))
      (setq *jaxon* (instance jaxon-robot :init))))
