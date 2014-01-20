(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :testmdofarm "package://hrpsys_ros_bridge_tutorials/models/testmdofarm.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/testmdofarm-utils.l"))
  (require :testmdofarm-utils "package://hrpsys_ros_bridge_tutorials/euslisp/testmdofarm-utils.l"))

(defclass testmdofarm-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod testmdofarm-interface
  (:init (&rest args)
         (send-super :init :robot testmdofarm-robot)))

(defun testmdofarm-init ()
  (if (not (boundp '*ri*))
      (setq *ri* (instance testmdofarm-interface :init)))
  (if (not (boundp '*testmdofarm*))
      (setq *testmdofarm* (instance testmdofarm-robot :init)))
  (objects (list *testmdofarm*))
  )
