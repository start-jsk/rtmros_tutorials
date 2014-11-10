(require :staro "package://hrpsys_ros_bridge_tutorials/models/staro.l")
(require :staro-utils "package://hrpsys_ros_bridge_tutorials/euslisp/staro-utils.l")
(require :rtm-ros-robot-interface "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")

(defclass staro-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod staro-interface
  (:init
   (&rest args)
   (send-super* :init :robot staro-robot args))
  )

(defun staro-init ()
  (if (not (boundp '*ri*))
      (setq *ri* (instance staro-interface :init)))
  (if (not (boundp '*staro*))
      (setq *staro* (instance staro-robot :init)))
  (unless (boundp '*irtviewer*) (make-irtviewer :width 320 :height 320) (send *irtviewer* :change-background #f(0.3 0.5 0.7)))
  (objects (list *staro*))
  )
