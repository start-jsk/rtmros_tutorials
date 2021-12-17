(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :tqleg0 "package://hrpsys_ros_bridge_tutorials/models/a0b.l")

(defclass a0b-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod a0b-interface
  (:init (&rest args &key ((:controller-timeout ct) nil))
         (send-super* :init :robot a0b-robot :controller-timeout ct args)))

(defun a0b-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* a0b-interface :init args)))
  (if (not (boundp '*a0b*))
      (setq *a0b* (instance a0b-robot :init))))
