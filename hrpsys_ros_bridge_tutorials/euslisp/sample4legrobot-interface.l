(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :sample4legrobot "package://hrpsys_ros_bridge_tutorials/models/sample4legrobot.l")

(defclass sample4legrobot-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod sample4legrobot-interface
  (:init (&rest args)
         (send-super* :init :robot sample4legrobot-robot args)))

(defun sample4legrobot-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* sample4legrobot-interface :init args)))
  (if (not (boundp '*s4lr*))
      (setq *s4lr* (instance sample4legrobot-robot :init)))
  )
