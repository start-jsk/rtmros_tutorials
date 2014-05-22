(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :urataleg "package://hrpsys_ros_bridge_tutorials/models/urataleg.l")

(defclass urataleg-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod urataleg-interface
  (:init
   (&rest args)
   (prog1
       (send-super* :init :robot urataleg-robot args)
     (send self :set-st-param
           :k-tpcc-p #f(2.5 2.5)
           :k-tpcc-x #f(5 5)
           :k-brot-p #f(0 0)
           :k-brot-tc #f(0.1 0.1))
     (send self :set-auto-balancer-param
           :default-zmp-offsets (list (float-vector 20 -15 0) (float-vector 20 15 0)))
     ))
  )

(defun urataleg-init ()
  (if (not (boundp '*ri*))
      (setq *ri* (instance urataleg-interface :init)))
  (if (not (boundp '*urataleg*))
      (setq *urataleg* (instance urataleg-robot :init))))
