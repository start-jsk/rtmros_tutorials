(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :jaxon_red "package://hrpsys_ros_bridge_tutorials/models/jaxon_red.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-utils.l"))
  (require :jaxon_red-utils "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-utils.l"))
(require :jaxon-common-interface "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-common-interface.l")

(defclass jaxon_red-interface
  :super jaxon-common-interface
  :slots ())
(defmethod jaxon_red-interface
  (:init
   (&rest args &key ((:controller-timeout ct) nil))
   (prog1
       (send-super* :init :robot jaxon_red-robot :controller-timeout ct args)
     (print ";; Add hands")
     (setq hand-enable (send self :set-hand-controller))
     (when (and (ros::get-param "/use_sim_time")
                (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_jvrc/euslisp/jvrc-hand-interface.l")))
       (load "package://hrpsys_ros_bridge_jvrc/euslisp/jvrc-hand-interface.l")
       (send self :put :hand-controller (instance jvrc-hand-controller :init))
       (setq hand-enable t))
     )
   )
  (:set-hand-controller () nil)
  )

(when (probe-file (ros::resolve-ros-path "package://thk_hand_controller/euslisp/thkhand-simple-controller.l"))
  (load "package://thk_hand_controller/euslisp/thkhand-simple-controller.l")
  (defmethod jaxon_red-interface
    (:set-hand-controller () (send self :put :hand-controller (instance thkhand-simple-controller :init)) t)
    )
  )

(defun jaxon_red-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* jaxon_red-interface :init args)))
  (if (not (boundp '*jaxon_red*))
      (setq *jaxon_red* (instance jaxon_red-robot :init))))
