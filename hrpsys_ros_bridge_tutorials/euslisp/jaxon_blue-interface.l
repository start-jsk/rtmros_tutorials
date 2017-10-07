(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :jaxon_blue "package://hrpsys_ros_bridge_tutorials/models/jaxon_blue.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_blue-utils.l"))
  (require :jaxon_blue-utils "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_blue-utils.l"))
(require :jaxon-common-interface "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-common-interface.l")

(defclass jaxon_blue-interface
  :super jaxon-common-interface
  :slots ())
(defmethod jaxon_blue-interface
  (:init
   (&rest args &key ((:controller-timeout ct) nil))
   (prog1
       (send-super* :init :robot jaxon_blue-robot :controller-timeout ct args)
     (print ";; Add hands")
     (setq hand-enable (send self :set-hand-controller))
     ;; (when (and (ros::get-param "/use_sim_time")
     ;;            (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_jvrc/euslisp/jvrc-hand-interface.l")))
     ;;   (load "package://hrpsys_ros_bridge_jvrc/euslisp/jvrc-hand-interface.l")
     ;;   (send self :put :hand-controller (instance jvrc-hand-controller :init))
     ;;   (setq hand-enable t))
     )
   )
  (:set-hand-controller () nil)
  )

;; (when (probe-file (ros::resolve-ros-path "package://thk_hand_controller/euslisp/thkhand-simple-controller.l"))
;;   (load "package://thk_hand_controller/euslisp/thkhand-simple-controller.l")
;;   (defmethod jaxon_blue-interface
;;     (:set-hand-controller () (send self :put :hand-controller (instance thkhand-simple-controller :init)) t)
;;     )
;;   )

(defun jaxon_blue-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* jaxon_blue-interface :init args)))
  (if (not (boundp '*jaxon_blue*))
      (setq *jaxon_blue* (instance jaxon_blue-robot :init))))
