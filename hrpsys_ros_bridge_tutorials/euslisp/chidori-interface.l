(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :chidori "package://hrpsys_ros_bridge_tutorials/models/chidori.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/chidori-utils.l"))
  (require :chidori-utils "package://hrpsys_ros_bridge_tutorials/euslisp/chidori-utils.l"))

(defclass chidori-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod chidori-interface
  (:init
   (&rest args &key ((:controller-timeout ct) nil))
   (prog1
       (send-super* :init :robot chidori-robot :controller-timeout ct args)
     (print 1)
     ;; add controller
     (send self :add-controller :lleg-controller :joint-enable-check t :create-actions t)
     (send self :add-controller :rleg-controller :joint-enable-check t :create-actions t)
     )
   )
  (:lleg-controller
   ()
   (list
    (list
     (cons :group-name "lleg")
     (cons :controller-action "lleg_controller/joint_trajectory_action")
     (cons :controller-state "lleg_controller/state")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (list "LLEG_JOINT0" "LLEG_JOINT1" "LLEG_JOINT2"
                              "LLEG_JOINT3" "LLEG_JOINT4" "LLEG_JOINT5"))))
   )
  (:rleg-controller
   ()
   (list
    (list
     (cons :group-name "rleg")
     (cons :controller-action "rleg_controller/joint_trajectory_action")
     (cons :controller-state "rleg_controller/state")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (list "RLEG_JOINT0" "RLEG_JOINT1" "RLEG_JOINT2"
                              "RLEG_JOINT3" "RLEG_JOINT4" "RLEG_JOINT5"))))
   )
  )

(defun chidori-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* chidori-interface :init args)))
  (if (not (boundp '*chidori*))
      (setq *chidori* (instance chidori-robot :init))))
