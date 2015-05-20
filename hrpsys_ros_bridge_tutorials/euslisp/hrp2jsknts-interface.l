(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :hrp2jsknts "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknts.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-utils.l"))
  (require :hrp2jsknts-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-utils.l"))
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2-common-interface.l")
(if (ros::resolve-ros-path "package://jsk_hrp2_ros_bridge")
    (ros::load-ros-manifest "jsk_hrp2_ros_bridge"))

(defclass hrp2jsknts-interface
  :super hrp2-common-interface
  :slots ())
(defmethod hrp2jsknts-interface
  (:init (&rest args)
         (send-super* :init :robot hrp2jsknts-robot args)
         ;; add controller
         (send self :add-controller :larm-controller :joint-enable-check t :create-actions t)
         (send self :add-controller :rarm-controller :joint-enable-check t :create-actions t)
         (send self :add-controller :lleg-controller :joint-enable-check t :create-actions t)
         (send self :add-controller :rleg-controller :joint-enable-check t :create-actions t)
         )
  (:larm-controller
   ()
   (list
    (list
     (cons :group-name "larm")
     (cons :controller-action "larm_controller/joint_trajectory_action")
     (cons :controller-state "larm_controller/state")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (list "LARM_JOINT0" "LARM_JOINT1" "LARM_JOINT2" "LARM_JOINT3"
                              "LARM_JOINT4" "LARM_JOINT5" "LARM_JOINT6" "LARM_JOINT7"))))
   )
  (:rarm-controller
   ()
   (list
    (list
     (cons :group-name "rarm")
     (cons :controller-action "rarm_controller/joint_trajectory_action")
     (cons :controller-state "rarm_controller/state")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (list "RARM_JOINT0" "RARM_JOINT1" "RARM_JOINT2" "RARM_JOINT3"
                              "RARM_JOINT4" "RARM_JOINT5" "RARM_JOINT6" "RARM_JOINT7"))))
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
                              "LLEG_JOINT3" "LLEG_JOINT4" "LLEG_JOINT5" "LLEG_JOINT6"))))
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
                              "RLEG_JOINT3" "RLEG_JOINT4" "RLEG_JOINT5" "RLEG_JOINT6"))))
   )
  )

(defun hrp2jsknts-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* hrp2jsknts-interface :init args)))
  (if (not (boundp '*hrp2jsknts*))
      (setq *hrp2jsknts* (instance hrp2jsknts-robot :init))))
