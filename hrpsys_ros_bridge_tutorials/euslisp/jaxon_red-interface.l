(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :jaxon_red "package://hrpsys_ros_bridge_tutorials/models/jaxon_red.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-utils.l"))
  (require :jaxon_red-utils "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-utils.l"))

(defclass jaxon_red-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod jaxon_red-interface
  (:init (&rest args)
         (send-super* :init :robot jaxon_red-robot args)))

(when (probe-file (ros::resolve-ros-path "package://thk_hand_controller/euslisp/thkhand-simple-controller.l"))
  (load "package://thk_hand_controller/euslisp/thkhand-simple-controller.l")
  (unless (assoc :init-org (send jaxon_red-interface :methods))
    (rplaca (assoc :init (send jaxon_red-interface :methods)) :init-org))
  (defmethod jaxon_red-interface
    (:init
     (&rest args)
     (prog1
         (send* self :init-org args)
       (print 1)
       (send self :put :hand-controller (instance thkhand-simple-controller :init))
       ;; add controller
       (send self :add-controller :larm-controller :joint-enable-check t :create-actions t)
       (send self :add-controller :rarm-controller :joint-enable-check t :create-actions t)
       (send self :add-controller :lleg-controller :joint-enable-check t :create-actions t)
       (send self :add-controller :rleg-controller :joint-enable-check t :create-actions t)
       )
     )
    (:move-gripper
     (&rest args)
     (send* (send self :get :hand-controller) :move-gripper args))
    (:start-grasp
     (&rest args)
     (send* (send self :get :hand-controller) :start-grasp args))
    (:stop-grasp
     (&rest args)
     (send* (send self :get :hand-controller) :stop-grasp args))
    (:hand-reset
     (&rest args)
     (send* (send self :get :hand-controller) :reset args))
    ;; (:hand-open
    ;;  (&rest args)
    ;;  (send* (send self :get :hand-controller) :open args))
    ;; (:hand-close
    ;;  (&rest args)
    ;;  (send* (send self :get :hand-controller) :close args))
    (:hand-stop
     (&rest args)
     (send* (send self :get :hand-controller) :stop args))
    (:hand-resume
     (&rest args)
     (send* (send self :get :hand-controller) :resume args))
    (:get-joint-angle
     (&rest args)
     (send* (send self :get :hand-controller) :get-joint-angle args))
    (:get-joint-velocity
     (&rest args)
     (send* (send self :get :hand-controller) :get-joint-velocity args))
    (:get-joint-effort
     (&rest args)
     (send* (send self :get :hand-controller) :get-joint-effort args))
    ;; controller group configuration
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
  )

(defun jaxon_red-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* jaxon_red-interface :init args)))
  (if (not (boundp '*jaxon_red*))
      (setq *jaxon_red* (instance jaxon_red-robot :init))))
