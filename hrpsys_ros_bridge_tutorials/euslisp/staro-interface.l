(require :staro "package://hrpsys_ros_bridge_tutorials/models/staro.l")
(require :staro-utils "package://hrpsys_ros_bridge_tutorials/euslisp/staro-utils.l")
(require :rtm-ros-robot-interface "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")

(defclass staro-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod staro-interface
  (:init
   (&rest args)
   (prog1
       (send-super :init :robot staro-robot)
     ;; add controller
     (send self :add-controller :larm-controller :create-actions t)
     (send self :add-controller :rarm-controller :create-actions t)
     (send self :add-controller :lleg-controller :create-actions t)
     (send self :add-controller :rleg-controller :create-actions t)
     (send self :add-controller :torso-controller :create-actions t)
     (send self :add-controller :head-controller :create-actions t)
     ;; add robotiq
     (if (ros::rospack-find "robotiq_s_model_control")
         (progn
           (load "package://staro_ros_bridge/euslisp/robotiq-interface.l")
           (send self :put :lhand-interface (instance robotiq-interface :init
                                                      (send (send self :robot) :hand :larm) "10.68.0.21"
                                                      :output "leftSModelRobotOutput" :input "leftSModelRobotInput"))
           (send self :put :rhand-interface (instance robotiq-interface :init
                                                      (send (send self :robot) :hand :rarm) "10.68.0.22"
                                                      :output "rightSModelRobotOutput" :input "rightSModelRobotInput"))
           ;; RobotiqHand model is already in wrl of staro
           ;; (mapcar #'(lambda (l lm)
           ;;             (send (send self :get lm) :newcoords
           ;;                   (send (send self l :end-coords :parent) :copy-worldcoords))
           ;;             (send (send self l :end-coords :parent) :assoc (send self :get lm)))
           ;;         '(:larm :rarm) '(:lhand-model :rhand-model))
           ;; (setq bodies (append bodies
           ;;                      (send (send self :get :lhand-model) :bodies)
           ;;                      (send (send self :get :rhand-model) :bodies)))
           )
       (warn  "~c[33m [WARN] robotiq_s_model_control for RobotiqHand cannnot be found. skip definition for RobotiqHand methods. ~c[m~%" #x1b #x1b) ;; ~c: escape sequence
       )
     )
   )
  ;; define hand interface
  (:hand
   (arm &rest args)
   (let (target ret)
     (case arm
       (:larm
        (setq target (send self :get :lhand-interface))
        (setq ret (forward-message-to target args)))
       (:rarm
        (setq target (send self :get :rhand-interface))
        (setq ret (forward-message-to target args)))
       ((:both :arms)
        (setq target (send self :get :lhand-interface))
        (push (forward-message-to target args) ret)
        (setq target (send self :get :rhand-interface))
        (push (forward-message-to target args) ret))
       )
     ret)
   )
  ;; wrapper for staro
  (:angle-vector
   (av &optional (tm 20000) (ctype controller-type))
   (send-super :angle-vector av tm ctype)
   )
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
  (:torso-controller
   ()
   (list
    (list
     (cons :group-name "torso")
     (cons :controller-action "torso_controller/joint_trajectory_action")
     (cons :controller-state "torso_controller/state")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (list "CHEST_JOINT0" "CHEST_JOINT1"))))
   )
  (:head-controller
   ()
   (list
    (list
     (cons :group-name "head")
     (cons :controller-action "head_controller/joint_trajectory_action")
     (cons :controller-state "head_controller/state")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (list "HEAD_JOINT0" "HEAD_JOINT1"))))
   )
  (:sync-controller
   (controller &optional (interpolation-time 5000) (blockp t))
   (let* ((controller-info (car (send self controller)))
          (groupname (cdr (assoc :group-name controller-info)))
          (jointnames (cdr (assoc :joint-names controller-info)))
          (current-angle-vector (send self :state :potentio-vector)))
     (warn "sync controller ~A~%" controller)
     (send self :angle-vector current-angle-vector interpolation-time controller)
     (when blockp
       (send self :wait-interpolation)
       )
     (send self :angle-vector current-angle-vector interpolation-time :default-controller)
     (when blockp
       (send self :wait-interpolation)
       )
     (send self :remove-joint-group groupname)
     (send self :wait-interpolation) ;; wait until removing
     (send self :add-joint-group groupname jointnames)
     )
   )
  
  )

(defun staro-init ()
  (if (not (boundp '*ri*))
      (setq *ri* (instance staro-interface :init)))
  (if (not (boundp '*staro*))
      (setq *staro* (instance staro-robot :init)))
  (unless (boundp '*irtviewer*) (make-irtviewer :width 320 :height 320) (send *irtviewer* :change-background #f(0.3 0.5 0.7)))
  (objects (list *staro*))
  )
