(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")

(defclass jaxon-common-interface
  :super rtm-ros-robot-interface
  :slots (hand-enable)
  )

(defmethod jaxon-common-interface
  (:init
   (&rest args &key ((:robot rb)) ((:controller-timeout ct) nil))
   (prog1
       (send-super* :init :robot rb :controller-timeout ct args)
     ;; add controller
     (print ";; Add controllers")
     (send self :add-controller :larm-controller :joint-enable-check t :create-actions t)
     (send self :add-controller :rarm-controller :joint-enable-check t :create-actions t)
     (send self :add-controller :lleg-controller :joint-enable-check t :create-actions t)
     (send self :add-controller :rleg-controller :joint-enable-check t :create-actions t)
     ))
  (:move-gripper
   (&rest args)
   (if hand-enable
       (send* (send self :get :hand-controller) :move-gripper args)
     (warn ";; can not use hand~%")))
  (:start-grasp
   (&rest args)
   (if hand-enable
       (send* (send self :get :hand-controller) :start-grasp args)
     (warn ";; can not use hand~%")))
  (:stop-grasp
   (&rest args)
   (if hand-enable
       (send* (send self :get :hand-controller) :stop-grasp args)
     (warn ";; can not use hand~%")))
  (:hand-reset
   (&rest args)
   (if hand-enable
       (send* (send self :get :hand-controller) :reset args)
     (warn ";; can not use hand~%")))
  ;; (:hand-open
  ;;  (&rest args)
  ;;  (send* (send self :get :hand-controller) :open args))
  ;; (:hand-close
  ;;  (&rest args)
  ;;  (send* (send self :get :hand-controller) :close args))
  (:hand-stop
   (&rest args)
   (if hand-enable
       (send* (send self :get :hand-controller) :stop args)
     (warn ";; can not use hand~%")))
  (:hand-resume
   (&rest args)
   (if hand-enable
       (send* (send self :get :hand-controller) :resume args)
     (warn ";; can not use hand~%")))
  (:get-joint-angle
   (&rest args)
   (warning-message 1 ";;!!!!!!!!~%;; (send *ri* :get-joint-angle) METHOD IS DEPRECATED!!~%;; Please use (send *ri* :state :gripper :arms :angle-vector)~%;;!!!!!!!!~%")
   (if hand-enable
       (send* (send self :get :hand-controller) :get-joint-angle args)
     (warn ";; can not use hand~%")))
  (:get-joint-velocity
   (&rest args)
   (warning-message 1 ";;!!!!!!!!~%;; (send *ri* :get-joint-velocity) METHOD IS DEPRECATED!!~%;; Please use (send *ri* :state :gripper :arms :velocity-vector)~%;;!!!!!!!!~%")
   (if hand-enable
       (send* (send self :get :hand-controller) :get-joint-velocity args)
     (warn ";; can not use hand~%")))
  (:get-joint-effort
   (&rest args)
   (warning-message 1 ";;!!!!!!!!~%;; (send *ri* :get-joint-effort) METHOD IS DEPRECATED!!~%;; Please use (send *ri* :state :gripper :arms :effort-vector)~%;;!!!!!!!!~%")
   (if hand-enable
       (send* (send self :get :hand-controller) :get-joint-effort args)
     (warn ";; can not use hand~%")))
  (:gripper
   (arm method)
   (if (memq arm '(:rarm :larm :arms))
       (case
        method
        (:angle-vector
         (if hand-enable
             (send* (send self :get :hand-controller) :get-joint-angle (if (eq arm :arms) arm (list arm)))
           (warn ";; can not use hand~%")))
        (:velocity-vector
         (if hand-enable
             (send* (send self :get :hand-controller) :get-joint-velocity (if (eq arm :arms) arm (list arm)))
           (warn ";; can not use hand~%")))
        (:torque-vector
         (if hand-enable
             (send* (send self :get :hand-controller) :get-joint-effort (if (eq arm :arms) arm (list arm)))
           (warn ";; can not use hand~%")))
        (t )
        )
     (error ";; no such arm in :gripper ~A~%" arm))
   )
  (:state (&rest args) ;; overwrite for gripper
   (case
    (car args)
    (:gripper
     (send* self :gripper (cdr args)))
    (t
     (send-super* :state args))))
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
