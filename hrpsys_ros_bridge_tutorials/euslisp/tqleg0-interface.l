(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :tqleg0 "package://hrpsys_ros_bridge_tutorials/models/tqleg0.l")
;;(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/tqleg0-utils.l"))
;;  (require :tqleg0-utils "package://hrpsys_ros_bridge_tutorials/euslisp/tqleg0-utils.l"))

(defclass tqleg0-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod tqleg0-interface
  (:init (&rest args &key ((:controller-timeout ct) nil))
         (send-super* :init :robot tqleg0-robot :controller-timeout ct args)))

;;(unless (assoc :init-org (send jaxon-interface :methods))
;;  (rplaca (assoc :init (send jaxon-interface :methods)) :init-org))
;; (defmethod tqleg0-interface
;;   (:get-joint-angle
;;    (&rest args)
;;    (send* (send self :get :hand-controller) :get-joint-angle args))
;;   (:get-joint-velocity
;;    (&rest args)
;;    (send* (send self :get :hand-controller) :get-joint-velocity args))
;;   (:get-joint-effort
;;    (&rest args)
;;    (send* (send self :get :hand-controller) :get-joint-effort args))
;;   ;; controller group configuration
;;   (:lleg-controller
;;    ()
;;    (list
;;     (list
;;      (cons :group-name "lleg")
;;      (cons :controller-action "lleg_controller/joint_trajectory_action")
;;      (cons :controller-state "lleg_controller/state")
;;      (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
;;      (cons :joint-names (list "LLEG_JOINT0" "LLEG_JOINT1" "LLEG_JOINT2"
;; 			      "LLEG_JOINT3" "LLEG_JOINT4" "LLEG_JOINT5"))))
;;    )
;;   (:rleg-controller
;;    ()
;;    (list
;;     (list
;;      (cons :group-name "rleg")
;;      (cons :controller-action "rleg_controller/joint_trajectory_action")
;;      (cons :controller-state "rleg_controller/state")
;;      (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
;;      (cons :joint-names (list "RLEG_JOINT0" "RLEG_JOINT1" "RLEG_JOINT2"
;; 			      "RLEG_JOINT3" "RLEG_JOINT4" "RLEG_JOINT5"))))
;;    )
;;   )

(defun tqleg0-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* tqleg0-interface :init args)))
  (if (not (boundp '*tqleg0*))
      (setq *tqleg0* (instance tqleg0-robot :init)))
  (if (not (boundp '*ystleg*))
      (setq *ystleg* *tqleg0*)))
(defun ystleg-init () (tqleg0-init))
