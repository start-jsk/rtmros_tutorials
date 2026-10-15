(load "package://pr2eus/robot-interface.l")
(require :nextage "package://nextage_tutorials/nextage.l")

(defclass nextage-interface
  :super robot-interface
  :slots ())
(defmethod nextage-interface
  (:init (&rest args)
         (send-super :init :robot nextageopen-robot))
  (:default-controller ()
   (append
    (send self :larm-controller)
    (send self :rarm-controller)
    (send self :torso-controller)
    (send self :head-controller)))
  (:larm-controller ()
   (list
    (list
     (cons :controller-action "/larm_controller/follow_joint_trajectory_action")
     (cons :controller-state "/larm_controller/fstate")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (send-all (send robot :larm :joint-list) :name)))))
  (:rarm-controller ()
   (list
    (list
     (cons :controller-action "/rarm_controller/follow_joint_trajectory_action")
     (cons :controller-state "/rarm_controller/fstate")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (send-all (send robot :rarm :joint-list) :name)))))
  (:torso-controller ()
   (list
    (list
     (cons :controller-action "/torso_controller/follow_joint_trajectory_action")
     (cons :controller-state "/torso_controller/fstate")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (send-all (send robot :torso :joint-list) :name)))))
  (:head-controller ()
   (list
    (list
     (cons :controller-action "/head_controller/follow_joint_trajectory_action")
     (cons :controller-state "/head_controller/fstate")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (send-all (send robot :head :joint-list) :name)))))
  )

(defun nextage-init ()
  (if (not (boundp '*ri*))
      (setq *ri* (instance nextage-interface :init)))
  (if (not (boundp '*nextage*))
      (setq *nextage* (instance nextageopen-robot :init))))
