(load "package://pr2eus/robot-interface.l")
(require :nextage "package://nextage_tutorials/jsk-nextage.l")
(load "package://nextage_tutorials/euslisp/jsk-nextage-utils.l")

;; If dynamixel hand is available:
(when (ros::resolve-ros-path "package://dynamixel_detachable_hand/euslisp/lhand-interface.l")
  (load "package://dynamixel_detachable_hand/euslisp/lhand-interface.l")
  (load "package://dynamixel_detachable_hand/euslisp/rhand-interface.l"))

(defclass nextage-interface
  :super robot-interface
  :slots (on-gazebo-ros-control))

(defmethod nextage-interface
  (:init (&rest args &key (type nil) &allow-other-keys)
    (setq on-gazebo-ros-control
          (and (ros::get-param "/gazebo/time_step" nil)
               (ros::get-param "/torso_controller/type" nil)))
    (when on-gazebo-ros-control
      (ros::ros-warn "Found Gazebo/ros_control environment"))
    (send-super* :init :robot nextageopen-robot :type
                 (cond (type type)
                       (on-gazebo-ros-control :gazebo-ros-controller)
                       (t :default-controller))
                 args))
  (:gazebo-ros-controller ()
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
  (:lhand-init ()
    (unless (boundp '*lhand*)
      (setq *lhand* (instance lhand-interface :init)))
    *lhand*)
  (:rhand-init ()
    (unless (boundp '*rhand*)
      (setq *rhand* (instance rhand-interface :init)))
    *rhand*)
  (:start-grasp (&optional (arm :arms) &rest args)
    (case arm
      (:larm (send* (send self :lhand-init) :close args))
      (:rarm (send* (send self :rhand-init) :close args))
      (:arms (send* (send self :lhand-init) :close args)
             (send* (send self :rhand-init) :close args))))
  (:stop-grasp (&optional (arm :arms) &rest args)
    (case arm
      (:larm (send* (send self :lhand-init) :open args))
      (:rarm (send* (send self :rhand-init) :open args))
      (:arms (send* (send self :lhand-init) :open args)
             (send* (send self :rhand-init) :open args))))
  (:open-forceps (&optional (arm :arms) &rest args)
    (case arm
      (:larm (send* (send self :lhand-init) :open-forceps args))
      (:rarm (send* (send self :rhand-init) :open-forceps args))
      (:arms (send* (send self :lhand-init) :open-forceps args)
             (send* (send self :rhand-init) :open-forceps args))))
  (:close-forceps (&optional (arm :arms) &rest args)
    (case arm
      (:larm (send* (send self :lhand-init) :close-forceps args))
      (:rarm (send* (send self :rhand-init) :close-forceps args))
      (:arms (send* (send self :lhand-init) :close-forceps args)
             (send* (send self :rhand-init) :close-forceps args))))
  (:open-holder (&optional (arm :arms) &rest args)
    (case arm
      (:larm (send* (send self :lhand-init) :open-holder args))
      (:rarm (send* (send self :rhand-init) :open-holder args))
      (:arms (send* (send self :lhand-init) :open-holder args)
             (send* (send self :rhand-init) :open-holder args))))
  (:close-holder (&optional (arm :arms) &rest args)
    (case arm
      (:larm (send* (send self :lhand-init) :close-holder args))
      (:rarm (send* (send self :rhand-init) :close-holder args))
      (:arms (send* (send self :lhand-init) :close-holder args)
             (send* (send self :rhand-init) :close-holder args))))
  (:open-scissors (&optional (arm :arms) &rest args)
    (case arm
      (:larm (send* (send self :lhand-init) :open-scissors args))
      (:rarm (send* (send self :rhand-init) :open-scissors args))
      (:arms (send* (send self :lhand-init) :open-scissors args)
             (send* (send self :rhand-init) :open-scissors args))))
  (:close-scissors (&optional (arm :arms) &rest args)
    (case arm
      (:larm (send* (send self :lhand-init) :close-scissors args))
      (:rarm (send* (send self :rhand-init) :close-scissors args))
      (:arms (send* (send self :lhand-init) :close-scissors args)
             (send* (send self :rhand-init) :close-scissors args))))
  )

(defun nextage-init ()
  (if (not (boundp '*ri*))
      (setq *ri* (instance nextage-interface :init)))
  (if (not (boundp '*nextage*))
      (setq *nextage* (instance nextageopen-robot :init))))
