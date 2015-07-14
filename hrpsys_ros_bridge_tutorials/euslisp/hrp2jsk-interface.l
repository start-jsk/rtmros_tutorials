(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :hrp2jsk "package://hrpsys_ros_bridge_tutorials/models/hrp2jsk.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-utils.l"))
  (require :hrp2jsk-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-utils.l"))

(defclass hrp2jsk-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod hrp2jsk-interface
  (:init (&rest args)
         (send-super* :init :robot hrp2jsk-robot args))
  (:start-grasp
   (limb &key (target-error 0.15) (gain-percentage 5) &rest args)
   (case limb
     (:larm
      (send* self :set-servo-gain-percentage
	     "LARM_JOINT7" gain-percentage args)
      (send* self :graspcontrollerservice_startgrasp
	     :name "LHAND"
	     :target_error target-error args))
     (:rarm
      (send* self :set-servo-gain-percentage
	     "RARM_JOINT7" gain-percentage args)
      (send* self :graspcontrollerservice_startgrasp
	     :name "RHAND"
	     :target_error target-error args))
     )
   )
  (:stop-grasp
   (limb &rest args)
   (case limb
     (:larm
      (send* self :set-servo-gain-percentage "LARM_JOINT7" 100 args)
      (send* self :graspcontrollerservice_stopgrasp
	     :name "LHAND"
	     args))
     (:rarm
      (send* self :set-servo-gain-percentage "RARM_JOINT7" 100 args)
      (send* self :graspcontrollerservice_stopgrasp
	     :name "RHAND"
	     args))
     )))

(defun hrp2jsk-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* hrp2jsk-interface :init args)))
  (if (not (boundp '*hrp2jsk*))
      (setq *hrp2jsk* (instance hrp2jsk-robot :init))))
