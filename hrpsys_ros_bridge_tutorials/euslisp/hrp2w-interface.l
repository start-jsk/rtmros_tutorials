(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :hrp2w "package://hrpsys_ros_bridge_tutorials/models/hrp2w.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-utils.l"))
  (require :hrp2w-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-utils.l"))

(defclass hrp2w-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod hrp2w-interface
  (:init (&rest args)
         (send-super* :init :robot hrp2w-robot args)))

;; methods for grasp controller
(defmethod hrp2w-interface
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
     )
   )
  )

(defun hrp2w-init (&key (sim nil))
  (if (not (boundp '*ri*))
      (setq *ri* (instance hrp2w-interface :init)))
  (if (not (boundp '*hrp2w*))
      (setq *hrp2w* (instance hrp2w-robot :init)))
  (unless sim
    (if (not (boundp '*vmax*))
	(progn
	  (require :vmax "package://vmax_controller/euslisp/vmax-interface.l")
	  (setq *vmax* (instance vmax-interface :init)))))
  (setq ((send *hrp2w* :head :neck-p) . max-angle) 44)
  )

