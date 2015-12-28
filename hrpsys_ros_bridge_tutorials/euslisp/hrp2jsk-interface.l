(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :hrp2jsk "package://hrpsys_ros_bridge_tutorials/models/hrp2jsk.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-utils.l"))
  (require :hrp2jsk-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-utils.l"))

(defclass hrp2jsk-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod hrp2jsk-interface
  (:init (&rest args)
         (prog1
             (send-super* :init :robot hrp2jsk-robot args)
           ;; add controller
           (dolist (limb '(:rarm :larm :rleg :lleg :head :torso))
             (send self :def-limb-controller-method limb)
             (send self :add-controller (read-from-string (format nil "~A-controller" limb)) :joint-enable-check t :create-actions t))))
  (:get-hand-config-list () (list (list :larm "LHAND" "LARM_JOINT7") (list :rarm "RHAND" "RARM_JOINT7")))
  (:start-grasp
   (limb &key (target-error 0.15) (gain-percentage 5))
   (send-super :start-grasp limb :target-error target-error :gain-percentage gain-percentage)
   )
  )

(defun hrp2jsk-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* hrp2jsk-interface :init args)))
  (if (not (boundp '*hrp2jsk*))
      (setq *hrp2jsk* (instance hrp2jsk-robot :init))))
