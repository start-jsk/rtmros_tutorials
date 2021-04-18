(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :hrp2jsknts "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknts_with_3hand.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-utils.l"))
  (require :hrp2jsknts-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-utils.l"))
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2-common-interface.l")

(defclass hrp2jsknts-interface
  :super hrp2-common-interface
  :slots ())
(defmethod hrp2jsknts-interface
  (:init (&rest args)
    (prog1
        (send-super* :init :robot hrp2jsknts-robot args)
      ;; add controller
      (dolist (limb '(:rarm :larm :rleg :lleg :head :torso :rhand :lhand))
        (send self :def-limb-controller-method limb)
        (send self :add-controller (read-from-string (format nil "~A-controller" limb)) :joint-enable-check t :create-actions t))))
  )

(defun hrp2jsknts-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* hrp2jsknts-interface :init args)))
  (if (not (boundp '*hrp2jsknts*))
      (setq *hrp2jsknts* (instance hrp2jsknts-robot :init))))
