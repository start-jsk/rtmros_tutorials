(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :hrp2jsknt "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknt_with_3hand.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-utils.l"))
  (require :hrp2jsknt-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-utils.l"))
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2-common-interface.l")

(defclass hrp2jsknt-interface
  :super hrp2-common-interface
  :slots ())
(defmethod hrp2jsknt-interface
  (:init (&rest args)
    (prog1
        (send-super* :init :robot hrp2jsknt-robot args)
      ;; add controller
      (dolist (limb '(:rarm :larm :rleg :lleg :head :torso :rhand :lhand))
        (send self :def-limb-controller-method limb)
        (send self :add-controller (read-from-string (format nil "~A-controller" limb)) :joint-enable-check t :create-actions t))))
  )

(defun hrp2jsknt-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* hrp2jsknt-interface :init args)))
  (if (not (boundp '*hrp2jsknt*))
      (setq *hrp2jsknt* (instance hrp2jsknt-robot :init))))
