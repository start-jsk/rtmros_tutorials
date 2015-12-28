(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :chidori "package://hrpsys_ros_bridge_tutorials/models/chidori.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/chidori-utils.l"))
  (require :chidori-utils "package://hrpsys_ros_bridge_tutorials/euslisp/chidori-utils.l"))

(defclass chidori-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod chidori-interface
  (:init
   (&rest args &key ((:controller-timeout ct) nil))
   (prog1
       (send-super* :init :robot chidori-robot :controller-timeout ct args)
      ;; add controller
      (dolist (limb '(:rleg :lleg))
        (send self :def-limb-controller-method limb)
        (send self :add-controller (read-from-string (format nil "~A-controller" limb)) :joint-enable-check t :create-actions t)))
   )
  )

(defun chidori-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* chidori-interface :init args)))
  (if (not (boundp '*chidori*))
      (setq *chidori* (instance chidori-robot :init))))
