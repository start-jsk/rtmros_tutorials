(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :tablis "package://hrpsys_ros_bridge_tutorials/models/tablis.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/tablis-utils.l"))
  (require :tablis-utils "package://hrpsys_ros_bridge_tutorials/euslisp/tablis-utils.l"))

(defclass tablis-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod tablis-interface
  (:init
   (&rest args &key ((:controller-timeout ct) nil))
   (prog1
       (send-super* :init :robot tablis-robot :controller-timeout ct args)
      ;; add controller
      (dolist (limb '(:rleg :lleg))
        (send self :def-limb-controller-method limb)
        (send self :add-controller (read-from-string (format nil "~A-controller" limb)) :joint-enable-check t :create-actions t)))
   )
  )

(defun tablis-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* tablis-interface :init args)))
  (if (not (boundp '*tablis*))
      (setq *tablis* (instance tablis-robot :init))))
