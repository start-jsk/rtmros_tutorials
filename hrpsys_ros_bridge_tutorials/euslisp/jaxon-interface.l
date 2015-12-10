(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :jaxon "package://hrpsys_ros_bridge_tutorials/models/jaxon.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-utils.l"))
  (require :jaxon-utils "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-utils.l"))
(require :jaxon-common-interface "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-common-interface.l")

(defclass jaxon-interface
  :super jaxon-common-interface
  :slots ())
(defmethod jaxon-interface
  (:init (&rest args &key ((:controller-timeout ct) nil))
         (send-super* :init :robot jaxon-robot :controller-timeout ct args)))

(when (probe-file (ros::resolve-ros-path "package://thk_hand_controller/euslisp/thkhand-simple-controller.l"))
  (load "package://thk_hand_controller/euslisp/thkhand-simple-controller.l")
  (unless (assoc :init-org (send jaxon-interface :methods))
    (rplaca (assoc :init (send jaxon-interface :methods)) :init-org))
  (defmethod jaxon-interface
    (:init
     (&rest args &key ((:controller-timeout ct) nil))
     (prog1
         (send* self :init-org :controller-timeout ct args)
       (print ";; Add hands")
       (send self :put :hand-controller (instance thkhand-simple-controller :init))
       (setq hand-enable t)
       )
     )
    )
  )

(defun jaxon-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* jaxon-interface :init args)))
  (if (not (boundp '*jaxon*))
      (setq *jaxon* (instance jaxon-robot :init))))
