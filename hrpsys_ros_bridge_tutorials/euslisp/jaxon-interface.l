(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :jaxon "package://hrpsys_ros_bridge_tutorials/models/jaxon.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-utils.l"))
  (require :jaxon-utils "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-utils.l"))

(defclass jaxon-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod jaxon-interface
  (:init (&rest args)
         (send-super* :init :robot jaxon-robot args)))

(when (probe-file (ros::resolve-ros-path "package://thk_hand_controller/euslisp/thkhand-simple-controller.l"))
  (load "package://thk_hand_controller/euslisp/thkhand-simple-controller.l")
  (unless (assoc :init-org (send jaxon-interface :methods))
    (rplaca (assoc :init (send jaxon-interface :methods)) :init-org))
  (defmethod jaxon-interface
    (:init
     (&rest args)
     (prog1
         (send* self :init-org args)
       (print 1)
       (send self :put :hand-controller (instance thkhand-simple-controller :init))
       ))
    (:move-gripper
     (&rest args)
     (send* (send self :get :hand-controller) :move-gripper args))
    (:start-grasp
     (&rest args)
     (send* (send self :get :hand-controller) :start-grasp args))
    (:stop-grasp
     (&rest args)
     (send* (send self :get :hand-controller) :stop-grasp args))
    (:hand-reset
     (&rest args)
     (send* (send self :get :hand-controller) :reset args))
    ;; (:hand-open
    ;;  (&rest args)
    ;;  (send* (send self :get :hand-controller) :open args))
    ;; (:hand-close
    ;;  (&rest args)
    ;;  (send* (send self :get :hand-controller) :close args))
    (:hand-stop
     (&rest args)
     (send* (send self :get :hand-controller) :stop args))
    (:hand-resume
     (&rest args)
     (send* (send self :get :hand-controller) :resume args))
    (:get-joint-angle
     (&rest args)
     (send* (send self :get :hand-controller) :get-joint-angle args))
    (:get-joint-velocity
     (&rest args)
     (send* (send self :get :hand-controller) :get-joint-velocity args))
    (:get-joint-effort
     (&rest args)
     (send* (send self :get :hand-controller) :get-joint-effort args))
    )
  )

(defun jaxon-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* jaxon-interface :init args)))
  (if (not (boundp '*jaxon*))
      (setq *jaxon* (instance jaxon-robot :init))))
