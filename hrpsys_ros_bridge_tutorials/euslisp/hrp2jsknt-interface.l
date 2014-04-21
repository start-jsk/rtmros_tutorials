(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :hrp2jsknt "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknt.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-utils.l"))
  (require :hrp2jsknt-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-utils.l"))
(if (ros::resolve-ros-path "package://jsk_hrp2_ros_bridge")
    (ros::load-ros-manifest "jsk_hrp2_ros_bridge"))

(defclass hrp2jsknt-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod hrp2jsknt-interface
  (:init
   (&rest args)
   (prog1
       (send-super :init :robot hrp2jsknt-robot)
     (send self :set-st-param
           :k-tpcc-p #f(2.5 2.5)
           :k-tpcc-x #f(5 5)
           :k-brot-p #f(0 0)
           :k-brot-tc #f(0.1 0.1))
     (send self :set-auto-balancer-param
           :default-zmp-offsets (list (float-vector 20 -15 0) (float-vector 20 15 0)))
     ))
  (:define-all-ROSBridge-srv-methods
   (&key (debug-view nil) (ros-pkg-name "hrpsys_ros_bridge"))
   (send-super :define-all-ROSBridge-srv-methods)
   (if (ros::resolve-ros-path "package://jsk_hrp2_ros_bridge")
       (send-super :define-all-ROSBridge-srv-methods :ros-pkg-name "jsk_hrp2_ros_bridge"))
   )
  (:hand-controlmode-vector
   (cmode)
   (send self :hrp3handcontrollerservice_setcontrolmodevector :i_cmode cmode))
  (:hand-angle-vector
   (av &optional (tm 1000))
   (send self :hrp3handcontrollerservice_setjointangles :jvs av :tm (/ tm 1000.0)))
  ;;(:hand-wait-interpolation-raw
  (:hand-wait-interpolation
   ()
   (send self :hrp3handcontrollerservice_waitinterpolation)
   )
  )

(defun hrp2jsknt-init ()
  (if (not (boundp '*ri*))
      (setq *ri* (instance hrp2jsknt-interface :init)))
  (if (not (boundp '*hrp2jsknt*))
      (setq *hrp2jsknt* (instance hrp2jsknt-robot :init))))
