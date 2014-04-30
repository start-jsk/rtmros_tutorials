(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :hrp2jsknts "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknts.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-utils.l"))
  (require :hrp2jsknts-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-utils.l"))
(if (ros::resolve-ros-path "package://jsk_hrp2_ros_bridge")
    (ros::load-ros-manifest "jsk_hrp2_ros_bridge"))

(defclass hrp2jsknts-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod hrp2jsknts-interface
  (:init (&rest args)
         (send-super :init :robot hrp2jsknts-robot))
  (:define-all-ROSBridge-srv-methods
   (&key (debug-view nil) (ros-pkg-name "hrpsys_ros_bridge"))
   (send-super :define-all-ROSBridge-srv-methods)
   (if (ros::resolve-ros-path "package://jsk_hrp2_ros_bridge")
       (send-super :define-all-ROSBridge-srv-methods :ros-pkg-name "jsk_hrp2_ros_bridge"))
   )
  (:hand-controlmode-vector
   (cmode)
   (if (find-method self :hrp3handcontrollerservice_setcontrolmodevector)
       (send self :hrp3handcontrollerservice_setcontrolmodevector :i_cmode cmode)
     (warn ";; :hrp3handcontrollerservice_setcontrolmodevector is not implemented!~%"))
   )
  (:hand-angle-vector
   (av &optional (tm 1000))
   (if (find-method self :hrp3handcontrollerservice_setjointangles)
       (send self :hrp3handcontrollerservice_setjointangles :jvs av :tm (/ tm 1000.0))
     (warn ";; :hrp3handcontrollerservice_setjointangles is not implemented!~%"))
   )
  ;;(:hand-wait-interpolation-raw
  (:hand-wait-interpolation
   ()
   (if (find-method self :hrp3handcontrollerservice_waitinterpolation)
       (send self :hrp3handcontrollerservice_waitinterpolation)
     (warn ";; :hrp3handcontrollerservice_waitinterpolation is not implemented!~%"))
   )
)

(defun hrp2jsknts-init ()
  (if (not (boundp '*ri*))
      (setq *ri* (instance hrp2jsknts-interface :init)))
  (if (not (boundp '*hrp2jsknts*))
      (setq *hrp2jsknts* (instance hrp2jsknts-robot :init))))
