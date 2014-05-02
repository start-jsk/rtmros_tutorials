(defclass hrp2-common-interface
  :super rtm-ros-robot-interface
  :slots ()
  )

(defmethod hrp2-common-interface
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
  (:hand-servo-on
   ()
   (if (find-method self :hrp3handcontrollerservice_handservoon)
       (send self :hrp3handcontrollerservice_handservoon)
     (warn ";; :hrp3handcontrollerservice_handservoon is not implemented!~%"))
   )
  (:hand-servo-off
   ()
   (if (find-method self :hrp3handcontrollerservice_handservooff)
       (send self :hrp3handcontrollerservice_handservooff)
     (warn ";; :hrp3handcontrollerservice_handservooff is not implemented!~%"))
   )
  (:hand-joint-calib
   ()
   (if (find-method self :hrp3handcontrollerservice_handjointcalib)
       (send self :hrp3handcontrollerservice_handjointcalib)
     (warn ";; :hrp3handcontrollerservice_handjointcalib is not implemented!~%"))
   )
  )