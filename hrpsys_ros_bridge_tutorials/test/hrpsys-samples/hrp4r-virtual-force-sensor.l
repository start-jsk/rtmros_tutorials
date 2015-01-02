#!/usr/bin/env roseus

;; $ rtmlaunch hrpsys_ros_bridge_tutorials hrp4r.launch
;; $ roseus hrp4r-virtual-force-sensor.l "(hrp4r-virtual-force-sensor-demo)"

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp4r-interface.l")

(defun hrp4r-virtual-force-sensor-init ()
  (hrp4r-init)
  (send *hrp4r* :reset-pose)
  (send *ri* :angle-vector (send *hrp4r* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  t)

(defun hrp4r-virtual-force-sensor-demo0 ()
  "1. Test impedance controller"
  (send *ri* :start-impedance :rarm)
  (send *ri* :set-ref-force (float-vector 10 10 -10) 2000 :rarm)
  (send *rI* :wait-interpolation-seq)
  (send *ri* :set-ref-force (float-vector 0 0 0) 2000 :rarm)
  (send *rI* :wait-interpolation-seq)
  (send *ri* :stop-impedance :rarm)
  t)


(defun hrp4r-virtual-force-sensor-demo ()
  (hrp4r-virtual-force-sensor-init)
  (hrp4r-virtual-force-sensor-demo0)
  )

(warn ";; (hrp4r-virtual-force-sensor-demo)~%")