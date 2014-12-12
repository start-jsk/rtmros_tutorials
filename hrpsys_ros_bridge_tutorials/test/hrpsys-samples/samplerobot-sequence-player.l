#!/usr/bin/env roseus

;; $ rtmlaunch hrpsys_ros_bridge_tutorials samplerobot.launch
;; $ roseus samplerobot-sequence-player.l "(samplerobot-sequence-player-demo)"

(load "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")

(defun samplerobot-sequence-player-init ()
  (samplerobot-init)
  t)

(defun samplerobot-sequence-player-demo0 ()
  "1. :angle-vector (setJointAngles) and :wait-interpolation (waitInterpolation)"
  (send *ri* :angle-vector (send *sr* :angle-vector) 700)
  (warn ";; before wait interpolation")
  ;;(send *ri* :wait-interpolation)
  (unix:sleep 2)
  (warn ";; after wait interpolation")
  (warn ";; seq diff ~A~%" (distance (send *ri* :state :potentio-vector) (send *sr* :angle-vector)))
  (eps= (distance (send *ri* :state :potentio-vector) (send *sr* :angle-vector)) 0.0)
  nil)

(defun samplerobot-sequence-player-demo1 ()
  "2. :set-interpolation-mode"
  (send *ri* :set-interpolation-mode :linear)
  (send *sr* :reset-manip-pose)
  (send *ri* :angle-vector (send *sr* :angle-vector) 500)
  ;;(send *ri* :wait-interpolation)
  (unix:sleep 2)
  (send *ri* :set-interpolation-mode :hoffarbib)
  (send *sr* :reset-pose)
  (send *ri* :angle-vector (send *sr* :angle-vector) 500)
  ;;(send *ri* :wait-interpolation)
  (unix:sleep 2)
  t)

(defun samplerobot-sequence-player-demo ()
  (samplerobot-sequence-player-init)
  (samplerobot-sequence-player-demo0)
  (samplerobot-sequence-player-demo1)
  )

(warn ";; (samplerobot-sequence-player-demo)~%")
