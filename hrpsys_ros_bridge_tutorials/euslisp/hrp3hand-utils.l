(load "package://hrpsys_ros_bridge_tutorials/models/hrp3hand_l.l")
(load "package://hrpsys_ros_bridge_tutorials/models/hrp3hand_r.l")

(defun get-hrp3hand-class-methods ()
  '(
  ;; poses
  (:open-pose ()
   (send self :angle-vector (float-vector  0 60 -10 30  0  0)))
  (:preclose-pose ()
   (send self :angle-vector (float-vector  40  60 -10  35  10 90)))
  (:close-pose ()
   (send self :angle-vector (float-vector  65   0  30  50  90 90)))
  (:reset-pose ()
   (send self :angle-vector (float-vector  0 0 0 0 0 0)))
  ;;
  (:standard-pose () ;; PoS
   (send self :angle-vector (float-vector  20 90 0 10 -20 -20)))
  (:hook-pose () ;; PoH
   (send self :angle-vector (float-vector  90 90 0 10 -20 -20)))
  (:index-pose () ;; PoI
   (send self :angle-vector (float-vector  60 90 0 70 -20 -20)))
  (:extension-pose () ;; PoE
   (send self :angle-vector (float-vector  90 30 0 10 -20 -20)))
  (:distal-pose () ;; PoD
   (send self :angle-vector (float-vector  50 60 -20 10 20 40)))
  (:hook-pose2 ()
    (send self :angle-vector (float-vector  90 70 0 10 -20 -40)))
  (:distal-pose2 ()
    (send self :angle-vector (float-vector  90 90 -20 10 20 60)))
    )
  )

(eval
 `(defmethod hrp3hand_l-robot
    ,@(get-hrp3hand-class-methods)))

(eval
 `(defmethod hrp3hand_r-robot
    ,@(get-hrp3hand-class-methods)))
