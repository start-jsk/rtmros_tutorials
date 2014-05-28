(load "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknt.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp3hand-utils.l")

(unless (assoc :init-org (send hrp2jsknt-robot :methods))
  (rplaca (assoc :init (send hrp2jsknt-robot :methods)) :init-org))
(eval
 `(defmethod hrp2jsknt-robot
    ,@(get-hrp2-with-hand-class-methods)))
