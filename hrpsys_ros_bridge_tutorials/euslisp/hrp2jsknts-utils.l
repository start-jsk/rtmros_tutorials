(load "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknts.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp3hand-utils.l")

(unless (assoc :init-org (send hrp2jsknts-robot :methods))
  (rplaca (assoc :init (send hrp2jsknts-robot :methods)) :init-org))
(eval
 `(defmethod hrp2jsknts-robot
    ,@(get-hrp2-with-hand-class-methods)))

