(load "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknts.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp3hand-utils.l")

(unless (assoc :init-org (send hrp2jsknts-robot :methods))
  (rplaca (assoc :init (send hrp2jsknts-robot :methods)) :init-org))
(eval
 `(defmethod hrp2jsknts-robot
    ,@(get-hrp2-with-hand-class-methods)))

(defmethod hrp2jsknts-robot
  (:init-ending
   ()
   (prog1
       (send-super :init-ending)
     (send self :define-min-max-table)
     ))
  (:inverse-kinematics
   (target-coords
    &rest args
    &key (additional-weight-list (mapcar #'(lambda (l) (list (send self l :toe-p :child-link) 0)) '(:rleg :lleg)))
    &allow-other-keys)
   (send-super* :inverse-kinematics target-coords :additional-weight-list additional-weight-list args))
  (:fullbody-inverse-kinematics
   (target-coords
    &rest args
    &key (additional-weight-list (mapcar #'(lambda (l) (list (send self l :toe-p :child-link) 0)) '(:rleg :lleg)))
    &allow-other-keys)
   (send-super* :fullbody-inverse-kinematics target-coords :additional-weight-list additional-weight-list args))
  )
