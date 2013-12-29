(load "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknt.l")
(load "package://hrpsys_ros_bridge_tutorials/models/hrp3hand_l.l")
(load "package://hrpsys_ros_bridge_tutorials/models/hrp3hand_r.l")

(unless (assoc :init-org (send hrp2jsknt-robot :methods))
  (rplaca (assoc :init (send hrp2jsknt-robot :methods)) :init-org))
(defmethod hrp2jsknt-robot
  (:init
   (&rest args)
   (prog1
       (send* self :init-org args)
     (send self :put :lhand-model (instance hrp3hand_l-robot :init))
     (send self :put :rhand-model (instance hrp3hand_r-robot :init))
     (mapcar #'(lambda (l lm)
                 (send (send self :get lm) :newcoords
                       (send (send self l :end-coords :parent) :copy-worldcoords))
                 (send (send self l :end-coords :parent) :assoc (send self :get lm)))
             '(:larm :rarm) '(:lhand-model :rhand-model))
     (setq bodies (append bodies
                          (send (send self :get :lhand-model) :bodies)
                          (send (send self :get :rhand-model) :bodies)))
     ))
  (:hand (arm &rest args)
   (let (hr ret)
     (case arm
       (:larm
        (setq hr (send self :get :lhand-model))
        (setq ret (forward-message-to hr args)))
       (:rarm
        (setq hr (send self :get :rhand-model))
        (setq ret (forward-message-to hr args)))
       (:both
        (setq hr (send self :get :lhand-model))
        (push (forward-message-to hr args) ret)
        (setq hr (send self :get :rhand-model))
        (push (forward-message-to hr args) ret))
       )
     ret))
  )
