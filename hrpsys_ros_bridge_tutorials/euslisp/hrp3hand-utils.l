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
  (:grasp-pose ()
    (send self :angle-vector (float-vector 77.9709 -11.4732 8.28742 -16.3569 106.185 86.0974)))
    )
  )

(defun get-hrp2-with-hand-class-methods ()
 '(
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
       ((:both :arms)
        (setq hr (send self :get :lhand-model))
        (push (forward-message-to hr args) ret)
        (setq hr (send self :get :rhand-model))
        (push (forward-message-to hr args) ret))
       )
     ret))
  (:hand-angle-vector
   (&optional (av))
   (when av
     (send self :hand :rarm :angle-vector (subseq av 0 6))
     (send self :hand :larm :angle-vector (subseq av 6))
     )
   (concatenate float-vector
                (send self :hand :rarm :angle-vector)
                (send self :hand :larm :angle-vector))
   )
  )
 )

(eval
 `(defmethod hrp3hand_l-robot
    ,@(get-hrp3hand-class-methods)))

(eval
 `(defmethod hrp3hand_r-robot
    ,@(get-hrp3hand-class-methods)))
