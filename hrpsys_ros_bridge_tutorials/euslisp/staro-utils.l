(require :staro "package://hrpsys_ros_bridge_tutorials/models/staro.l")

;; for RobotiqHand
(if (ros::rospack-find "robotiq_s_model_control")
    (progn (load "package://staro_ros_bridge/euslisp/robotiq-simple-controller.l")
           (unless (assoc :init-org (send STARO-robot :methods))
             (rplaca (assoc :init (send STARO-robot :methods)) :init-org))
           (defmethod STARO-robot
             (:init
              (&rest args)
              (prog1
                  (send* self :init-org args)
                (send self :put :lhand-model (instance robotiq-controller :init "10.68.0.21" :output "leftSModelRobotOutput" :input "leftSModelRobotInput"))
                (send self :put :rhand-model (instance robotiq-controller :init "10.68.0.22" :output "rightSModelRobotOutput" :input "rightSModelRobotInput"))
                ;; RobotiqHand model is already in wrl of staro
                ;; (mapcar #'(lambda (l lm)
                ;;             (send (send self :get lm) :newcoords
                ;;                   (send (send self l :end-coords :parent) :copy-worldcoords))
                ;;             (send (send self l :end-coords :parent) :assoc (send self :get lm)))
                ;;         '(:larm :rarm) '(:lhand-model :rhand-model))
                ;; (setq bodies (append bodies
                ;;                      (send (send self :get :lhand-model) :bodies)
                ;;                      (send (send self :get :rhand-model) :bodies)))
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
             )
           )
  (warn  "~c[33m [WARN] robotiq_s_model_control for RobotiqHand cannnot be found. skip definition for RobotiqHand methods. ~c[m~%" #x1b #x1b) ;; ~c: escape sequence
  )

(defmethod STARO-robot
  (:limb
   (limb method &rest args)
   (case method
     (:grasp-end-coords
      (case limb
        (:larm (user::forward-message-to (send self :get :larm-grasp-end-coords) args))
        (:rarm (user::forward-message-to (send self :get :rarm-grasp-end-coords) args))
        (t nil)
        )
      )
     (t (send-super* :limb limb method args))
     )
   )
  )

(defmethod STARO-robot
  (:init-ending
   (&rest args)
   (prog1
    (send-super* :init-ending args)
    (send self :add-additional-body-parts)

    ;; add grasp-end-coords to arms
    (let (rarm-grasp-end-coords larm-grasp-end-coords)
      (setq rarm-grasp-end-coords (make-cascoords :coords (send (send self :rarm :end-coords :parent) :copy-worldcoords) :name :rarm-grasp-end-coords))
      (setq larm-grasp-end-coords (make-cascoords :coords (send (send self :larm :end-coords :parent) :copy-worldcoords) :name :larm-grasp-end-coords))
      (send (send (send (send rarm-grasp-end-coords :rotate (deg2rad 45) :y)
                        :translate (float-vector 0.0 -146.0 -89.5)) ;; from CAD
                  :rotate (deg2rad -90) :x) ;; make :z parallel to palm
            :rotate (deg2rad 90) :z) ;; make :x normal of palm
      (send (send (send (send larm-grasp-end-coords :rotate (deg2rad 45) :y)
                        :translate (float-vector 0.0 146.0 -89.5)) ;; from CAD
                  :rotate (deg2rad 90) :x) ;; make :z parallel to palm
            :rotate (deg2rad -90) :z) ;; make :x normal of palm
      (send self :put :rarm-grasp-end-coords rarm-grasp-end-coords)
      (send self :put :larm-grasp-end-coords larm-grasp-end-coords)
      (send (send self :rarm :end-coords :parent) :assoc (send self :get :rarm-grasp-end-coords))
      (send (send self :larm :end-coords :parent) :assoc (send self :get :larm-grasp-end-coords)))

    ;; remove 7-9 bodies from (car (send *staro* :links)) of pqpmodel by ueda-san
    (let ((target-link (car (send self :links)))
          (fat 0)
          (m (geometry::pqpmakemodel))
          vs v1 v2 v3 (id 0))
      (setf (get target-link :pqpmodel) m)
      (let* ((candidate-bodies (subseq (send target-link :bodies) 0 7))
             (fs (flatten (send-all candidate-bodies :faces))))
        (geometry::pqpbeginmodel m)
        (dolist (f fs)
          (dolist (poly (geo::face-to-triangle-aux f))
            (setq vs (send poly :vertices)
                  v1 (send target-link :inverse-transform-vector (first vs))
                  v2 (send target-link :inverse-transform-vector (second vs))
                  v3 (send target-link :inverse-transform-vector (third vs)))
            (when (not (= fat 0))
              (setq v1 (v+ v1 (scale fat (normalize-vector v1)))
                    v2 (v+ v2 (scale fat (normalize-vector v2)))
                    v3 (v+ v3 (scale fat (normalize-vector v3)))))
            (geometry::pqpaddtri m v1 v2 v3 id)
            (incf id)
            )
          )
        (geometry::pqpendmodel m)
        (format t "pqp: ~A" m)
        m))
    )
   )
  (:add-additional-body-parts
   ()
   (dolist (leg '(:rleg :lleg))
     (let ((b (make-cube 100 100 50)))
       (send b :put :face-color #f(1 1 1))
       (send b :newcoords
             (send (send (send (elt (send self leg :links) 3) :copy-worldcoords) :translate (float-vector 100 0 -80))
                   :rotate (deg2rad 60) :y))
       (send (elt (send self leg :links) 3) :assoc b)
       (setq ((elt (send self leg :links) 3) . geo::bodies)
             (append (send (elt (send self leg :links) 3) :bodies) (list b)))
       )))
 )
