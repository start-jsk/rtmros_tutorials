(require :staro "package://hrpsys_ros_bridge_tutorials/models/staro.l")

;; add robotiq hand model to *staro*
(if (ros::rospack-find "robotiq_s_model_control")
    (progn (load "package://staro_ros_bridge/euslisp/s-model_articulated.l")
           ;; STARO-robot has only robotiq model
           (unless (assoc :init-org (send STARO-robot :methods))
             (rplaca (assoc :init (send STARO-robot :methods)) :init-org))
           (defmethod STARO-robot
             (:init
              (&rest args)
              (prog1
                  (send* self :init-org args)
                (send self :put :lhand-model (instance s-model_articulated-robot :init))
                (send self :put :rhand-model (instance s-model_articulated-robot :init))
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
                    (let (target ret)
                      (case arm
                        (:larm
                         (setq target (send self :get :lhand-model))
                         (setq ret (forward-message-to target args)))
                        (:rarm
                         (setq target (send self :get :rhand-model))
                         (setq ret (forward-message-to target args)))
                        ((:both :arms)
                         (setq target (send self :get :lhand-model))
                         (push (forward-message-to target args) ret)
                         (setq target (send self :get :rhand-model))
                         (push (forward-message-to target args) ret))
                        )
                      ret))
             )           
           )
  (warn  "~c[33m [WARN] robotiq_s_model_control for RobotiqHand cannnot be found. skip definition for RobotiqHand methods. ~c[m~%" #x1b #x1b) ;; ~c: escape sequence
  )

;; add contact end-coords to landing with yukizaki attachment
(defmethod STARO-robot
  (:limb
   (limb method &rest args)
   (case method
     (:contact-end-coords
      (case limb
        (:larm (user::forward-message-to (send self :get :larm-contact-end-coords) args))
        (:rarm (user::forward-message-to (send self :get :rarm-contact-end-coords) args))
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
    (when (member :define-min-max-table (send self :methods))
      (send self :define-min-max-table))
    (send self :add-additional-body-parts)
    
    ;; add contact-end-coords to arms
    (let (rarm-contact-end-coords larm-contact-end-coords)
      (setq rarm-contact-end-coords (make-cascoords :coords (send (send self :rarm :end-coords :parent) :copy-worldcoords) :name :rarm-contact-end-coords))
      (setq larm-contact-end-coords (make-cascoords :coords (send (send self :larm :end-coords :parent) :copy-worldcoords) :name :larm-contact-end-coords))
      (send (send (send (send rarm-contact-end-coords :rotate (deg2rad 45) :y)
                        :translate (float-vector 0.0 -239.3 0.0))
                  :rotate (deg2rad -90) :x)
            :rotate (deg2rad 90) :z)
      (send (send (send (send larm-contact-end-coords :rotate (deg2rad 45) :y)
                        :translate (float-vector 0.0 239.3 0.0))
                  :rotate (deg2rad 90) :x)
            :rotate (deg2rad -90) :z)
      (send self :put :rarm-contact-end-coords rarm-contact-end-coords)
      (send self :put :larm-contact-end-coords larm-contact-end-coords)
      (send (send self :rarm :end-coords :parent) :assoc (send self :get :rarm-contact-end-coords))
      (send (send self :larm :end-coords :parent) :assoc (send self :get :larm-contact-end-coords)))

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
 (:draw-all-limbs
  ()
  (mapcar #'(lambda (limb) (send (send self limb :end-coords) :draw-on :flush t :color #f(0 1 0))) '(:rleg :lleg :rarm :larm)))
 (:fix-body-to-coords
  (fix-coords)
  (let (support-coords tmp-coords move-coords)
    (setq support-coords (send self :body_lk))
    (setq tmp-coords (send fix-coords :copy-worldcoords))
    (setq move-coords (send support-coords :transformation self))
    (send tmp-coords :transform move-coords :local)
    (send self :newcoords tmp-coords)
    (send self :worldcoords)
    tmp-coords))
 ;; pose
 (:servo-off-walking-pose
  (&key (root-link-height-offset 0))
  (send self :reset-servo-off-pose)
  (let ((tav (send self :torso :angle-vector))
        (rav (send self :rarm :angle-vector))
        (lav (send self :larm :angle-vector)))
    (send self :reset-pose)
    (send self :fix-leg-to-coords (make-coords))
    ;; copy servo-off-pose upper body joint angles
    (send self :torso :angle-vector tav)
    (send self :rarm :angle-vector rav)
    (send self :larm :angle-vector lav)
    (send self :torso :waist-p :joint-angle 50)
    ;; move hand outward
    (send self :rarm :move-end-pos #f(70 -150 70) :world :rotation-axis nil)
    (send self :larm :move-end-pos #f(70 150 70) :world :rotation-axis nil)
    (send self :legs :move-end-pos #f(0 0 50))
    (send self :fix-leg-to-coords (make-coords))
    (send self :move-centroid-on-foot :both '(:rleg :lleg))))
 )

;; copy real robot pose to *staro*
(defun adapt-model
  ()
  (send *staro* :angle-vector (send *ri* :state :potentio-vector)))

;; copy *staro* pose to *ri* and real robot
(defun send-pose
  (&optional (send-time 20000))
  (send *ri* :angle-vector (send *staro* :angle-vector) send-time))

;; copy real robot pose to *staro* and *ri*
(defun adapt
  ()
  (adapt-model)
  (send-pose 1000))
