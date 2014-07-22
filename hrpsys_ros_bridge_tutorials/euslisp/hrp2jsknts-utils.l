(load "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknts.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp3hand-utils.l")

(unless (assoc :init-org (send hrp2jsknts-robot :methods))
  (rplaca (assoc :init (send hrp2jsknts-robot :methods)) :init-org))
(eval
 `(defmethod hrp2jsknts-robot
    ,@(get-hrp2-with-hand-class-methods)))

(defmethod hrp2jsknt-robot
  (:move-centroid-on-foot
   (leg fix-limbs
        &rest args
        &key (mid 0.5)
        (use-toes (make-list (length (remove-if-not #'(lambda (x) (memq x '(:rleg :lleg))) fix-limbs)) :initial-element nil))
        (target-centroid-pos (send self :calc-target-centroid-pos leg (remove-if-not #'(lambda (x) (memq x '(:rleg :lleg))) fix-limbs) use-toes mid))
        &allow-other-keys)
   (send-super*
    :move-centroid-on-foot leg fix-limbs
    :use-toes use-toes :target-centroid-pos target-centroid-pos
    args))
  (:calc-target-centroid-pos-for-one-leg
   (fix-leg &optional (use-toe) (offset))
   (if use-toe
       (cadr (send (send self :foot-polygon fix-leg '((:toe-p))) :centroid))
     (send (send self fix-leg :end-coords) :transform-vector offset)))
  (:calc-target-centroid-pos
   (centroid-leg fix-legs
                 &optional (use-toes (make-list (length fix-legs) :initial-element nil))
                 (mid 0.5)
                 (offsets
                  (mapcar
                   #'(lambda (x)
                       (cond
                        ((substringp "HRP2JSKNT" (send self :name))
                         (float-vector -20 0 0))
                        (t
                         (float-vector 0 0 0))))
                   (range (length fix-legs)))))
   (if (eq centroid-leg :both)
       (apply
        #'midpoint mid
        (mapcar
         #'(lambda (tmp-leg tmp-ut tmp-of)
             (send self :calc-target-centroid-pos-for-one-leg tmp-leg tmp-ut tmp-of))
         fix-legs use-toes offsets))
     (send self :calc-target-centroid-pos-for-one-leg centroid-leg (car use-toes) (car offsets))))
  (:fullbody-inverse-kinematics
   (target-coords
    &rest args
    &key (fix-limbs '(:rleg :lleg))
    (use-toes (make-list (length fix-limbs) :initial-element nil)) ;; :use-toes '(t nil) -> use toe-joint of :rleg and does not use toe-joint of :lleg
    (target-centroid-pos (send self :calc-target-centroid-pos (if (= (length fix-limbs) 2) :both (car fix-limbs)) fix-limbs use-toes))
    (additional-weight-list)
    &allow-other-keys)
   (let* ((leg-ul (send self :calc-union-link-list
                        (mapcar #'(lambda (fl ut)
                                    (remove-if #'(lambda (x) (unless ut (eq x (send (send self fl :toe-p) :child-link))))
                                               (send self fl :links)))
                                fix-limbs use-toes))))
     (send self :reset-joint-angle-limit-weight-old leg-ul)
     (send-super* :fullbody-inverse-kinematics
                  target-coords
                  :target-centroid-pos target-centroid-pos
                  :additional-weight-list
                  (append
                   additional-weight-list
                   (remove nil
                           (mapcar #'(lambda (l u)
                                       (if (send self l :toe-p)
                                           (list (send self l :toe-p :child-link) (if u 1.0 0.0))))
                                   fix-limbs use-toes))
                   (list
                    (list (car (send self :links))
                          #'(lambda ()
                              ;; set root-link 6dof-joint's weight based on legs' joint limit
                              (let* ((min-weight
                                      (reduce #'(lambda (x y) (min x y))
                                              (coerce (send self :calc-inverse-kinematics-weight-from-link-list
                                                            leg-ul :union-link-list leg-ul) cons))))
                                (fill (instantiate float-vector 6) min-weight)))
                          )
                    ))
                  args)))
  )

(defun range (n)
  (cond ((<= n 0)
         (return-from range nil))
        ((= n 1)
         (list 0))
        (t
         (append (range (- n 1)) (list (- n 1))))))
