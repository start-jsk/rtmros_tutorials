(load "package://hrpsys_ros_bridge_tutorials/models/hironxjsk.l")

(defmethod hironxjsk-robot
  (:init-ending
    (&rest args)
    (prog1
      (send-super* :init-ending args)
      (send self :add-hand-tip-coords)
      ;; number of servo motors in one hand
      (send self :put :hand-servo-num 4)))
  ;; Based on https://github.com/start-jsk/rtmros_tutorials/blob/34cd4d03556a6630c8e89058a24c96edd779edaa/hrpsys_ros_bridge_tutorials/euslisp/jaxon-utils.l#L75-L93
  (:add-hand-tip-coords
    (&key (offset (float-vector 0 0 -43)))
    (let* ((limb (list :rarm :larm))
           (name (list :rhand-tip-coords :lhand-tip-coords))
           tmpcec)
      (mapcar #'(lambda (l n)
                  (setq tmpcec
                        (make-cascoords
                          :init :link-list
                          :parent (send self l :end-coords)
                          :coords (send (send self l :end-coords :copy-worldcoords)
                                        :translate offset)
                          :name n))
                  (send self :put n tmpcec)
                  (send (send self l :end-coords :parent) :assoc (send self :get n)))
              limb name))))

;; Hand interface
;; Please synchronize with hironxjsk-interface
(defmethod hironxjsk-robot
  (:check-hand-vector-length (vec &optional (hand-num 1))
    ;; Ensure that `vec' is a 4 element vector for single hand or 8 element for double hand
    (let ((len (* hand-num (send self :get :hand-servo-num))))
      (assert (= (length vec) len)
              "[ERROR] Expecting vector of length ~a~%" len)))
  (:hand-angle-vector (hand &optional av)
    (when av
      ;; check type
      (case hand
        (:hands
          (if (= (length av) (send self :get :hand-servo-num))
            (setq av (concatenate float-vector av av)))
          (send self :check-hand-vector-length av 2))
        ((:rhand :lhand)
         (send self :check-hand-vector-length av))))

    (flet ((get-joint-list (hand)
             (let (acc)
               (dotimes (i 4) (push (read-from-string (format nil "~a_joint~a" hand i)) acc))
               (nreverse acc))))
      (let ((joint-list (case hand
                          (:hands (append (get-joint-list :rhand) (get-joint-list :lhand)))
                          ((:rhand :lhand) (get-joint-list hand))
                          (t (error ";; No such hand: ~A~%." hand)))))
        (return-from :hand-angle-vector
          (if av
            ;; setjointangles
            (map nil #'(lambda (joint angle) (send self joint :joint-angle angle))
                  joint-list av)
            ;; getjointangles
            (map float-vector #'(lambda (joint) (send self joint :joint-angle))
                 joint-list))))))
  (:hand-width2angles (width)
    ;; Calculates the hand angles to achieve a certain parallel aperture
    (let ((safetymargin 3) (w0 19) (l1 41.9))
      (unless (<= (- safetymargin) width %(2 * (w0 + l1 - safetymargin)))
        (warn ";; width value ~a is off margins~%" width)
        (return-from :hand-width2angles nil))
      (let ((a (rad2deg %(pi/2 - acos((width / 2.0 + safetymargin - w0) / l1)))))
        (float-vector a (- a) (- a) a))))
  (:hand-angles2width (vec)
    ;; Calculates the hand aperture given a certain angle vector
    (send self :check-hand-vector-length vec)
    (let ((safetymargin 3) (w0 19) (l1 41.9) (l2 20))
      (flet ((get-width (r1 r2) %( w0 + l1 * cos(pi/2 - r1) + l2 * cos(pi/2 - r1 - r2) - safetymargin)))
            (multiple-value-bind (a1 a2 b1 b2) (map cons #'deg2rad vec)
              (+ (get-width a1 a2)
                 (get-width (- b1) (- b2)))))))
  (:hand-width (hand &optional width)
    ;; Get/Set the hand width
    (if width
      ;; set hand width
      (progn
        (send self :hand-angle-vector hand (send self :hand-width2angles width)))
      ;; get hand width
      (send self :hand-angles2width (send self :hand-angle-vector hand))))
  (:start-grasp (&optional (arm :arms))
    (case arm
          (:arms (setq arm :hands))
          (:rarm (setq arm :rhand))
          (:larm (setq arm :lhand)))
    (send self :hand-width arm 0))
  (:stop-grasp (&optional (arm :arms))
    (case arm
          (:arms (setq arm :hands))
          (:rarm (setq arm :rhand))
          (:larm (setq arm :lhand)))
    (send self :hand-width arm 100)))
