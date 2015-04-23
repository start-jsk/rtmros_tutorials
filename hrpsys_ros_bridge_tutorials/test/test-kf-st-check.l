;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Test program for KalmanFilter and Stabilizer
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun test-kf-st-check ()
  "Test program for waist bending pose for kf and st test."
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (y-or-n-p ";; Move to initial pose?~%")
  (let ((wy-list (list -44 -30 -15 0 15 30 44))
        (wp-list (list -4 0 20 40 59)))
    (send *robot* :torso :waist-y :joint-angle (car wy-list))
    (send *ri* :angle-vector (send *robot* :angle-vector) 4000)
    (send *ri* :wait-interpolation)
    (dolist (wp wp-list)
      (dolist (wy wy-list)
        (format t ";;   wp ~A[deg], wy ~A[deg]~%" wp wy)
        (send *robot* :torso :waist-p :joint-angle wp)
        (send *robot* :torso :waist-y :joint-angle wy)
        (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
        (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
        (send *ri* :wait-interpolation)
        (unix:sleep 2)
        )
      (send *robot* :torso :waist-y :joint-angle (car wy-list))
      (send *ri* :angle-vector (send *robot* :angle-vector) 4000)
      (send *ri* :wait-interpolation)
      )
    (send *robot* :reset-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
    (send *ri* :angle-vector (send *robot* :angle-vector) 4000)
    (send *ri* :wait-interpolation)
    (print ";; Finish~%")
    t))
(warn ";; (test-kf-st-check)~%")


