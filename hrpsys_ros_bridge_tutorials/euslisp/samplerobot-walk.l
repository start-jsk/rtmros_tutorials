(ros::roseus "samplerobot_walk")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")

(samplerobot-init)
(objects (list *sr*))

(defun samplerobot-walk
  ()
  (send *sr* :reset-pose)
  (send *ri* :angle-vector (send *sr* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :go-pos 0.5 0.15 45) ;; x[m] y[m] th[rad]
  )
(warn ";; (samplerobot-walk) ;; example to specify target x, y, theta~%")

(defun samplerobot-walk2
  ()
  (send *sr* :reset-pose)
  (send *ri* :angle-vector (send *sr* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (print ";; Press Enter to Stop")
  (ros::rate 10)
  (send *ri* :cmd-vel-mode)
  )
(warn ";; (samplerobot-walk2) ;; example to specify target velocity ;; please use \"teleop_pr2_keyboard\" terminal~%")

(defun samplerobot-walk3 ()
  (send *sr* :reset-pose)
  (send *sr* :fix-leg-to-coords (make-coords))
  (send *ri* :angle-vector (send *sr* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *sr* :legs :crotch-y :min-angle -90)
  (send *sr* :legs :crotch-y :max-angle 90)
  (send *ri* :set-foot-steps (send *sr* :go-pos-params->footstep-list 100 50 20))
  )

(defun samplerobot-walk4 ()
  (send *sr* :reset-pose)
  (send *sr* :fix-leg-to-coords (make-coords))
  (send *sr* :legs :move-end-pos #f(0 0 100) :world)
  (send *sr* :fix-leg-to-coords (make-coords))
  (send *ri* :angle-vector (send *sr* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *sr* :legs :crotch-y :min-angle -90)
  (send *sr* :legs :crotch-y :max-angle 90)
  (send *ri* :set-gait-generator-param :default-step-height 0.15)
  (send *ri* :set-auto-balancer-param :default-zmp-offsets
        (list (float-vector 20 0 0) (float-vector 20 0 0)))
  (send *ri* :start-st)
  (let ((fs (list
             (send *sr* :lleg :end-coords :copy-worldcoords)
             (send (send *sr* :rleg :end-coords :copy-worldcoords) :translate (float-vector 270 0 110))
             (send (send *sr* :lleg :end-coords :copy-worldcoords) :translate (float-vector 270 0 110))
             (send (send *sr* :rleg :end-coords :copy-worldcoords) :translate (float-vector 580 0 210))
             (send (send *sr* :lleg :end-coords :copy-worldcoords) :translate (float-vector 580 0 210))
             (send (send *sr* :rleg :end-coords :copy-worldcoords) :translate (float-vector 890 0 310))
             (send (send *sr* :lleg :end-coords :copy-worldcoords) :translate (float-vector 890 0 310))
             (send (send *sr* :rleg :end-coords :copy-worldcoords) :translate (float-vector 1200 0 410))
             (send (send *sr* :lleg :end-coords :copy-worldcoords) :translate (float-vector 1200 0 410))
             (send (send *sr* :rleg :end-coords :copy-worldcoords) :translate (float-vector 1510 0 510))
             (send (send *sr* :lleg :end-coords :copy-worldcoords) :translate (float-vector 1510 0 510))
             )))
    (dolist (f fs) (send f :put :l/r (if (oddp (position f fs)) :rleg :lleg)))
    (send *ri* :set-foot-steps fs)
    (send *ri* :stop-st)
    ))

(defun samplerobot-walk-eus
  ()
  ;; limit crotch-y range which are originally inf and -inf
  (send *sr* :legs :crotch-y :min-angle -90)
  (send *sr* :legs :crotch-y :max-angle 90)
  (format t ";; generate pattern~%")
  (let ((pattern-list
         (send *sr* :calc-walk-pattern-from-footstep-list
               (send *sr* :go-pos-params->footstep-list
                     500 150 45) ;; x[mm] y[mm] th[rad]
               :debug-view :no-message :dt 0.01))
        (fname (format nil "~A/samplerobot-walk-pattern"
                       (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/build"))))
    (format t ";; dump pattern file~%")
    (dump-seq-pattern-file pattern-list fname)
    (format t ";; load pattern file~%")
    (send *ri* :load-pattern fname)
    ))

