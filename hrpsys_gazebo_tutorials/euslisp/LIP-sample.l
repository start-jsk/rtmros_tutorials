(ros::roseus-add-msgs "hrpsys_gazebo_msgs")
(ros::roseus-add-srvs "hrpsys_gazebo_msgs")

(ros::roseus "LIP_sample")

(defun switch-foot
  (&key
   (pos (float-vector 0 0 0)) ;; [mm]
   (mass-vel (float-vector 0 0 0)) ;; [mm/sec]
   (keep-mass-vel t)
   )
  (let* ((req (instance hrpsys_gazebo_msgs::LIPSwitchFootRequest :init)))
    (send (send req :command) :foot_position (ros::pos->tf-translation pos))
    (send (send req :command) :mass_velocity (ros::pos->tf-translation mass-vel))
    (send (send req :command) :keep_mass_velocity keep-mass-vel)
    (ros::service-call "/LIP/switch_foot" req))
  )
(format t "(switch-foot :pos #f(1000 0 0))~%")

(defun set-state
  (&key
   (foot-pos (float-vector 0 0 0)) ;; [mm]
   (mass-pos (float-vector 0 0 2000)) ;; [mm]
   (mass-vel (float-vector 0 0 0)) ;; [mm/sec]
   )
  (let* ((req (instance hrpsys_gazebo_msgs::LIPSetStateRequest :init)))
    (send (send req :command) :foot_position (ros::pos->tf-translation foot-pos))
    (send (send req :command) :mass_position (ros::pos->tf-translation mass-pos))
    (send (send req :command) :mass_velocity (ros::pos->tf-translation mass-vel))
    (ros::service-call "/LIP/set_state" req))
  )
(format t "(set-state :foot-pos #f(0 0 0) :mass-pos #f(0 0 2000) :mass-vel #f(0 0 0))~%")

(defun get-state-initialize
  ()
  (ros::subscribe "/LIP/state" hrpsys_gazebo_msgs::LIPState #'get-state-callback)
  (setq *state* nil)
  )

(defun get-state-callback
  (msg)
  (setq *state* nil)
  (push (list :foot-position (ros::tf-point->pos (send msg :foot_position))) *state*)
  (push (list :mass-position (ros::tf-point->pos (send msg :mass_position))) *state*)
  (push (list :mass-velocity (ros::tf-point->pos (send msg :mass_velocity))) *state*)
  )


(defun capture-point-test
  ()
  (unless (boundp '*state*) (get-state-initialize))
  (setq *g* 9.8) ;; [m/sec^2]

  (set-state :foot-pos #f(0 0 0) :mass-pos #f(0 0 2000) :mass-vel #f(0 0 0))
  (unix::usleep (* 100 1000))
  (set-state :foot-pos #f(0 0 0) :mass-pos #f(0 0 2000) :mass-vel #f(0 0 0))

  (let (mass-pos ;; [m]
        mass-vel ;; [m]
        foot-pos ;; [m]
        )
    (while t
     (read-line)

     (ros::spin-once)
     (setq mass-pos (scale 0.001 (cadr (assoc :mass-position *state*))))
     (setq mass-vel (scale 0.001 (cadr (assoc :mass-velocity *state*))))

     (setq foot-pos (v+ (scale (sqrt (/ (elt mass-pos 2) *g*)) mass-vel) mass-pos))
     (setf (elt foot-pos 2) 0)
     (switch-foot :pos (scale 1000 foot-pos))
     )
    )
  )


(defun switch-foot-test
  ()
  (format t "press Enter to finish~%")
  (do-until-key
   (switch-foot :pos #f(1000 0 0))
   (unix::usleep (* 500 1000))
   (switch-foot :pos #f(-1000 0 0))
   (unix::usleep (* 500 1000))
   (switch-foot :pos #f(0 1000 0))
   (unix::usleep (* 500 1000))
   (switch-foot :pos #f(0 -1000 0))
   )
  )
