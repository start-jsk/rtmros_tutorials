(ros::load-ros-manifest "hrpsys_gazebo_msgs")

(when (boundp 'robot-interface)
  (defmethod robot-interface
    (:hand-angle-vector
     (av &optional l/r)
     (cond ((or (equal l/r :lhand) (equal l/r :larm))
            (send hcp :send-command (list av nil)))
           ((or (equal l/r :rhand) (equal l/r :rarm))
            (send hcp :send-command (list nil av)))
           (t
            (send hcp :send-command av))
           ))
    (:hand-angle-state-vector
     (&optional l/r)
     (cond ((or (equal l/r :lhand) (equal l/r :larm))
            (elt (send hcp :get-state) 0))
           ((or (equal l/r :rhand) (equal l/r :rarm))
            (elt (send hcp :get-state) 1))
           (t
            (send hcp :get-state))
           ))
    (:hand-effort-vector
     (ef &optional l/r)
     (cond ((or (equal l/r :lhand) (equal l/r :larm))
            (send hcp :send-effort-command (list ef nil)))
           ((or (equal l/r :rhand) (equal l/r :rarm))
            (send hcp :send-effort-command (list nil ef)))
           (t
            (send hcp :send-effort-command (list ef ef)))
           ))
    ))
(when (boundp 'hrp2jsknt-interface)
  (defmethod hrp2jsknt-interface
    (:init-hand
     ()
     (setq hcp (instance hand-command-publisher-class :init)))
    (:start-grasp
     (&optional l/r)
     (send self :hand-effort-vector (float-vector 0 -25 25 0 25 25) l/r))
    (:stop-grasp
     (&optional l/r)
     (send self :hand-effort-vector (float-vector 0 0 0 0 0 0) l/r)
     )))
(when (boundp 'hrp2jsknts-interface)
  (defmethod hrp2jsknts-interface
    (:init-hand
     ()
     (setq hcp (instance hand-command-publisher-class :init)))
    (:start-grasp
     (&optional l/r)
     (send self :hand-effort-vector (float-vector 0 -25 25 0 25 25) l/r))
    (:stop-grasp
     (&optional l/r)
     (send self :hand-effort-vector (float-vector 0 0 0 0 0 0) l/r)
     )))
(when (boundp 'staro-interface)
  (defmethod staro-interface
    (:init-hand ()
     (setq hcp (instance hand-command-publisher-class :init
                         :left-hand-command-topic  "/L_ROBOTIQ/joint_command"
                         :right-hand-command-topic "/R_ROBOTIQ/joint_command"
                         :left-hand-state-topic  "/L_ROBOTIQ/robot_state"
                         :right-hand-state-topic "/R_ROBOTIQ/robot_state"
                         :number-of-joints 12)))
    (:start-grasp
     (&optional l/r)
     (send self :hand-angle-vector (float-vector 80.0 60.0 0.0 80.0 60.0 0.0 80.0 60.0 0.0 0.0 0.0 0.0) l/r)
     )
    (:stop-grasp
     (&optional l/r)
     (send self :hand-angle-vector (float-vector 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0) l/r)
     )
    ))
;;(warn "~%")
;;(warn "(send *ri* (list (float-vector 10 20 30 40 50 60) (float-vector 50 40 30 20 10 0)))")
;;(warn "(send *ri* :get-state)~%")
;;(warn "(send *ri* :hand-angle-vector #f(10 20 30 40 50 60) :larm)")
;;(warn "(send *ri* :get-state :larm)~%")

(defclass hand-command-publisher-class
  :super propertied-object
  :slots (number-of-joints
          left-hand-av right-hand-av
          lhand-com rhand-com
          lhand-state rhand-state))
(defmethod hand-command-publisher-class
  (:init (&key (left-hand-command-topic  "/HRP3HAND_L/joint_command")
               (right-hand-command-topic "/HRP3HAND_R/joint_command")
               (left-hand-state-topic  "/HRP3HAND_L/robot_state")
               (right-hand-state-topic "/HRP3HAND_R/robot_state")
               ((:number-of-joints tmp-number-of-joints) 6))
   (setq lhand-com left-hand-command-topic
         rhand-com right-hand-command-topic
         lhand-state left-hand-state-topic
         rhand-state right-hand-state-topic
         number-of-joints tmp-number-of-joints)
   (send self :advertise-subscribe-topics)
   ;; (ros::rate 100)
   )
  (:advertise-subscribe-topics ()
   (ros::advertise lhand-com hrpsys_gazebo_msgs::JointCommand 1)
   (ros::advertise rhand-com hrpsys_gazebo_msgs::JointCommand 1)
   (ros::subscribe lhand-state hrpsys_gazebo_msgs::RobotState #'send self :left-hand-cb)
   (ros::subscribe rhand-state hrpsys_gazebo_msgs::RobotState #'send self :right-hand-cb)
   self)
  (:send-command
   (av-list)
   (let* (left-av right-av)
     (cond
      ((vectorp av-list)
       (setq left-av (if (>= (length av-list) number-of-joints)
                          (subseq av-list 0 number-of-joints)))
       (setq right-av (if (>= (length av-list) (* number-of-joints 2))
                         (subseq av-list numebr-of-joints (* number-of-joints 2))))
       )
      (t
       (setq left-av (elt av-list 0))
       (setq right-av (elt av-list 1))))
     (setq msg (instance hrpsys_gazebo_msgs::JointCommand :init))
     (when left-av
       (send msg :position (concatenate float-vector (mapcar #'deg2rad (concatenate cons left-av))))
       (ros::publish lhand-com msg))
     (when right-av
       (send msg :position (concatenate float-vector (mapcar #'deg2rad (concatenate cons right-av))))
       (ros::publish rhand-com msg)))
   )
  (:send-effort-command
   (ef-list)
   (let* ((left-ef (elt ef-list 0))
	  (right-ef (elt ef-list 1)))
     (setq msg (instance hrpsys_gazebo_msgs::JointCommand :init))
     (when left-ef
       (send msg :effort left-ef)
       (ros::publish lhand-com msg))
     (when right-ef
       (send msg :effort right-ef)
       (ros::publish rhand-com msg)))
   )
  (:get-state ()
   (ros::spin-once)
   (list left-hand-av right-hand-av))
  (:left-hand-cb
   (msg)
   (setq left-hand-av  (scale (/ 180.0 pi) (send msg :position))))
  (:right-hand-cb
   (msg)
   (setq right-hand-av (scale (/ 180.0 pi) (send msg :position))))
  )
;;(warn "~%")
;;(warn "(setq *hc* (instance hand-command-publisher-class :init))")
;;(warn "(send *hc* :send-command (list (float-vector 0 0 0 0 0 0) (float-vector 30 30 30 30 30 30)))~%")
;;(warn "(send *hc* :send-command (list nil (float-vector 30 30 30 30 30 30)))~%")

(defun send-hand-pose
  (hpose &optional l/r)
  (cond ((or (equal l/r :lhand) (equal l/r :larm))
	 (send *ri* :hand-angle-vector (send (send *hrp2* :hand :larm) hpose) :larm))
	((or (equal l/r :rhand) (equal l/r :rarm))
	 (send *ri* :hand-angle-vector (send (send *hrp2* :hand :rarm) hpose) :rarm))
	(t
	 (send *ri* :hand-angle-vector (send (send *hrp2* :hand :larm) hpose) :larm)
	 (send *ri* :hand-angle-vector (send (send *hrp2* :hand :rarm) hpose) :rarm))))

(defun init-hand-pose
  (&optional l/r)
  (send-hand-pose :init-pose l/r))

(defun open-hand-pose
  (&optional l/r)
  (send-hand-pose :open-pose l/r))

(defun close-hand-pose
  (&optional l/r)
  (send-hand-pose :close-pose l/r))

(defun hook-hand-pose
  (&optional l/r)
  (send-hand-pose :hook-pose l/r))

(defun grasp-hand-pose
  (&optional l/r)
  (send-hand-pose :grasp-pose l/r))


(defun start-grasp
  (&optional l/r)
  (send *ri* :start-grasp l/r))

(defun stop-grasp
  (&optional l/r)
  (send *ri* :stop-grasp l/r))
