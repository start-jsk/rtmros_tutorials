(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :hironxjsk "package://hrpsys_ros_bridge_tutorials/models/hironxjsk.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-utils.l"))
  (require :hironxjsk-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-utils.l"))

(defclass hironxjsk-interface
  :super rtm-ros-robot-interface
  :slots (hand-actions
          hand-servo-num
          on-gazebo-ros-control))

;; Initialize
(defmethod hironxjsk-interface
  (:init (&rest args &key (type nil) &allow-other-keys)
    (setq robot (instance hironxjsk-robot :init))
    ;; Define {limb}-controller, usually we can define manually as jsk_robots
    (dolist (limb '(:rarm :larm :head :torso))
      (send self :def-limb-controller-method limb))
    ;; If gazebo with ros_control, overwrite :default-controller
    (setq on-gazebo-ros-control
          (and (ros::get-param "/gazebo/time_step" nil)
               ;; rtm-ros-bridge does not have type parametrs
               (ros::get-param "/torso_controller/type" nil)))
    (when on-gazebo-ros-control
      (ros::ros-warn "Found Gazebo/ros_control environment"))
    (prog1
      ;; Hironx has two types of joint_states on one topic: whole body and hand,
      ;; so queue size of joint_states should be two.
      ;; https://github.com/jsk-ros-pkg/jsk_pr2eus/blob/0.3.13/pr2eus/robot-interface.l#L120
      (send-super* :init :joint-states-queue-size 2 :robot robot :type
                   (cond (type type)
                         (on-gazebo-ros-control :gazebo-ros-controller)
                         (t :default-controller))
                   args)
      (dolist (limb '(:rarm :larm :head :torso))
        (let ((ctype (read-from-string (format nil "~A-controller" limb))))
          (send self :add-controller ctype)))

      ;; add hand controller for gazebo with ros_control
      (when on-gazebo-ros-control
        (setq hand-actions (make-hash-table))
        (dolist (hand (list :rhand :lhand))
          ;; initialize hand action
          (sethash hand hand-actions
                   (instance ros::simple-action-client :init
                             (format nil "/~A_controller/follow_joint_trajectory_action"
                                       (string-downcase hand))
                             control_msgs::FollowJointTrajectoryAction
                             :groupname groupname))
          ;; check if hand action is respond (based on baxter-interface)
          (unless
              (and joint-action-enable (send (gethash hand hand-actions) :wait-for-server 3))
            (ros::ros-warn "~A is not respond" (gethash hand hand-actions))
            (ros::ros-info "*** if you do not have hand, you can ignore this message ***"))))
      ;; number of servo motors in one hand
      (setq hand-servo-num 4)))
  (:gazebo-ros-controller ()
    (append
     (send self :rarm-controller)
     (send self :larm-controller)
     (send self :head-controller)
     (send self :torso-controller)))
  (:call-operation-return (method &rest args)
    ;; Call method until it returns true
    ;; Used to ensure operation on the hand service calls, that sometimes fail
    (do ((res (send* self method args)
              (send* self method args)))
        ((send res :operation_return) res))))

;; ServoControllerService for hand
;; Based on https://github.com/start-jsk/rtmros_hironx/blob/2.1.0/hironx_ros_bridge/src/hironx_ros_bridge/hironx_client.py
;; and https://github.com/start-jsk/rtmros_tutorials/blob/0.1.6/hrpsys_ros_bridge_tutorials/euslisp/hrp2-common-interface.l
;; Please synchronize with hironxjsk-utils
(defmethod hironxjsk-interface
  (:check-hand-vector-length (vec &optional (hand-num 1))
   ;; Ensure that `vec' is a 4 element vector for single hand or 8 element for double hand
   (let ((len (* hand-num hand-servo-num)))
     (assert (= (length vec) len)
             "[ERROR] Expecting vector of length ~a~%" len)))
  (:hand-angle-vector (hand &optional av (tm 1000))
   (when av
     ;; check type
     (case hand
           (:hands
            (if (= (length av) hand-servo-num) (setq av (concatenate float-vector av av)))
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
       ;; simulation mode
       (when (send self :simulation-modep)
             (return-from :hand-angle-vector
               (if av
                 ;; setjointangles
                 (map nil #'(lambda (joint angle) (send robot joint :joint-angle angle))
                       joint-list av)
                 ;; getjointangles
                 (map float-vector #'(lambda (joint) (send robot joint :joint-angle))
                      joint-list))))
       ;; gazebo with ros_control
       (when on-gazebo-ros-control
         (return-from :hand-angle-vector
           (if av
             ;; setjointangles
             (let ((av-copy (copy-object av)))
               (dolist (r/l (if (eq hand :hands) (list :rhand :lhand) (list hand)))
                 (let* ((action (gethash r/l hand-actions))
                        (goal (send action :make-goal-instance))
                        (st (ros::time-now)))
                   (send goal :header :seq 1)
                   (send goal :header :stamp st)
                   (send goal :goal :trajectory :header :stamp st)
                   (send goal :goal :trajectory :joint_names
                         (mapcar #'string-upcase (get-joint-list r/l)))
                   (send goal :goal :trajectory :points
                         (list (instance trajectory_msgs::JointTrajectoryPoint
                                         :init
                                         :positions (map float-vector #'deg2rad (subseq av-copy 0 hand-servo-num))
                                         :time_from_start (ros::time (/ tm 1000.0)))))
                   (setq av-copy (subseq av-copy hand-servo-num))
                   (send action :send-goal goal)))
               nil)
             ;; getjointangles
             (progn
               (send self :update-robot-state :wait-until-update t)
               (map float-vector #'(lambda (joint) (send robot joint :joint-angle))
                    joint-list)))))))

   ;; real robot
   (if av
     ;; setjointangles
     (let ((av-rad-list (map cons #'deg2rad av)))
       (case hand
             (:hands
              (send self :call-operation-return :servocontrollerservice_setjointangles
                           :jvs av-rad-list :tm (/ tm 1000.0)))
             ((:rhand :lhand)
              (send self :call-operation-return :servocontrollerservice_setjointanglesofgroup
                    :gname (string-downcase hand) :jvs av-rad-list :tm (/ tm 1000.0)))
             (t (error ";; No such hand: ~A~%." hand))))
     ;; getjointangles
     (let ((ids (case hand
                      (:hands (list 2 3 4 5 6 7 8 9))
                      (:rhand (list 2 3 4 5))
                      (:lhand (list 6 7 8 9))
                      (t (error ";; No such hand: ~A~%." hand))))
           (dirs (case hand
                       (:hands #f(1 1 -1 -1 1 1 -1 -1))
                       ((:lhand :rhand) #f(1 1 -1 -1)))))
       ;; servocontrollerservice_getjointangles do not consider servo offset and direction
       ;; servocontrollerservice_getjointangle do not consider servo direction
       ;; defined in /opt/jsk/etc/HIRONX/hrprtc/Robot.conf
       ;; servo.id: 2,3,4,5, 6,7,8,9
       ;; servo.offset: -0.78,0.0,-0.82,0.0, -0.85,0.0,-0.82,0.0
       ;; servo.dir 1,1,-1,-1,1,1,-1,-1
       (map float-vector
            #'(lambda (id dir)
                (* dir (send (send self :call-operation-return :servocontrollerservice_getjointangle :id id) :jv)))
            ids dirs))))
  (:hand-servo-on ()
    (unless (or (send self :simulation-modep) on-gazebo-ros-control)
      (send self :call-operation-return :servocontrollerservice_servoon)))
  (:hand-servo-off ()
    (unless (or (send self :simulation-modep) on-gazebo-ros-control)
      (send self :call-operation-return :servocontrollerservice_servooff)))
  (:hand-effort (&optional (hand :hands) effort)
    ;; effort is percentage or sequence of percentages
    (if (or (send self :simulation-modep) on-gazebo-ros-control) (return-from :hand-effort nil))
    (let ((ids (case hand
                     (:hands (list 2 3 4 5 6 7 8 9))
                     (:rhand (list 2 3 4 5))
                     (:lhand (list 6 7 8 9))
                     (t (error ";; No such hand: ~A~%." hand)))))
      (cond
       ((null effort)
        ;; getmaxtorque
        (mapcar
         #'(lambda (id) (send (send self :call-operation-return :servocontrollerservice_getmaxtorque :id id) :percentage))
         ids))
       ((and (numberp effort) (plusp effort))
        ;; setmaxtorque with same effort value
        (mapcar
         #'(lambda (id) (send self :call-operation-return :servocontrollerservice_setmaxtorque :id id :percentage effort))
         ids))
       ((or (consp effort) (vectorp effort))
        ;; check length
        (case hand
              (:hands
               (if (= (length effort) hand-servo-num)
                   (setq effort (concatenate float-vector effort effort)))
               (send self :check-hand-vector-length effort 2))
              ((:rhand :lhand)
               (send self :check-hand-vector-length effort)))
        ;; setmaxtorque with different effort values
        (map cons
         #'(lambda (id val)
             (if val (send self :call-operation-return :servocontrollerservice_setmaxtorque :id id :percentage val)))
         ids effort))
       (t
        ;; unsupported type
        (error "number or sequence expected")))))
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
  (:hand-width (hand &optional width &key (time 1000) effort)
    ;; Get/Set the hand width
    (if width
      ;; set hand width
      (progn
        (when effort (send self :hand-effort hand effort))
        (send self :hand-angle-vector hand (send self :hand-width2angles width) time))
      ;; get hand width
      (send self :hand-angles2width (send self :hand-angle-vector hand))))
  (:start-grasp (&optional (arm :arms) &key (time 1000) effort)
    (case arm
          (:arms (setq arm :hands))
          (:rarm (setq arm :rhand))
          (:larm (setq arm :lhand)))
    (send self :hand-width arm 0 :time time :effort effort))
  (:stop-grasp (&optional (arm :arms) &key (time 1000) effort)
    (case arm
          (:arms (setq arm :hands))
          (:rarm (setq arm :rhand))
          (:larm (setq arm :lhand)))
    (send self :hand-width arm 100 :time time :effort effort)))

(defun hironxjsk-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* hironxjsk-interface :init args)))
  (if (not (boundp '*hironxjsk*))
      (setq *hironxjsk* (instance hironxjsk-robot :init)))
  ;; read initial robot state
  (send *hironxjsk* :angle-vector (send *ri* :state :potentio-vector))
  ;; return robot instance
  *hironxjsk*)
