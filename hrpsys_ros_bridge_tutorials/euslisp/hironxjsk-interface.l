(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :hironxjsk "package://hrpsys_ros_bridge_tutorials/models/hironxjsk.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-utils.l"))
  (require :hironxjsk-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-utils.l"))
(if (ros::resolve-ros-path "package://hironx_ros_bridge")
  (ros::load-ros-manifest "hironx_ros_bridge"))

(defclass hironxjsk-interface
  :super rtm-ros-robot-interface
  :slots ())

;; Initialize
(defmethod hironxjsk-interface
  ;; Based on https://github.com/start-jsk/rtmros_tutorials/blob/9132c58702b3b193e14271b4c231ad0080187850/hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l
  (:init (&rest args)
    (prog1
      ;; Hironx has two types of joint_states on one topic: whole body and hand,
      ;; so queue size of joint_states should be two.
      ;; https://github.com/jsk-ros-pkg/jsk_pr2eus/blob/0.3.13/pr2eus/robot-interface.l#L120
      (send-super* :init :joint-states-queue-size 2 :robot hironxjsk-robot args)
      ;; add controller
      (dolist (limb '(:rarm :larm :head :torso))
        (send self :def-limb-controller-method limb)
        (send self :add-controller (read-from-string (format nil "~A-controller" limb))
              :joint-enable-check t :create-actions t))
      ;; Load param to erase offset of force moment sensor
      (send self :load-forcemoment-offset-param
            (format nil "~A/models/~A-force-moment-offset.l"
                    (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials")
                    (send robot :name))
            :set-offset t)))
  (:define-all-ROSBridge-srv-methods
    (&key (debug-view nil) (ros-pkg-name "hrpsys_ros_bridge"))
    ;; First, define ROSBridge method for old impedance controller
    (if (ros::resolve-ros-path "package://hironx_ros_bridge")
      (send-super :define-all-ROSBridge-srv-methods :ros-pkg-name "hironx_ros_bridge"))
    ;; Second, define ROSBridge method based on hrpsys_ros_bridge
    ;; Method created already is not overwritten, so we can keep using old impedance controller
    ;; See :get-ROSBridge-method-def-macro
    (send-super :define-all-ROSBridge-srv-methods))
  (:call-operation-return (method &rest args)
    (do ((res (send* self method args)
              (send* self method args)))
        ((send res :operation_return) res))))

;; AbsoluteForceSensor
;; Overwrite RemoveForceSensorLinkOffset methods
;; Based on https://github.com/start-jsk/rtmros_common/blob/1.4.2/hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l
(def-set-get-param-method 'hironx_ros_bridge::OpenHRP_AbsoluteForceSensorService_ForceMomentOffsetParam
  :raw-set-forcemoment-offset-param :raw-get-forcemoment-offset-param :get-forcemoment-offset-param-arguments
  :absoluteforcesensorservice_setforcemomentoffsetparam :absoluteforcesensorservice_getforcemomentoffsetparam
  :optional-args (list :name 'name))

(defmethod rtm-ros-robot-interface
  (:zero-set-forcemoment-offset-param
   (limb)
   "Set RemoveForceSensorLinkOffset's params offset to zero."
   (send self :set-forcemoment-offset-param limb :force-offset #f(0 0 0) :moment-offset #f(0 0 0) :link-offset-centroid #f(0 0 0) :link-offset-mass 0)
   )
  (:set-forcemoment-offset-param
   (limb &rest args)
   "Set RemoveForceSensorLinkOffset params for given limb.
    For arguments, please see (send *ri* :get-forcemoment-offset-param-arguments)."
   (send* self :force-sensor-method
         limb
         #'(lambda (name &rest _args)
             (send* self :raw-set-forcemoment-offset-param (send (car (send robot name :force-sensors)) :name) _args))
         :set-forcemoment-offset-param
         args))
  (:get-forcemoment-offset-param
   (limb)
   "Get RemoveForceSensorLinkOffset params for given limb."
   (send self :force-sensor-method
         limb
         #'(lambda (name &rest _args)
             (send self :raw-get-forcemoment-offset-param (send (car (send robot name :force-sensors)) :name)))
         :get-forcemoment-offset-param))
  (:load-forcemoment-offset-param
   (fname &key (set-offset t))
   "Load AbsoluteForceSensor params from fname (file path)."
   (mapcar #'(lambda (x)
               (send* self :set-forcemoment-offset-param (car x)
                      (if set-offset
                          (cdr x)
                        (list :link-offset-mass (cadr (memq :link-offset-mass (cdr x)))
                              :link-offset-centroid (cadr (memq :link-offset-centroid (cdr x)))))))
           (with-open-file
            (f fname :direction :input)
            (read f nil nil)))
   )
  (:load-forcemoment-offset-params (&rest args)
    (error ";; :load-forcemoment-offset-params cannot be used with hironx~%"))
  (:dump-forcemoment-offset-params (&rest args)
    (error ";; :dump-forcemoment-offset-params cannot be used with hironx~%"))
  (:remove-force-sensor-offset-rmfo (&rest args)
    (error ";; :remove-force-sensor-offset-rmfo cannot be used with hironx~%"))
  (:remove-force-sensor-offset-rmfo-arms (&rest args)
    (error ";; :remove-force-sensor-offset-rmfo-arms cannot be used with hironx~%"))
  (:remove-force-sensor-offset-rmfo-legs (&rest args)
    (error ";; :remove-force-sensor-offset-rmfo-legs cannot be used with hironx~%"))
  ;; Deprecated in https://github.com/start-jsk/rtmros_common/pull/1010,
  ;; but new methods cannot be used in hironx.
  ;; So we revert deprecated methods to the ones before the PR and use them.
  (:reset-force-moment-offset-arms
   ()
   "Remove force and moment offset for :rarm and :larm"
   (send self :reset-force-moment-offset '(:rarm :larm)))
  (:reset-force-moment-offset-legs (&rest args)
    (error ";; :reset-force-moment-offset-legs cannot be used with hironx~%"))
  (:reset-force-moment-offset
   (limbs)
   "Remove force and moment offsets. limbs should be list of limb symbol name."
   (send self :_reset-force-moment-offset limbs :force)
   (send self :_reset-force-moment-offset limbs :moment)
   )
  (:_reset-force-moment-offset
   (limbs f/m &key (itr 10))
   (let* ((params (mapcar #'(lambda (alimb) (send self :get-forcemoment-offset-param alimb)) limbs)))
     (labels ((calc-off
               (alimb)
               (send self (if (eq f/m :force) :off-force-vector :off-moment-vector) alimb))
              (get-avg-fm
               ()
               (let ((fm (mapcar #'(lambda (i)
                                     (send self :state)
                                     (mapcar #'(lambda (alimb) (send self (if (eq f/m :force) :off-force-vector :off-moment-vector) alimb)) limbs))
                                 (make-list itr))))
                 (mapcar #'(lambda (alimb)
                             (let ((idx (position alimb limbs)))
                               (vector-mean (mapcar #'(lambda (d) (elt d idx)) fm))))
                         limbs))))
       ;; estimate offsets
       (let* ((tmp-fm-offsets (mapcar #'(lambda (i)
                                          (send self :state)
                                          (mapcar #'calc-off limbs))
                                      (make-list itr)))
              (new-fm-offsets (mapcar #'(lambda (alimb)
                                          (let ((idx (position alimb limbs)))
                                            (vector-mean (mapcar #'(lambda (d) (elt d idx)) tmp-fm-offsets))))
                                      limbs))
              (org-fm-list (get-avg-fm)))
         ;; set offsets
         (mapcar #'(lambda (alimb new-fm-offset param)
                     (send self :set-forcemoment-offset-param alimb
                           (if (eq f/m :force) :force-offset :moment-offset)
                           (v+ (if (eq f/m :force)
                                   (send param :force_offset)
                                 (send param :moment_offset))
                               new-fm-offset)))
                 limbs new-fm-offsets params)
         (unix:usleep 10000)
         ;; check ;; compare sensor value before & after resetting
         (mapcar #'(lambda (alimb org-fm new-fm)
                     (format t ";; ~A error of ~A ;; ~A[~A] -> ~A[~A]~%"
                             (string-downcase f/m) alimb
                             (norm org-fm) (if (eq f/m :force) "N" "Nm")
                             (norm new-fm) (if (eq f/m :force) "N" "Nm")))
                 limbs org-fm-list (get-avg-fm))
         ))))
  )

;; ImpedanceControllerService
;; Based on https://github.com/start-jsk/rtmros_hironx/blob/2.1.0/hironx_ros_bridge/src/hironx_ros_bridge/hironx_client.py
;; and https://github.com/start-jsk/rtmros_common/blob/1.4.2/hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l.
;; Enable methods executable with old impedance controller and disable others.
;; The reason why I don't use def-set-get-param-method is that
;; OpenHRP_ImpedanceControllerService_setImpedanceControllerParam.srv has element "name" inside i_param,
;; while OpenHRP_ImpedanceControllerService_getImpedanceControllerParam.srv has that element directly.
;; Set optional-args as (list :name 'name) -> multiple declaration of variable "name" in set-param-method.
;; Set optional-args as nil -> pass nothing in service call of get-param-method.
(defmethod hironxjsk-interface
  (:raw-set-impedance-controller-param (&rest args)
    (error ";; :raw-set-impedance-controller-param cannot be used with hironx~%"))
  (:raw-get-impedance-controller-param (&rest args)
    (error ";; :raw-get-impedance-controller-param cannot be used with hironx~%"))
  (:start-impedance
    (limb &rest args &key (m-p 100) (d-p 100) (k-p 100) (m-r 100) (d-r 2000) (k-r 2000)
          (ref-force #f(0 0 0)) (force-gain #f(1 1 1)) (ref-moment #f(0 0 0))
          (moment-gain #f(0 0 0)) (sr-gain 1) (avoid-gain 0) (reference-gain 0)
          (manipulability-limit 0.1))
    "Start impedance controller mode.
     limb should be limb symbol name such as :rarm, :larm, or :arms."
    (let (sensor-name target-name)
      (cond ((eq limb :rarm)
             (setq sensor-name "rhsensor" target-name "RARM_JOINT5"))
            ((eq limb :larm)
             (setq sensor-name "lhsensor" target-name "LARM_JOINT5"))
            ((eq limb :arms)
             (return-from :start-impedance
                          (mapcar #'(lambda (l) (send* self :start-impedance l args))
                                  '(:rarm :larm))))
            (t (error ";; No such limb: ~A~%." limb)))
      (send self :impedancecontrollerservice_setimpedancecontrollerparam :i_param
            (instance hironx_ros_bridge::OpenHRP_ImpedanceControllerService_impedanceParam :init
                      :name sensor-name :base_name "CHEST_JOINT0" :target_name target-name
                      :m_p m-p :d_p d-p :k_p k-p :m_r m-r :d_r d-r :k_r k-r :ref_force ref-force
                      :force_gain force-gain :ref_moment ref-moment :moment_gain moment-gain
                      :sr_gain sr-gain :avoid_gain avoid-gain :reference_gain reference-gain
                      :manipulability_limit manipulability-limit))))
  (:raw-start-impedance (&rest args)
    (error ";; :raw-start-impedance cannot be used with hironx~%"))
  (:start-impedance-no-wait (&rest args)
    (error ";; :start-impedance-no-wait cannot be used with hironx~%"))
  (:stop-impedance (limb)
    "Stop impedance controller mode.
     limb should be limb symbol name such as :rarm, :larm, or :arms."
    (let (sensor-name)
      (cond ((eq limb :rarm)
             (setq sensor-name "rhsensor"))
            ((eq limb :larm)
             (setq sensor-name "lhsensor"))
            ((eq limb :arms)
             (return-from :stop-impedance
                          (mapcar #'(lambda (l) (send self :stop-impedance l))
                                  '(:rarm :larm))))
            (t (error ";; No such limb: ~A~%." limb)))
      (send self :impedancecontrollerservice_deleteimpedancecontrollerandwait :name sensor-name)))
  (:stop-impedance-no-wait (limb)
    (let (sensor-name)
      (cond ((eq limb :rarm)
             (setq sensor-name "rhsensor"))
            ((eq limb :larm)
             (setq sensor-name "lhsensor"))
            ((eq limb :arms)
             (return-from :stop-impedance-no-wait
                          (mapcar #'(lambda (l) (send self :stop-impedance-no-wait l))
                                  '(:rarm :larm))))
            (t (error ";; No such limb: ~A~%." limb)))
      (send self :impedancecontrollerservice_deleteimpedancecontroller :name sensor-name)))
  (:wait-impedance-controller-transition (&rest args)
    (error ";; :wait-impedance-controller-transition cannot be used with hironx~%"))
  (:set-impedance-controller-param (&rest args)
    (error ";; In hironx, we cannot tell :set-impedance-controller-param from :start-impedance~%"))
  (:get-impedance-controller-param (limb)
    (let (sensor-name)
      (cond ((eq limb :rarm)
             (setq sensor-name "rhsensor"))
            ((eq limb :larm)
             (setq sensor-name "lhsensor"))
            ((eq limb :arms)
             (return-from :get-impedance-controller-param
                          (mapcar #'(lambda (l) (send self :get-impedance-controller-param l))
                                  '(:rarm :larm))))
            (t (error ";; No such limb: ~A~%." limb)))
      (send (send self :impedancecontrollerservice_getimpedancecontrollerparam :name sensor-name)
            :i_param)))
  (:get-impedance-controller-controller-mode (&rest args)
    (error ";; :get-impedance-controller-controller-mode cannot be used with hironx~%")))

;; ServoControllerService for hand
;; Based on https://github.com/start-jsk/rtmros_hironx/blob/2.1.0/hironx_ros_bridge/src/hironx_ros_bridge/hironx_client.py
;; and https://github.com/start-jsk/rtmros_tutorials/blob/0.1.6/hrpsys_ros_bridge_tutorials/euslisp/hrp2-common-interface.l
(defmethod hironxjsk-interface
  (:hand-angle-vector
   (hand &optional av (tm 1000))
   (if av
     ;; setjointangles
     (let ((av-rad-list (map cons #'deg2rad av)))
       (case hand
             (:hands (send self :call-operation-return :servocontrollerservice_setjointangles
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
    (send self :call-operation-return :servocontrollerservice_servoon))
  (:hand-servo-off ()
    (send self :call-operation-return :servocontrollerservice_servooff))
  (:hand-effort (&optional (hand :hands) effort)
    "effort is percentage"
    (let ((ids (case hand
                     (:hands (list 2 3 4 5 6 7 8 9))
                     (:rhand (list 2 3 4 5))
                     (:lhand (list 6 7 8 9))
                     (t (error ";; No such hand: ~A~%." hand)))))
      (if effort
        ;; setmaxtorque
        (mapcar
         #'(lambda (id) (send self :call-operation-return :servocontrollerservice_setmaxtorque :id id :percentage effort))
         ids)
        ;; getmaxtorque
        (mapcar
         #'(lambda (id) (send (send self :call-operation-return :servocontrollerservice_getmaxtorque :id id) :percentage))
         ids))))
  (:hand-width2angles (width)
    (let ((safetymargin 3) (w0 19) (l1 41.9))
      (unless (<= 0 width %(2 * (w0 + l1 - safetymargin)))
        (warn ";; width value ~a is off margins~%" width)
        (return-from :hand-width2angles nil))
      (let ((a (rad2deg %(pi/2 - acos((width / 2.0 + safetymargin - w0) / l1)))))
        (float-vector a (- a) (- a) a))))
  (:hand-angles2width (vec)
    (assert (= (length vec) 4))
    (let ((safetymargin 3) (w0 19) (l1 41.9) (l2 20))
      (flet ((get-width (r1 r2) %( w0 + l1 * cos(pi/2 - r1) + l2 * cos(pi/2 - r1 - r2) - safetymargin)))
            (multiple-value-bind (a1 a2 b1 b2) (map cons #'deg2rad vec)
              (+ (get-width a1 a2)
                 (get-width (- b1) (- b2)))))))
  (:hand-width (hand &optional width &key (tm 1000) effort)
    (if width
      ;; set hand width
      (progn
        (when effort (send self :hand-effort hand effort))
        (send self :hand-angle-vector hand (send self :hand-width2angles width) tm))
      ;; get hand width
      (send self :hand-angles2width (send self :hand-angle-vector hand))))
  (:start-grasp (&optional (arm :arms) &key effort)
    (cond ((eq arm :rarm)
           (send self :hand-width :rhand 0 :effort effort))
          ((eq arm :larm)
           (send self :hand-width :lhand 0 :effort effort))
          ((eq arm :arms)
           (send self :hand-width :rhand 0 :effort effort)
           (send self :hand-width :lhand 0 :effort effort))
          (t (error ";; No such arm: ~A~%." arm))))
  (:stop-grasp (&optional (arm :arms) &key effort)
    (cond ((eq arm :rarm)
           (send self :hand-width :rhand 100 :effort effort))
          ((eq arm :larm)
           (send self :hand-width :lhand 100 :effort effort))
          ((eq arm :arms)
           (send self :hand-width :rhand 100 :effort effort)
           (send self :hand-width :lhand 100 :effort effort))
          (t (error ";; No such arm: ~A~%." arm)))))

(defun hironxjsk-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* hironxjsk-interface :init args)))
  (if (not (boundp '*hironxjsk*))
      (setq *hironxjsk* (instance hironxjsk-robot :init)))
  ;; read initial robot state
  (send *hironxjsk* :angle-vector (send *ri* :state :potentio-vector))
  ;; return robot instance
  *hironxjsk*)
