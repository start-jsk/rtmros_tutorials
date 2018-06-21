(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :hironx "package://hrpsys_ros_bridge_tutorials/models/hironx.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hironx-utils.l"))
  (require :hironx-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hironx-utils.l"))
(if (ros::resolve-ros-path "package://hironx_ros_bridge")
  (ros::load-ros-manifest "hironx_ros_bridge"))

(defclass hironx-interface
  :super rtm-ros-robot-interface
  :slots ())

;; Initialize
(defmethod hironx-interface
  (:init (&rest args)
         (send-super* :init :robot hironx-robot args))
  (:define-all-ROSBridge-srv-methods
    (&key (debug-view nil) (ros-pkg-name "hrpsys_ros_bridge"))
    ;; First, define ROSBridge method for old impedance controller
    (if (ros::resolve-ros-path "package://hironx_ros_bridge")
      (send-super :define-all-ROSBridge-srv-methods :ros-pkg-name "hironx_ros_bridge"))
    ;; Second, define ROSBridge method based on hrpsys_ros_bridge
    ;; Method created already is not overwritten, so we can keep using old impedance controller
    ;; See :get-ROSBridge-method-def-macro
    (send-super :define-all-ROSBridge-srv-methods)))

;; ImpedanceControllerService
;; Based on hironx_client.py in hironx_ros_bridge and rtm-ros-robot-interface.l.
;; Methods executable with old impedance controller.
;; The reason why I don't use def-set-get-param-method is that
;; OpenHRP_ImpedanceControllerService_setImpedanceControllerParam.srv has element "name" inside i_param,
;; while OpenHRP_ImpedanceControllerService_getImpedanceControllerParam.srv has that element directly.
;; Set optional-args as (list :name 'name) -> multiple declaration of variable "name" in set-param-method.
;; Set optional-args as nil -> pass nothing in service call of get-param-method
(defmethod hironx-interface
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
      (send self :impedancecontrollerservice_deleteimpedancecontroller :name sensor-name))))

(defun hironx-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* hironx-interface :init args)))
  (if (not (boundp '*hironx*))
      (setq *hironx* (instance hironx-robot :init))))
