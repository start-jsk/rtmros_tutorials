(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :hrp2w_mbzirc "package://hrpsys_ros_bridge_tutorials/models/hrp2w_mbzirc.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w_mbzirc-utils.l"))
  (require :hrp2w_mbzirc-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w_mbzirc-utils.l"))

(defclass hrp2w_mbzirc-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod hrp2w_mbzirc-interface
  (:init (&rest args)
    (prog1
        (send-super* :init :robot hrp2w_mbzirc-robot args)
      ;; add controller
      (when (find-method self :def-limb-controller-method)
        (dolist (limb '(:rarm :larm :head :torso))
          (send self :def-limb-controller-method limb)
          (send self :add-controller (read-from-string (format nil "~A-controller" limb)) :joint-enable-check t :create-actions t))))))

;; methods for grasp controller
(defmethod hrp2w_mbzirc-interface
  (:get-hand-config-list () (list (list :larm "LHAND" "LARM_JOINT7") (list :rarm "RHAND" "RARM_JOINT7")))
  (:start-grasp
   (limb &key (target-error 0.15) (gain-percentage 5))
   (send-super :start-grasp limb :target-error target-error :gain-percentage gain-percentage)
   )
  )

(defun hrp2w_mbzirc-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* hrp2w_mbzirc-interface :init args)))
  (if (not (boundp '*hrp2w_mbzirc*))
      (setq *hrp2w_mbzirc* (instance hrp2w_mbzirc-robot :init)))
  ;; (unless sim
  ;;   (if (not (boundp '*vmax*))
  ;;       (progn
  ;;         (require :vmax "package://vmax_controller/euslisp/vmax-interface.l")
  ;;         (setq *vmax* (instance vmax-interface :init)))))
  (setq ((send *hrp2w_mbzirc* :head :neck-p) . max-angle) 44)
  )
