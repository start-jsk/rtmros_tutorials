(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")

(defclass jaxon-common-interface
  :super rtm-ros-robot-interface
  :slots (hand-enable)
  )

(defmethod jaxon-common-interface
  (:init
   (&rest args &key ((:robot rb)) ((:controller-timeout ct) nil))
   (prog1
       (send-super* :init :robot rb :controller-timeout ct args)
     ;; add controller
     (print ";; Add controllers")
     ;; add controller
     (dolist (limb '(:rarm :larm :rleg :lleg :head :torso))
       (send self :def-limb-controller-method limb)
       (send self :add-controller (read-from-string (format nil "~A-controller" limb)) :joint-enable-check t :create-actions t))
     ))
  (:move-gripper
   (&rest args)
   (if hand-enable
       (send* (send self :get :hand-controller) :move-gripper args)
     (warn ";; can not use hand~%")))
  (:start-grasp
   (&rest args)
   (if hand-enable
       (send* (send self :get :hand-controller) :start-grasp args)
     (warn ";; can not use hand~%")))
  (:stop-grasp
   (&rest args)
   (if hand-enable
       (send* (send self :get :hand-controller) :stop-grasp args)
     (warn ";; can not use hand~%")))
  (:hand-reset
   (&rest args)
   (if hand-enable
       (send* (send self :get :hand-controller) :reset args)
     (warn ";; can not use hand~%")))
  ;; (:hand-open
  ;;  (&rest args)
  ;;  (send* (send self :get :hand-controller) :open args))
  ;; (:hand-close
  ;;  (&rest args)
  ;;  (send* (send self :get :hand-controller) :close args))
  (:hand-stop
   (&rest args)
   (if hand-enable
       (send* (send self :get :hand-controller) :stop args)
     (warn ";; can not use hand~%")))
  (:hand-resume
   (&rest args)
   (if hand-enable
       (send* (send self :get :hand-controller) :resume args)
     (warn ";; can not use hand~%")))
  (:get-joint-angle
   (&rest args)
   (warning-message 1 ";;!!!!!!!!~%;; (send *ri* :get-joint-angle) METHOD IS DEPRECATED!!~%;; Please use (send *ri* :state :gripper :arms :angle-vector)~%;;!!!!!!!!~%")
   (if hand-enable
       (send* (send self :get :hand-controller) :get-joint-angle args)
     (warn ";; can not use hand~%")))
  (:get-joint-velocity
   (&rest args)
   (warning-message 1 ";;!!!!!!!!~%;; (send *ri* :get-joint-velocity) METHOD IS DEPRECATED!!~%;; Please use (send *ri* :state :gripper :arms :velocity-vector)~%;;!!!!!!!!~%")
   (if hand-enable
       (send* (send self :get :hand-controller) :get-joint-velocity args)
     (warn ";; can not use hand~%")))
  (:get-joint-effort
   (&rest args)
   (warning-message 1 ";;!!!!!!!!~%;; (send *ri* :get-joint-effort) METHOD IS DEPRECATED!!~%;; Please use (send *ri* :state :gripper :arms :effort-vector)~%;;!!!!!!!!~%")
   (if hand-enable
       (send* (send self :get :hand-controller) :get-joint-effort args)
     (warn ";; can not use hand~%")))
  (:gripper
   (arm method)
   (if (memq arm '(:rarm :larm :arms))
       (case
        method
        (:angle-vector
         (if hand-enable
             (send* (send self :get :hand-controller) :get-joint-angle (if (eq arm :arms) arm (list arm)))
           (warn ";; can not use hand~%")))
        (:velocity-vector
         (if hand-enable
             (send* (send self :get :hand-controller) :get-joint-velocity (if (eq arm :arms) arm (list arm)))
           (warn ";; can not use hand~%")))
        (:torque-vector
         (if hand-enable
             (send* (send self :get :hand-controller) :get-joint-effort (if (eq arm :arms) arm (list arm)))
           (warn ";; can not use hand~%")))
        (t )
        )
     (error ";; no such arm in :gripper ~A~%" arm))
   )
  (:state (&rest args) ;; overwrite for gripper
   (case
    (car args)
    (:gripper
     (send* self :gripper (cdr args)))
    (t
     (send-super* :state args))))
)
