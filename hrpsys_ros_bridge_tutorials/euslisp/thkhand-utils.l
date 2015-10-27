;; Dummy thk hand model for joint
(defclass dummy-thk-hand-robot
  :super robot-model
  :slots ()
  )
(defmethod dummy-thk-hand-robot
  (:init
   (&key (name))
   (prog1
       (send-super :init :name name)
     (setq links (list (instance bodyset-link :init (make-cascoords) :name :root-link)
                       (instance bodyset-link :init (make-cascoords) :name :gripper-link)))
     (send self :assoc (car links))
     (send (car links) :assoc (cadr links))
     (setq joint-list (list (instance rotational-joint :init :name :hand-joint
                                      :parent-link (car links) :child-link (cadr links)
                                      :min 0.0 :max 1.0)))
     )
   )
  )

(defun get-jaxon-with-thk-hand-class-methods ()
 '(
  (:hand (arm &rest args)
   (let (hr ret)
     (case arm
       (:larm
        (setq hr (send self :get :lhand-model))
        (setq ret (forward-message-to hr args)))
       (:rarm
        (setq hr (send self :get :rhand-model))
        (setq ret (forward-message-to hr args)))
       ((:both :arms)
        (setq hr (send self :get :lhand-model))
        (push (forward-message-to hr args) ret)
        (setq hr (send self :get :rhand-model))
        (push (forward-message-to hr args) ret))
       )
     ret))
  )
 )
