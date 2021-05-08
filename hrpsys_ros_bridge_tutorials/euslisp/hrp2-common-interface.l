(defclass hrp2-common-interface
  :super rtm-ros-robot-interface
  :slots ()
  )

(defmethod hrp2-common-interface
  (:hand-angle-vector
   (av &optional (tm 1000))
   (let ((av-all (instantiate float-vector (length (send robot :angle-vector)))))
     (dotimes (i 6)
       (setelt av-all (position (elt (send robot :limb :rhand :joint-list) i) (send robot :joint-list)) (elt av i))
       (setelt av-all (position (elt (send robot :limb :lhand :joint-list) i) (send robot :joint-list)) (elt av (+ i 6))))
     (send self :angle-vector av-all tm :rhand-controller)
     (send self :angle-vector av-all tm :lhand-controller)
     av))
  (:hand-wait-interpolation
   ()
   (list (send self :wait-interpolation :rhand-controller)
         (send self :wait-interpolation :lhand-controller))
   )
  (:hand-servo-on
   ()
   (warn ";; :hrp3handcontrollerservice_handservoon is not implemented!~%")
   )
  (:hand-servo-off
   ()
   (warn ";; :hrp3handcontrollerservice_handservooff is not implemented!~%")
   )
  (:hand-joint-calib
   ()
   (warn ";; :hrp3handcontrollerservice_handjointcalib is not implemented!~%")
   )
  (:hand-state
   ()
   (send self :state)
   (let ((reference-all (send self :reference-vector))
         (reference (instantiate float-vector 12)))
     (dotimes (i 6)
       (setelt reference i (elt reference-all (position (elt (send robot :limb :rhand :joint-list) i) (send robot :joint-list))))
       (setelt reference (+ i 6) (elt reference-all (position (elt (send robot :limb :lhand :joint-list) i) (send robot :joint-list)))))
     (instance hrp3hand-state :init
               :reference reference
               :potentio (send robot :hand-angle-vector)
               )
     )
   )
  )

(defclass hrp3hand-state
  :super propertied-object
  :slots (_reference _potentio)
  )

(defmethod hrp3hand-state
  (:init
   (&key
    ((:reference __reference) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:potentio __potentio) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (setq _reference __reference)
   (setq _potentio __potentio)
   self)
  (:reference
   (&optional __reference)
   (if __reference (setq _reference __reference)) _reference)
  (:potentio
   (&optional __potentio)
   (if __potentio (setq _potentio __potentio)) _potentio)
  )
