(defclass hrp2-common-interface
  :super rtm-ros-robot-interface
  :slots ()
  )

(defmethod hrp2-common-interface
  (:hand-angle-vector
   (av &optional (tm 1000))
   (let ((av-all (instantiate float-vector (length (send robot :angle-vector)))))
     (dotimes (i 6)
       (setelt av-all (position (elt (send robot :rhand :joint-list) i) (send robot :joint-list)) (elt av i))
       (setelt av-all (position (elt (send robot :lhand :joint-list) i) (send robot :joint-list)) (elt av (+ i 6))))
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
       (setelt reference i (elt reference-all (position (elt (send robot :rhand :joint-list) i) (send robot :joint-list))))
       (setelt reference (+ i 6) (elt reference-all (position (elt (send robot :lhand :joint-list) i) (send robot :joint-list)))))
     (instance hrp3hand-state :init
               :reference reference
               :potentio (send robot :hand-angle-vector)
               )
     )
   )
  )

(defclass hrp3hand-state
  :super propertied-object
  :slots (_reference _potentio _error _sense _h815output _h815motorthermo _pgain _dgain _flimit _controlmode _curlim _fcthreshold _tactiles _thermo)
  )

(defmethod hrp3hand-state
  (:init
   (&key
    ((:reference __reference) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:potentio __potentio) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:error __error) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:sense __sense) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:h815output __h815output) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:h815motorthermo __h815motorthermo) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:pgain __pgain) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:dgain __dgain) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:flimit __flimit) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:controlmode __controlmode) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:curlim __curlim) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:fcthreshold __fcthreshold) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:tactiles __tactiles) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:thermo __thermo) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (setq _reference __reference)
   (setq _potentio __potentio)
   (setq _error __error)
   (setq _sense __sense)
   (setq _h815output __h815output)
   (setq _h815motorthermo __h815motorthermo)
   (setq _pgain __pgain)
   (setq _dgain __dgain)
   (setq _flimit __flimit)
   (setq _controlmode __controlmode)
   (setq _curlim __curlim)
   (setq _fcthreshold __fcthreshold)
   (setq _tactiles __tactiles)
   (setq _thermo __thermo)
   self)
  (:reference
   (&optional __reference)
   (if __reference (setq _reference __reference)) _reference)
  (:potentio
   (&optional __potentio)
   (if __potentio (setq _potentio __potentio)) _potentio)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
  (:sense
   (&optional __sense)
   (if __sense (setq _sense __sense)) _sense)
  (:h815output
   (&optional __h815output)
   (if __h815output (setq _h815output __h815output)) _h815output)
  (:h815motorthermo
   (&optional __h815motorthermo)
   (if __h815motorthermo (setq _h815motorthermo __h815motorthermo)) _h815motorthermo)
  (:pgain
   (&optional __pgain)
   (if __pgain (setq _pgain __pgain)) _pgain)
  (:dgain
   (&optional __dgain)
   (if __dgain (setq _dgain __dgain)) _dgain)
  (:flimit
   (&optional __flimit)
   (if __flimit (setq _flimit __flimit)) _flimit)
  (:controlmode
   (&optional __controlmode)
   (if __controlmode (setq _controlmode __controlmode)) _controlmode)
  (:curlim
   (&optional __curlim)
   (if __curlim (setq _curlim __curlim)) _curlim)
  (:fcthreshold
   (&optional __fcthreshold)
   (if __fcthreshold (setq _fcthreshold __fcthreshold)) _fcthreshold)
  (:tactiles
   (&optional __tactiles)
   (if __tactiles (setq _tactiles __tactiles)) _tactiles)
  (:thermo
   (&optional __thermo)
   (if __thermo (setq _thermo __thermo)) _thermo)
  )
