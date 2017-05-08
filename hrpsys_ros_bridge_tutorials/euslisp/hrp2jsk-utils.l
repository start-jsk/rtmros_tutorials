(defmethod hrp2jsk-robot
  (:init-ending
   ()
   (prog1
       (send-super :init-ending)
     (send self :define-min-max-table)
     )))
