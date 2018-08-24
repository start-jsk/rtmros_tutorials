(defmethod hrp2jsk-robot
  (:init-ending
   (&rest args)
   (prog1
       (send-super* :init-ending args)
     (send self :define-min-max-table)
     (when (< (norm (send (send (send self :worldcoords) :transformation
                                (send (car (send self :links)) :worldcoords)) :pos))
              0.1)
       ;; for new collada version
       (send self :dissoc (car (send self :links)))
       (send self :translate (float-vector 0 0 -705)) ;; translation distance should be changed depend on WAIST Joint translation in .wrl
       (send self :assoc (car (send self :links)))
       (send self :reset-coords)
       )
     ))
  )
