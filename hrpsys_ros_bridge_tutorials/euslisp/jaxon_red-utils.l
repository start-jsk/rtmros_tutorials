(require :jaxon "package://hrpsys_ros_bridge_tutorials/models/jaxon_red.l")

(defmethod JAXON_RED-robot
  (:init-ending
   (&rest args)
   (prog1
    (send-super* :init-ending args)
    (send self :add-shin-cushion-parts)
    (send self :add-shin-contact-coords)
    (send self :add-thk-contact-coords)
    (send self :add-wrist-contact-coords)
    ))
  (:add-shin-cushion-parts
   (&key (angle 6))
   (dolist (leg '(:rleg :lleg))
     (let ((b (make-cube 25 100 180)))
       (send b :put :face-color #f(1 0.6 0.4))
       (send b :newcoords
             (send (send (send (elt (send self leg :links) 3) :copy-worldcoords) :translate (float-vector 85 0 -150))
                   :rotate (deg2rad angle) :y))
       (send (elt (send self leg :links) 3) :assoc b)
       (setq ((elt (send self leg :links) 3) . geo::bodies)
             (append (send (elt (send self leg :links) 3) :bodies) (list b)))
       ))
   )
  (:add-shin-contact-coords
   (&key (offset (float-vector 98 0 -150)) (angle 6))
   (let* ((limb (list :rleg :lleg))
          (name (list :rleg-thin-contact-coords :lleg-thin-contact-coords))
          tmpcec)
     (mapcar #'(lambda (l n)
                 (setq tmpcec
                       (make-cascoords :init :link-list :parent (elt (send self l :links) 3)
                                       :coords
                                       (send
                                        (send
                                         (make-coords
                                          :pos (send (send (elt (send self l :links) 3) :copy-worldcoords) :worldpos)
                                          :rot (send (send (send self l :end-coords) :copy-worldcoords) :worldrot))
                                         :translate offset :local)
                                        :rotate (- (deg2rad angle) pi/2) :y :local)
                                       :name n))
                 (send self :put n tmpcec)
                 (send (elt (send self l :links) 3) :assoc (send self :get n)))
             limb name)))
  (:add-thk-contact-coords
   (&key (offset (float-vector 15 0 0)))
   (let* ((limb (list :rarm :larm))
          (name (list :rhand-contact-coords :lhand-contact-coords))
          tmpcec)
     (mapcar #'(lambda (l n sgn)
                 (setq tmpcec
                       (make-cascoords
                        :init :link-list
                        :parent (send self l :end-coords)
                        :coords (send (send (send (send self l :end-coords :copy-worldcoords)
                                                  :translate offset)
                                            :rotate -pi/2 :y)
                                      :rotate (* sgn -pi/2) :z)
                        :name n))
                 (send self :put n tmpcec)
                 (send (send self l :end-coords :parent) :assoc (send self :get n)))
             limb name (list +1 -1))))
  (:add-wrist-contact-coords
   (&key (offset (float-vector 0 70 0)))
   (let* ((limb (list :rarm :larm))
          (name (list :rarm-wrist-contact-coords :larm-wrist-contact-coords))
          tmpcec)
     (mapcar #'(lambda (l n sgn)
                 (setq tmpcec
                       (make-cascoords
                        :init :link-list :parent (elt (send self l :links) 6)
                        :coords
                        (send
                         (send
                          (make-coords
                           :pos (send (send (elt (send self l :links) 6) :copy-worldcoords) :worldpos)
                           :rot (send (send (send self l :end-coords) :copy-worldcoords) :worldrot))
                          :translate (scale sgn offset) :local)
                         :rotate (* sgn pi/2) :x :local)
                        :name n))
                 (send self :put n tmpcec)
                 (send (elt (send self l :links) 6) :assoc (send self :get n)))
             limb name (list +1 -1))))
  )
