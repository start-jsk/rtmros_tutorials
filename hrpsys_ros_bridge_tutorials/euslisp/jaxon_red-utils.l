(require :jaxon "package://hrpsys_ros_bridge_tutorials/models/jaxon_red.l")
(require :thkhand-utils "package://hrpsys_ros_bridge_tutorials/euslisp/thkhand-utils.l")

(defmethod JAXON_RED-robot
  (:init-ending
   (&rest args)
   (prog1
       (send-super* :init-ending args)
     (when (member :define-min-max-table (send self :methods))
       (send self :define-min-max-table))
     (send self :add-hoist-point-coords)
     (send self :add-shin-cushion-parts)
     (send self :add-shin-contact-coords)
     (send self :add-thk-contact-coords)
     (send self :add-wrist-contact-coords)
     (send self :add-hip-contact-coords)
     (send self :put :lhand-model (instance dummy-thk-hand-robot :init :name :l-thk-hand))
     (send self :put :rhand-model (instance dummy-thk-hand-robot :init :name :r-thk-hand))
     (when (< (norm (send (send (send self :worldcoords) :transformation
                                (send (car (send self :links)) :worldcoords)) :pos))
              0.1)
       ;; for new collada version
       (send self :dissoc (car (send self :links)))
       (send self :translate (float-vector 0 0 -1018.5)) ;; translation distance should be changed depend on WAIST Joint translation in .wrl
       (send self :assoc (car (send self :links)))
       (send self :reset-coords)
       )
     ))
  (:add-hoist-point-coords
   ()
   (let ((hoist-link (send self :link "CHEST_LINK2"))
         (tmp))
     (setq tmp
           (make-cascoords :init
                           :link-list
                           :parent hoist-link
                           :coords (send (send hoist-link :copy-worldcoords) :translate (float-vector 70 0 100) :world)
                           :name :hoist-coords))
     (send self :put :hoist-coords tmp)
     (send hoist-link :assoc (send self :get :hoist-coords)))
   )
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
   (&key (offset (float-vector (+ 15 3) 0 0)))
   (let* ((limb (list :rarm :larm))
          (name (list :rhand-contact-coords :lhand-contact-coords))
          tmpcec)
     (mapcar #'(lambda (l n sgn)
                 (setq tmpcec
                       (make-cascoords
                        :init :link-list
                        :parent (send self l :end-coords)
                        :coords (send (send (send (send self l :end-coords :copy-worldcoords)
                                                  :translate (cond ((eq l :rarm) (v- offset (float-vector 20 0 0)))
                                                                   ((eq l :larm) (v- offset (float-vector 5 0 0)))))
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
  (:add-hip-contact-coords
   (&key (offset (float-vector -252 0 -175))) ; calculated from CAD data
   (let* ((name :hip-contact-coords)
          tmpcec)
     (setq tmpcec
           (make-cascoords
            :init :link-list :parent (car (send self :links))
            :coords
            (send
             (make-coords
              :pos (send (send (car (send self :links)) :copy-worldcoords) :worldpos)
              :rot (send (send (car (send self :links)) :copy-worldcoords) :worldrot))
             :translate offset :local)
            :name name))
     (send self :put name tmpcec)
     (send (car (send self :links)) :assoc (send self :get name))
     ))
  (:reset-landing-pose
   (&key (default-pose (send self :reset-pose)))
   (send self :angle-vector default-pose)
   (send self :fix-leg-to-coords (make-coords))
   (let ((tc (list (send self :rleg :end-coords :copy-worldcoords)
                   (send self :lleg :end-coords :copy-worldcoords)
                   (send (send (send self :foot-midcoords) :copy-worldcoords)
                         :translate
                         (float-vector 0 0 (elt
                                            (send (send (send (send self :foot-midcoords) :copy-worldcoords)
                                                        :transformation (send (send self :get :hoist-coords) :copy-worldcoords) :local) :worldpos)
                                            2))
                         :local)))
         (ll (list (send self :rleg :end-coords)
                   (send self :lleg :end-coords)
                   (send self :get :hoist-coords))))
     (send self
           :fullbody-inverse-kinematics tc
           :move-target ll
           :link-list (mapcar #'(lambda (l) (send self :link-list (send l :parent))) ll)
           :translation-axis (list t t :z)
           :rotation-axis (list t t :y)
           :additional-weight-list (list (list (send self :rleg :knee-p :child-link) 0.1)
                                         (list (send self :lleg :knee-p :child-link) 0.1))
           :debug-view :no-message))
   )
  )

(eval
 `(defmethod jaxon_red-robot
    ,@(get-jaxon-with-thk-hand-class-methods)))
