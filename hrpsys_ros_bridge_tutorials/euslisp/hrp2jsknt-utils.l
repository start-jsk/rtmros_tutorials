(load "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknt.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp3hand-utils.l")

(unless (assoc :init-org (send hrp2jsknt-robot :methods))
  (rplaca (assoc :init (send hrp2jsknt-robot :methods)) :init-org))
(eval
 `(defmethod hrp2jsknt-robot
    ,@(get-hrp2-with-hand-class-methods)))

(defmethod hrp2jsknt-robot
  (:init-ending
   (&rest args)
   (prog1
       (send-super* :init-ending args)
     (send self :set-additional-end-coords)
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
  (:inverse-kinematics
   (target-coords
    &rest args
    &key (additional-weight-list (mapcar #'(lambda (l) (list (send self l :toe-p :child-link) 0)) '(:rleg :lleg)))
    &allow-other-keys)
   (send-super* :inverse-kinematics target-coords :additional-weight-list additional-weight-list args))
  (:fullbody-inverse-kinematics
   (target-coords
    &rest args
    &key (additional-weight-list (mapcar #'(lambda (l) (list (send self l :toe-p :child-link) 0)) '(:rleg :lleg)))
    &allow-other-keys)
   (send-super* :fullbody-inverse-kinematics target-coords :additional-weight-list additional-weight-list args))
  (:set-additional-end-coords
   ()
   ;; toe
   (send self :put :lleg-toe-end-coords
         (make-cascoords
          :name :lleg-toe-end-coords
          :coords (send (send (send self :lleg :toe-p) :child-link) :worldcoords)
          :parent (send (send self :lleg :toe-p) :child-link)))
   (send self :put :rleg-toe-end-coords
         (make-cascoords
          :name :rleg-toe-end-coords
          :coords (send (send (send self :rleg :toe-p) :child-link) :worldcoords)
          :parent (send (send self :rleg :toe-p) :child-link)))
   ;; toe tip
   (send self :put :lleg-toe-end-coords2
         (make-cascoords
          :name :lleg-toe-end-coords
          :coords (send (send (send (send (send self :lleg :toe-p) :child-link) :copy-worldcoords) :translate #f(50 0 0)) :rotate -pi/2 :y)
          :parent (send (send self :lleg :toe-p) :child-link)))
   (send self :put :rleg-toe-end-coords2
         (make-cascoords
          :name :rleg-toe-end-coords
          :coords (send (send (send (send (send self :rleg :toe-p) :child-link) :copy-worldcoords) :translate #f(50 0 0)) :rotate -pi/2 :y)
          :parent (send (send self :rleg :toe-p) :child-link)))
   ;; knee
   (send self :put :lleg-knee-end-coords
         (make-cascoords
          :name :lleg-knee-end-coords
          :coords (send (send (send self :lleg :knee-p :child-link) :copy-worldcoords) :translate #f(100 0 0) :local)
          :parent (send self :lleg :knee-p :child-link)))
   (send self :put :rleg-knee-end-coords
         (make-cascoords
          :name :rleg-knee-end-coords
          :coords (send (send (send self :rleg :knee-p :child-link) :copy-worldcoords) :translate #f(100 0 0) :local)
          :parent (send self :rleg :knee-p :child-link)))
   ;; crotch
   (send self :put :lleg-crotch-end-coords
         (make-cascoords
          :name :lleg-crotch-end-coords
          :coords (send (send (send self :lleg :crotch-p :child-link) :copy-worldcoords) :translate #f(0 0 0) :local)
          :parent (send self :lleg :crotch-p :child-link)))
   (send self :put :rleg-crotch-end-coords
         (make-cascoords
          :name :rleg-crotch-end-coords
          :coords (send (send (send self :rleg :crotch-p :child-link) :copy-worldcoords) :translate #f(0 0 0) :local)
          :parent (send self :rleg :crotch-p :child-link)))
   ;; wrist
   (send self :put :larm-wrist-end-coords
         (make-cascoords
          :name :larm-wrist-end-coords
          :coords (send (send (send (send (send (send (send self :larm :end-coords) :parent) :copy-worldcoords) :translate #f(0 50 0) :local) :rotate pi/2 :y :local) :rotate pi/2 :x :local) :rotate (deg2rad 15) :y :local)
          :parent (send (send self :larm :end-coords) :parent)))
   (send self :put :rarm-wrist-end-coords
         (make-cascoords
          :name :rarm-wrist-end-coords
          :coords (send (send (send (send (send (send (send self :rarm :end-coords) :parent) :copy-worldcoords) :translate #f(0 -50 0) :local) :rotate pi/2 :y :local) :rotate -pi/2 :x :local) :rotate (deg2rad 15) :y :local)
          :parent (send (send self :rarm :end-coords) :parent)))
   ;; wrist2
   (send self :put :larm-wrist-end-coords2
         (make-cascoords
          :name :larm-wrist-end-coords2
          :coords (send (send (send (send (send (send self :larm :end-coords) :parent) :parent) :copy-worldcoords) :rotate 0 :y :local) :translate #f(0 0 -50) :local)
          :parent (send (send (send self :larm :end-coords) :parent) :parent)))
   (send self :put :rarm-wrist-end-coords2
         (make-cascoords
          :name :rarm-wrist-end-coords2
          :coords (send (send (send (send (send (send self :rarm :end-coords) :parent) :parent) :copy-worldcoords) :rotate 0 :y :local) :translate #f(0 0 -50) :local)
          :parent (send (send (send self :rarm :end-coords) :parent) :parent)))
   ;; wrist3
   (send self :put :larm-wrist-end-coords3
         (make-cascoords
          :name :larm-wrist-end-coords2
          :coords (send (send (send (send (send (send self :larm :end-coords) :parent) :parent) :copy-worldcoords) :rotate pi/2 :y :local) :translate #f(0 0 -50) :local)
          :parent (send (send (send self :larm :end-coords) :parent) :parent)))
   (send self :put :rarm-wrist-end-coords3
         (make-cascoords
          :name :rarm-wrist-end-coords3
          :coords (send (send (send (send (send (send self :rarm :end-coords) :parent) :parent) :copy-worldcoords) :rotate pi/2 :y :local) :translate #f(0 0 -50) :local)
          :parent (send (send (send self :rarm :end-coords) :parent) :parent)))
   ;; elbow
   (send self :put :larm-elbow-end-coords
         (make-cascoords
          :name :larm-elbow-end-coords
          :coords (send (send (send (send self :larm :elbow-p :child-link) :copy-worldcoords) :rotate pi/2 :y :local) :translate #f(0 0 -30) :local)
          :parent (send self :larm :elbow-p :child-link)))
   (send self :put :rarm-elbow-end-coords
         (make-cascoords
          :name :rarm-elbow-end-coords
          :coords (send (send (send (send self :rarm :elbow-p :child-link) :copy-worldcoords) :rotate pi/2 :y :local) :translate #f(0 0 -30) :local)
          :parent (send self :rarm :elbow-p :child-link)))
   ;; head
   (send self :put :head-end-coords
         (make-cascoords
          :name :head-end-coords
          :coords (send (send (cadr (send self :head)) :copy-worldcoords) :translate #f(0 0 215) :local)
          :parent (cadr (send self :head))))
   )
  )
