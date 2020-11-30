#!/usr/bin/env roseus

;; demo on euslisp

(load "package://nextage_tutorials/nextage.l")

(setq *nextage* (nextageopen))
(send *nextage* :reset-pose)
(objects (list *nextage*))

(send *nextage* :head :neck-p :joint-angle 25)
(send *nextage* :head :neck-y :joint-angle 45)
(send *nextage* :torso :waist-y :joint-angle 30)

(send *nextage* :larm :move-end-pos #f(0 0 200) :world :debug-view :no-message)
(send *nextage* :rarm :move-end-pos #f(0 -200 0) :world :debug-view :no-message)

;; demo on gazebo
(load "package://nextage_tutorials/euslisp/nextage-interface.l")

(nextage-init)
(send *ri* :angle-vector (send *nextage* :angle-vector) 1000) ;; caution !!! it could be too fast!!!!
(send *ri* :wait-interpolation)

(send *nextage* :reset-pose)
(send *nextage* :head :neck-p :joint-angle 25)
(send *ri* :angle-vector (send *nextage* :angle-vector) 500) ;; caution !!! it could be too fast!!!!
(send *ri* :wait-interpolation)
