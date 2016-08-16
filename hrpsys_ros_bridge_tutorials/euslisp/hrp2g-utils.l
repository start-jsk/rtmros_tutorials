;; stop and start grasp
(defun hrp2g-stop-grasp (limb)
  (when (boundp '*hrp2g*)
    (case limb
	  (:larm
	   (send *hrp2g* :larm :thumb-r :joint-angle -45.0)
	   )
	  (:rarm
	   (send *hrp2g* :rarm :thumb-r :joint-angle 45.0)
	   ))
    (when (boundp '*ri*)
      (send *ri* :angle-vector (send *hrp2g* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      (send *ri* :stop-grasp limb)
      )
    )
  )

(defun hrp2g-start-grasp (limb &key
			       (angle 0.0)
			       (target-error 0.15)
			       (gain-percentage 5))
  (when (boundp '*hrp2g*)
    (case limb
	  (:larm
	   (send *hrp2g* :larm :thumb-r :joint-angle angle)
	   )
	  (:rarm
	   (send *hrp2g* :rarm :thumb-r :joint-angle angle)
	   ))
    (when (boundp '*ri*)
      (send *ri* :start-grasp limb
	    :target-error target-error
	    :gain-percentage gain-percentage)
      (send *ri* :angle-vector (send *hrp2g* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    )
  )
