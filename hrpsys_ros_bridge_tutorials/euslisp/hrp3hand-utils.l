(defun get-hrp2-with-hand-class-methods ()
 '(
  (:hand (arm &rest args)
   (case arm
     (:rarm (send* self :rhand args))
     (:larm (send* self :lhand args))
     ((:both :arms) (list (send* self :rhand args) (send* self :lhand args)))))
  (:limb
   (limb method &rest args)
   (if (member limb '(:rhand :lhand))
       (let ((av (case method
                   (:open-pose (float-vector  0 60 -10 30  0  0))
                   (:preclose-pose (float-vector  40  60 -10  35  10 90))
                   (:close-pose (float-vector  65   0  30  50  90 90))
                   (:reset-pose (float-vector  0 0 0 0 0 0))
                   ;;
                   (:standard-pose (float-vector  20 90 0 10 -20 -20))
                   (:hook-pose (float-vector  90 90 0 10 -20 -20))
                   (:index-pose (float-vector  60 90 0 70 -20 -20))
                   (:extension-pose (float-vector  90 30 0 10 -20 -20))
                   (:distal-pose (float-vector  50 60 -20 10 20 40))
                   (:hook-pose2 (float-vector  90 70 0 10 -20 -40))
                   (:distal-pose2 (float-vector  90 90 -20 10 20 60))
                   (:grasp-pose (float-vector 77.9709 -11.4732 8.28742 -16.3569 106.185 86.0974))
                   (:index-avoid-extention-pose2 (float-vector 90.0 -30.0 -10.0 10.0 -40.0 -40.0))
                   (:index-avoid-hook-pose (float-vector  90 90 -10 -10 -40 -20))
                   (:index-avoid-reset-pose (float-vector  0 0 -10 -10 -40 0))
                   (:index-avoid-hook-pose2 (float-vector  90 70 -10 -10 -40 -40))
                   (:index-avoid-distal-pose (float-vector  50 60 -10 -10 -40 40))
                   (:index-avoid-grasp-pose (float-vector 77.9709 -11.4732 -10 -10 -40 86.0974)))))
         (if av
             (send-super :limb limb :angle-vector av)
             (send-super* :limb limb method args)))
       (send-super* :limb limb method args)))
  (:hand-angle-vector
   (&optional (av))
   (when av
     (send self :rhand :angle-vector (subseq av 0 6))
     (send self :lhand :angle-vector (subseq av 6))
     )
   (concatenate float-vector
                (send self :rhand :angle-vector)
                (send self :lhand :angle-vector))
   )
  )
 )
