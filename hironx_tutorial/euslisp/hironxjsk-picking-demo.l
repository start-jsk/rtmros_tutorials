#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")

(ros::load-ros-package "jsk_recognition_msgs")


(defun tf-pose->coords (frame_id pose)
  (send (send *tfl* :lookup-transform "WAIST" frame_id (ros::time 0))
        :transform (ros::tf-pose->coords pose)))

(defun bbox->cube (bbox)
  (let* ((dims (ros::tf-point->pos (send bbox :dimensions)))
         (bx (make-cube (elt dims 0) (elt dims 1) (elt dims 2))))
    (send bx :newcoords
          (tf-pose->coords (send bbox :header :frame_id) (send bbox :pose)))
    bx))

(defun demo ()
  (hironxjsk-init)
  (objects (list *hironxjsk*))

  ;; Look at table
  (send *hironxjsk* :reset-manip-pose)
  (send *hironxjsk* :head :look-at
        (send (send (send *hironxjsk* :torso :end-coords :copy-worldcoords)
                    :translate #f(750 0 0)) :worldpos))
  (send *irtviewer* :draw-objects)  ;; Only for display
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  ;; Get recognition result
  (ros::duration-sleep 1.0)
  (setq *bbox-msg*
        (one-shot-subscribe "/segmentation_decomposer/boxes"
                            jsk_recognition_msgs::BoundingBoxArray
                            :after-stamp (ros::time-now)))
  (setq *target-bbox* (elt (send *bbox-msg* :boxes) 0))
  (setq *target-cube* (bbox->cube *target-bbox*))
  (objects *target-cube*)

  ;; Move right hand above target
  (let (h-offset)
    (setq h-offset
          (+ 20
             (/ (z-of-cube *target-cube*) 2)
             (elt (v- (send *target-cube* :worldpos)
                      (send (send *hironxjsk* :get :rhand-tip-coords) :worldpos))
                  2)))
    (assert (send *hironxjsk* :rarm :move-end-pos (float-vector 0 0 h-offset) :world))
    ;; (assert X) raises error if X returns nil.
    ;; I use assert here to stop execution if IK fails
    (setq *mid-av* (send *hironxjsk* :angle-vector))
    (send *hironxjsk* :stop-grasp :rarm)
    (send *irtviewer* :draw-objects)  ;; Only for display
    (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
    (send *ri* :hand-angle-vector :rhand
          (send *hironxjsk* :hand-angle-vector :rhand) 1000)
    (send *ri* :wait-interpolation))
  (assert
    (send *hironxjsk* :rarm :inverse-kinematics
          (make-coords
            :pos (float-vector (elt (send *target-cube* :worldpos) 0)
                               (elt (send *target-cube* :worldpos) 1)
                               (elt (send (send *hironxjsk* :get :rhand-tip-coords)
                                          :worldpos)
                                    2)))
          :move-target (send *hironxjsk* :get :rhand-tip-coords)))
  (send *irtviewer* :draw-objects)  ;; Only for display
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  ;; Grasp target
  (assert
    (send *hironxjsk* :rarm :inverse-kinematics
          (make-coords :pos (send *target-cube* :worldpos))
          :move-target (send *hironxjsk* :get :rhand-tip-coords)))  ;; move-target for precision grip (pinching with fingertips)
                                                                    ;; If you want power grip, use default move-target
  (assert (send *hironxjsk* :rarm :move-end-pos #f(0 0 -10) :world))  ;; Add small offset for stable grasp
  (send *irtviewer* :draw-objects)  ;; Only for display
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (send *hironxjsk* :start-grasp :rarm)
  (send *irtviewer* :draw-objects)  ;; Only for display
  (send *ri* :hand-angle-vector :rhand
        (send *hironxjsk* :hand-angle-vector :rhand) 1000)
  (ros::duration-sleep 0.5)  ;; Don't wait completely to make grasp more stable.
                             ;; Try (ros::duration-sleep 1.0) to check what happens

  ;; Make grasp stable (this procedure is required due to multi-fingered hand)
  (let (real-av root-angle command-av)
    ;; Overwrite command of fingertip joints to fit fingertip to target
    (setq real-av (send *ri* :hand-angle-vector :rhand))
    (setq root-angle (/ (+ (aref real-av 0)
                           (- (aref real-av 2)))
                        2))
    (setq command-av (send *hironxjsk* :hand-angle-vector :rhand))
    (setf (aref command-av 1) (- root-angle))
    (setf (aref command-av 3) root-angle)
    (send *hironxjsk* :hand-angle-vector :rhand command-av)
    (send *ri* :hand-angle-vector :rhand
          (send *hironxjsk* :hand-angle-vector :rhand) 1000)
    (ros::duration-sleep 1.5))

  ;; Pick up & place target
  (assert (send *hironxjsk* :rarm :move-end-pos #f(0 0 100) :world))
  (send *irtviewer* :draw-objects)  ;; Only for display
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (let (avs)
    (push (send *hironxjsk* :rarm :move-end-pos #f(0 -100 0) :world) avs)
    (push (send *hironxjsk* :rarm :move-end-pos #f(0 0 -100) :world) avs)
    (nreverse avs)
    (assert (every #'vectorp avs))
    (send *irtviewer* :draw-objects)  ;; Only for display
    (send *ri* :angle-vector-sequence avs (list 1000 1000))
    (send *ri* :wait-interpolation))
  (send *hironxjsk* :stop-grasp :rarm)
  (send *irtviewer* :draw-objects)  ;; Only for display
  (send *ri* :hand-angle-vector :rhand
        (send *hironxjsk* :hand-angle-vector :rhand) 1000)
  (ros::duration-sleep 2.0)
  (assert (send *hironxjsk* :rarm :move-end-pos #f(0 0 100) :world))
  (send *irtviewer* :draw-objects)  ;; Only for display
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  ;; Go back to reset-manip-pose
  (let (avs)
    (push (send *hironxjsk* :angle-vector *mid-av*) avs)
    (push (send *hironxjsk* :reset-manip-pose) avs)
    (nreverse avs)
    (send *irtviewer* :draw-objects)  ;; Only for display
    (send *ri* :angle-vector-sequence avs (list 1000 1000))
    (send *ri* :wait-interpolation)))

(demo)
