(load "package://hrpsys_ros_bridge_tutorials/models/hironxjsk.l")

(defmethod hironxjsk-robot
  (:init-ending
    (&rest args)
    (prog1
      (send-super* :init-ending args)
      (send self :add-hand-tip-coords)
      ;; number of servo motors in one hand
      (send self :put :hand-servo-num 4)))
  ;; Based on https://github.com/start-jsk/rtmros_tutorials/blob/34cd4d03556a6630c8e89058a24c96edd779edaa/hrpsys_ros_bridge_tutorials/euslisp/jaxon-utils.l#L75-L93
  (:add-hand-tip-coords
    (&key (offset (float-vector 0 0 -43)))
    (let* ((limb (list :rarm :larm))
           (name (list :rhand-tip-coords :lhand-tip-coords))
           tmpcec)
      (mapcar #'(lambda (l n)
                  (setq tmpcec
                        (make-cascoords
                          :init :link-list
                          :parent (send self l :end-coords)
                          :coords (send (send self l :end-coords :copy-worldcoords)
                                        :translate offset)
                          :name n))
                  (send self :put n tmpcec)
                  (send (send self l :end-coords :parent) :assoc (send self :get n)))
              limb name))))

;; Hand interface
;; Please synchronize with hironxjsk-interface
(defmethod hironxjsk-robot
  (:check-hand-vector-length (vec &optional (hand-num 1))
    ;; Ensure that `vec' is a 4 element vector for single hand or 8 element for double hand
    (let ((len (* hand-num (send self :get :hand-servo-num))))
      (assert (= (length vec) len)
              "[ERROR] Expecting vector of length ~a~%" len)))
  (:hand-angle-vector (hand &optional av)
    (when av
      ;; check type
      (case hand
        (:hands
          (if (= (length av) (send self :get :hand-servo-num))
            (setq av (concatenate float-vector av av)))
          (send self :check-hand-vector-length av 2))
        ((:rhand :lhand)
         (send self :check-hand-vector-length av))))

    (flet ((get-joint-list (hand)
             (let (acc)
               (dotimes (i 4) (push (read-from-string (format nil "~a_joint~a" hand i)) acc))
               (nreverse acc))))
      (let ((joint-list (case hand
                          (:hands (append (get-joint-list :rhand) (get-joint-list :lhand)))
                          ((:rhand :lhand) (get-joint-list hand))
                          (t (error ";; No such hand: ~A~%." hand)))))
        (return-from :hand-angle-vector
          (if av
            ;; setjointangles
            (map nil #'(lambda (joint angle) (send self joint :joint-angle angle))
                  joint-list av)
            ;; getjointangles
            (map float-vector #'(lambda (joint) (send self joint :joint-angle))
                 joint-list))))))
  (:hand-width2angles (width)
    ;; Calculates the hand angles to achieve a certain parallel aperture
    (let ((safetymargin 3) (w0 19) (l1 41.9))
      (unless (<= (- safetymargin) width %(2 * (w0 + l1 - safetymargin)))
        (warn ";; width value ~a is off margins~%" width)
        (return-from :hand-width2angles nil))
      (let ((a (rad2deg %(pi/2 - acos((width / 2.0 + safetymargin - w0) / l1)))))
        (float-vector a (- a) (- a) a))))
  (:hand-angles2width (vec)
    ;; Calculates the hand aperture given a certain angle vector
    (send self :check-hand-vector-length vec)
    (let ((safetymargin 3) (w0 19) (l1 41.9) (l2 20))
      (flet ((get-width (r1 r2) %( w0 + l1 * cos(pi/2 - r1) + l2 * cos(pi/2 - r1 - r2) - safetymargin)))
            (multiple-value-bind (a1 a2 b1 b2) (map cons #'deg2rad vec)
              (+ (get-width a1 a2)
                 (get-width (- b1) (- b2)))))))
  (:hand-width (hand &optional width)
    ;; Get/Set the hand width
    (if width
      ;; set hand width
      (progn
        (send self :hand-angle-vector hand (send self :hand-width2angles width)))
      ;; get hand width
      (send self :hand-angles2width (send self :hand-angle-vector hand))))
  (:start-grasp (&optional (arm :arms))
    (case arm
          (:arms (setq arm :hands))
          (:rarm (setq arm :rhand))
          (:larm (setq arm :lhand)))
    (send self :hand-width arm 0))
  (:stop-grasp (&optional (arm :arms))
    (case arm
          (:arms (setq arm :hands))
          (:rarm (setq arm :rhand))
          (:larm (setq arm :lhand)))
    (send self :hand-width arm 100)))

;; For proper :self-collision-check.
;; Overwrite https://github.com/euslisp/jskeus/blob/1.2.5/irteus/irtmodel.l#L2670-L2684
;; to exclude link pairs which always collide.
;; Same method as https://github.com/jsk-ros-pkg/jsk_robot/pull/1026.
(defmethod hironxjsk-robot
  (:collision-check-pairs
    (&key ((:links ls) (cons (car links) (all-child-links (car links)))))
    (let (pairs l neighbors
                (exclude-pairs
                  (list (cons CHEST_JOINT0_Link_lk HEAD_JOINT1_Link_lk))))
      (while (setq l (pop ls))
        (setq neighbors (remove nil
                                (append
                                  (send l :descendants)
                                  (send l :child-links)
                                  (list (send l :parent-link) (send l :parent)))))
        (dolist (l2 ls)
          (if (and (not (memq l2 neighbors))
                   (not (member (cons l l2) exclude-pairs :test #'equal)))
            (push (cons l l2) pairs))))
      pairs)))

;; additional robot model description from camera_info
;;
(defun HIRONXJSK () (setq *HIRONXJSK* (instance HIRONXJSK-sensor-robot :init)))
;;
(defclass HIRONXJSK-sensor-robot
  :super HIRONXJSK-robot
  :slots (head_camera-depth head_camera-rgb))
;;
(defmethod HIRONXJSK-sensor-robot
  (:init (&rest args)
    (send-super* :init args)
    ;; head_camera frame definition, this data is taken from hironx_tutorial/launch/hironxjsk_real.launch
    ;; define cameras
    ;; #<sensor_msgs::camerainfo #X7a782d0> #f(574.053 0.0 314.5 0.0 0.0 574.053 235.5 0.0 0.0 0.0 1.0 0.0)
    (setq head_camera-depth (make-camera-from-ros-camera-info-aux 640 480 #f(574.053 0.0 314.5 0.0 0.0 574.053 235.5 0.0 0.0 0.0 1.0 0.0) HEAD_CAMERA_DEPTH-sensor-coords :name :head_camera/depth))

    ;; #<sensor_msgs::camerainfo #X7ad2d48> #f(574.053 0.0 319.5 0.0 0.0 574.053 239.5 0.0 0.0 0.0 1.0 0.0)
    (setq head_camera-rgb (make-camera-from-ros-camera-info-aux 640 480 #f(574.053 0.0 319.5 0.0 0.0 574.053 239.5 0.0 0.0 0.0 1.0 0.0) HEAD_CAMERA_RGB-sensor-coords :name :head_camera/rgb))

    (setq cameras (append (list (send self :head_camera-depth) (send self :head_camera-rgb)) cameras))
    self)

  (:head_camera-depth (&rest args) (forward-message-to head_camera-depth args))
  (:head_camera-rgb (&rest args) (forward-message-to head_camera-rgb args)))
