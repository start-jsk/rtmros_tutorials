(require :hrp4r "package://hrpsys_ros_bridge_tutorials/models/hrp4r.l")

(defmethod hrp4r-robot
  (:init-ending
   ()
   (prog1
       (send-super :init-ending)
     (when (member :define-min-max-table (send self :methods))
       (send self :define-min-max-table))
     (when (< (norm (send (send (send self :worldcoords) :transformation
                                (send (car (send self :links)) :worldcoords)) :pos))
              0.1)
       ;; for new collada version
       (send self :dissoc (car (send self :links)))
       (send self :translate (float-vector 0 0 -791)) ;; translation distance should be changed depend on WAIST Joint translation in .wrl
       (send self :assoc (car (send self :links)))
       (send self :reset-coords)
       )
     ;; set force-sensor from .conf
     (labels ((data-string-split ;; this function will be replaced by https://github.com/euslisp/EusLisp/issues/16
               (str separator)
               (let ((start-idx 0) (ret))
                 (dotimes (i (length str))
                   (when (= (elt str i) (elt separator 0))
                     (push (subseq str start-idx i) ret)
                     (setq start-idx (1+ i))))
                 (if (/= start-idx (length str)) (push (subseq str start-idx) ret))
                 (reverse ret)))
              (data-str->data-list
               (data-str)
               (mapcar #'(lambda (x) (read-from-string x))
                       (remove-if #'(lambda (x) (string= "" x)) (data-string-split data-str " ")))))
       (let* ((vfsensor-setting
               (car (piped-fork-returns-list (format nil "grep virtual_force_sensor ~A/models/HRP4R.conf" (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials")))))
              (vfsensor-setting-list
               (mapcar #'(lambda (x) (string-left-trim " " x)) (data-string-split (string-left-trim "virtual_force_sensor:" vfsensor-setting) ",")))
              (id-offset (length (send self :force-sensors))))
         (dotimes (i (/ (length vfsensor-setting-list) 10))
           (let* ((pl (send (send self :joint (elt vfsensor-setting-list (+ (* 10 i) 2))) :child-link))
                  (fs (make-cascoords :coords
                                      (send (send pl :copy-worldcoords)
                                            :transform
                                            (make-coords
                                             :pos (scale 1e3 (concatenate float-vector (mapcar #'read-from-string (subseq vfsensor-setting-list (+ (* i 10) 3) (+ (* i 10) 6)))))
                                             :rot (rotation-matrix (read-from-string (elt vfsensor-setting-list (+ (* 10 i) 9)))
                                                                   (concatenate float-vector (mapcar #'read-from-string (subseq vfsensor-setting-list (+ (* i 10) 6) (+ (* i 10) 9)))))))
                                      :name (elt vfsensor-setting-list (* 10 i)))))
             (send pl :assoc fs)
             (setq force-sensors (append force-sensors (list fs)))
             ))
         )))
   )
  )
