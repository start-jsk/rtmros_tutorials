(load "package://hrpsys_ros_bridge_tutorials/euslisp/testmdofarm-interface.l")
(load "package://hrpsys_ros_bridge/euslisp/datalogger-log-parser.l")

(defun setup ()
  (if (not (boundp '*ri*))
      (testmdofarm-init))
  (if (not (boundp '*robot*))
      (setq *robot* *testmdofarm*))
  (send *robot* :init-pose)
  (send *robot* :newcoords (make-coords))
  (send *robot* :rarm :move-end-pos #f(100 100 -150) :world)
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *robot* :angle-vector) 100)
  (send *ri* :wait-interpolation))

(defun shake-imu (&key (init-ypr-angle (float-vector 0 0 0))
                       (deflection-ypr-angle (float-vector 0 0 0)) (wrt :world)
                       (divisor 10) (time 1500))
  ;; initial setting
  (let* ((_y (send (send *robot* :rarm :end-coords :copy-worldcoords) :rotate (elt init-ypr-angle 0) :z))
         (_p (send _y :rotate (elt init-ypr-angle 1) :y))
         (init-mat (send _p :rotate (elt init-ypr-angle 2) :x)))
    (send *robot* :rarm :inverse-kinematics init-mat)
    (send *robot* :newcoords (make-coords))
    (send *irtviewer* :draw-objects)
    (print "move initial pose")
    (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
    (send *ri* :wait-interpolation))
  ;; shake
  (let* ((start-coords (send *robot* :rarm :end-coords :copy-worldcoords))
         (dist-coords-A
          (send
           (send
            (send (copy-object start-coords) :rotate (elt deflection-ypr-angle 0) :z wrt)
            :rotate (elt deflection-ypr-angle 1) :y wrt)
           :rotate (elt deflection-ypr-angle 2) :x wrt))
         (dist-coords-B (midcoords -1 start-coords dist-coords-A))
         avs tms)
    (let (tc)
      (dotimes (i (* divisor 4))
        (format t "~A / ~A~%" (+ i 1) (* divisor 4))
        (cond
         ((< i divisor)
          (setq tc (midcoords (* (mod i divisor) (/ 1.0 divisor)) start-coords dist-coords-A)))
         ((< i (* divisor 2))
          (setq tc (midcoords (* (mod i divisor) (/ 1.0 divisor)) dist-coords-A start-coords)))
         ((< i (* divisor 3))
          (setq tc (midcoords (* (mod i divisor) (/ 1.0 divisor)) start-coords dist-coords-B)))
         ((< i (* divisor 4))
          (setq tc (midcoords (* (mod i divisor) (/ 1.0 divisor)) dist-coords-B start-coords))))
        (send *robot* :newcoords (make-coords))
        (send *robot* :rarm :inverse-kinematics tc)
        (send *irtviewer* :draw-objects)
        (unix:sleep (/ (/ time divisor) 1000))
        (push (send *robot* :angle-vector) avs)
        (push (/ time divisor) tms)))
    (print "finish to calculate avs")
    (list (reverse avs) (reverse tms))))

(defun take-log (avs tms &key (fname "/tmp/test-mdof-arm"))
  (send *ri* :start-log)
  (unix:sleep 1)                        ;wait for sensor value to converge
  (send *ri* :angle-vector-sequence avs tms)
  (send *ri* :wait-interpolation)
  (unix:sleep 1)                        ;wait for sensor value to converge
  (send *ri* :save-log fname))


(defun parse-log (&key (fname "/tmp/test-mdof-arm") (output "/tmp/kuroiwa.dat"))
  (setq *log* (instance datalogger-log-parser-controller :init fname :robot *robot*))
  (with-open-file
   (f (format nil output) :direction :output)
   (let (tm observed-rpy-kf observed-rpy-msg observed-rpy-imucoords true-rpy imu-msg start-tm acc gyro ef-pos)
     (do-until-key-with-check
      (progn
        (send *log* :state)
        (if (find-method *log* :eof-p)
            (not (send *log* :eof-p))))
      (unless start-tm
        (setq start-tm (+ (send (send *log* :time) :sec)
                          (* 1e-9 (send (send *log* :time) :nsec)))))
      (setq tm
            (- (+ (send (send *log* :time) :sec)
                  (* 1e-9 (send (send *log* :time) :nsec)))
               start-tm))
      (setq imu-msg (cdr (car (*log* . robot-state))))
      (setq observed-rpy-kf
            (reverse
             (mapcar #'(lambda (x) (rad2deg x))
                     (concatenate cons (send (elt (send *log* :parser-list) 4) :read-state)))))
      (setq observed-rpy-msg
            (mapcar #'(lambda (x) (rad2deg x))
                    (car
                     (rpy-angle
                      (ros::tf-quaternion->rot (send imu-msg :orientation))))))
      (setq observed-rpy-imucoords
            (car (rpy-angle (send (send *log* :imucoords) :worldrot))))
      (setq acc (send (elt (send *log* :parser-list) 0) :read-state))
      (setq gyro (send (elt (send *log* :parser-list) 2) :read-state))
      (send *robot* :angle-vector (send *log* :potentio-vector))
      (send *robot* :newcoords (make-coords))
      ;; (send *irtviewer* :draw-objects)
      (setq true-rpy
            (mapcar #'(lambda (x) (rad2deg x))
                    (car
                     (rpy-angle
                      (send (copy-object (car (send *robot* :imu-sensors))) :worldrot)))))
      (setq ef-pos (send (send *robot* :rarm :end-coords :copy-worldcoords) :worldpos))
      (format f "~A ~A ~A ~A ~A ~A ~A ~A ~A ~A ~A ~A ~A ~A ~A ~A ~A ~A ~A ~A ~A ~A~%"
              tm
              (elt observed-rpy-kf 0) (elt observed-rpy-kf 1) (elt observed-rpy-kf 2)
              (elt true-rpy 0) (elt true-rpy 1) (elt true-rpy 2)
              (elt observed-rpy-msg 0) (elt observed-rpy-msg 1) (elt observed-rpy-msg 2)
              (elt observed-rpy-imucoords 0) (elt observed-rpy-imucoords 1) (elt observed-rpy-imucoords 2)
              (elt acc 0) (elt acc 1) (elt acc 2)
              (elt gyro 0) (elt gyro 1) (elt gyro 2)
              (elt ef-pos 0) (elt ef-pos 1) (elt ef-pos 2)
              )))))

(warn "(main :deflection-ypr-angle (float-vector 0 (/ pi 6) 0) :wrt :world :init-ypr-angle (float-vector 0 (/ pi 2) 0))~%")
(defun main (&key (deflection-ypr-angle (float-vector 0 0.5 0))
                  (wrt :world)
                  (init-ypr-angle (float-vector 0.5 0 0))
                  (divisor 10)
                  (time 1500)
                  (fname "/tmp/test-mdof-arm"))
  (setup)
  (let ((avs-and-tms (shake-imu :init-ypr-angle init-ypr-angle
                                :deflection-ypr-angle deflection-ypr-angle :wrt wrt
                                :divisor divisor :time time)))
    (print 'start-take-log)
    (bench (take-log (car avs-and-tms) (cadr avs-and-tms) :fname fname)))
  (print 'start-parse-log)
  (bench (parse-log :fname fname))
  (print "plot...")
  (unix:system "./plot.py"))
