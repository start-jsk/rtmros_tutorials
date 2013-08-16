#+:ros (require :unittest "lib/llib/unittest.l")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; setup functions for hrpsys, hrpsys_ros_bridge, and euslisp test
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(unless *unit-test*
  (init-unit-test))

(defun setup-robot (robot-name robot-model-pkg-path)
  (ros::roseus (format nil "~A_test_eus" robot-name))
  (load (format nil "~A/euslisp/~A-interface.l" robot-model-pkg-path robot-name))
  (funcall (read-from-string (format nil "~A-init" robot-name)))
  )

(defun setup-hrpsys-ros-test
  (robot-name
   &key (robot-model-pkg-name "hrpsys_ros_bridge_tutorials")
        (robot-model-pkg-path (ros::resolve-ros-path (format nil "package://~A" robot-model-pkg-name)))
        (use-deftest t))
  (rtshell-cmd-exec "rtdel -z")
  ;; define global variable to use deftest
  (defvar *robot-name* robot-name)
  ;; (defvar *robot-model-pkg-name* robot-model-pkg-name)
  (defvar *robot-model-pkg-path* robot-model-pkg-path)
  (if use-deftest
      (deftest test-robot-init
        (setup-robot *robot-name* *robot-model-pkg-path*)
        (assert t))
    (setup-robot *robot-name* *robot-model-pkg-path*))
  ;; fix order of test. check hrpsysseqstaterosbridge and simulator first.
  ;;   Note that (send *unit-test* :functions) is reverser of (*unit-test* . functions)
  (setq (*unit-test* . functions)
        (append (remove-if #'(lambda (x) (member (string-downcase x) (list "test-rosbridge-activated"
                                                                           "test-robot-init"
                                                                           "test-simulator"
                                                                           "test-hrpsysseqstaterosbridge"
                                                                           "test-tf-published")
                                                 :test #'string=))
                           (*unit-test* . functions))
                (list 'test-robot-init 'test-tf-published 'test-rosbridge-activated 'test-hrpsysseqstaterosbridge 'test-simulator)
                ))
  t)

(defun run-hrpsys-ros-bridge-test
  (robot-name
   &key (robot-model-pkg-name "hrpsys_ros_bridge_tutorials")
        (robot-model-pkg-path (ros::resolve-ros-path (format nil "package://~A" robot-model-pkg-name))))
  (setup-hrpsys-ros-test
   robot-name
   :robot-model-pkg-name robot-model-pkg-name
   :robot-model-pkg-path robot-model-pkg-path)
  (run-all-tests)
  (exit 0))

(defun hrpsys-ros-bridge-test-debug-mode
  (robot-name
   &key (robot-model-pkg-name "hrpsys_ros_bridge_tutorials")
        (robot-model-pkg-path (ros::resolve-ros-path (format nil "package://~A" robot-model-pkg-name))))
  (setup-hrpsys-ros-test
   robot-name
   :robot-model-pkg-name robot-model-pkg-name
   :robot-model-pkg-path robot-model-pkg-path
   :use-deftest nil)
  (setq lisp::*exit-on-fatal-error* nil)
  (warn ";; (run-all-tests)~%")
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; test functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; test for hrpsys-simulator and rosbridge using rtshell
(defun rtshell-cmd-exec (cmd)
  (let ((p (piped-fork (format nil "bash -c \". `rospack find openrtm_tools`/scripts/rtshell-setup.sh && ~A\"" cmd))))
    (prog1
        (let ((ln) (ret))
          (while (setq ln (read-line p nil nil)) (push ln ret))
          (reverse ret))
      (close p)
      )))

(defun wait-for-rtcs (cmd &key (timeout-count 5))
  (warn ";; wait for '~A'~%" cmd)
  (dotimes (i timeout-count)
    (if (rtshell-cmd-exec cmd)
        (return-from wait-for-rtcs t)
      (unix:sleep 5)
      ))
  (warn ";; '~A' not found~%" cmd)
  (exit 1)
  nil)

(defun wait-for-Simulator (&key (timeout-count 5))
  (wait-for-rtcs
   (format nil "rtls localhost | grep -v longfloor | grep '(Robot)0.rtc'")
   :timeout-count timeout-count))

(defun wait-for-HrpsysSeqStateROSBridge (&key (timeout-count 5))
  (wait-for-rtcs
   "rtcat localhost/HrpsysSeqStateROSBridge0.rtc"
   :timeout-count timeout-count))

(defun wait-for-ROSBridge-activated (&key (timeout-count 20))
  (mapcar #'(lambda (x)
              (wait-for-rtcs
               (format nil "rtcat localhost/~A | grep Active" (string-left-trim "0123456789 " x))
               :timeout-count timeout-count))
          (rtshell-cmd-exec "rtls localhost/ -l | grep ROSBridge | cut -d/ -f5")))

(defun wait-for-tf-published (&key (timeout-count 10))
  (and
   (wait-for-rtcs
    "rtcat -l localhost/HrpsysSeqStateROSBridge0.rtc:rsangle | grep Connected | grep '(Robot)0.rtc:q'"
    :timeout-count timeout-count)
   (wait-for-rtcs
    "rtcat -l localhost/HrpsysSeqStateROSBridge0.rtc:baseTform | grep Connected | grep 'sh.rtc:baseTformOut'"
    :timeout-count timeout-count)
   (wait-for-rtcs
    "rtcat -l localhost/HrpsysSeqStateROSBridge0.rtc:baseRpy | grep Connected | grep 'kf.rtc:rpy'"
    :timeout-count timeout-count)
   ))

(deftest test-Simulator
  (assert (wait-for-Simulator)))

(deftest test-HrpsysSeqStateROSBridge
  (assert (wait-for-HrpsysSeqStateROSBridge)))

(deftest test-ROSBridge-activated
  (assert (wait-for-ROSBridge-activated)))

(deftest test-tf-published
  (assert (wait-for-tf-published)))

;; test for :angle-vector
(defun tmp-test-seq-angle-vector
  (rob &optional (func))
  (mapcar #'(lambda (av)
              (let ((newav (send rob :angle-vector av)))
                (send *ri* :angle-vector (send rob :angle-vector) 1000)
                (send *ri* :wait-interpolation)
                (if func (setq newav (funcall func)))
                (eps= (distance (send *ri* :state :potentio-vector) newav) 0.0)))
          (list (instantiate float-vector (length (send rob :angle-vector)))
                (send rob :reset-pose)))
  )

;; test for force offset identification
(load "package://hrpsys_ros_bridge/euslisp/calib-force-sensor-params.l")
(defun tmp-test-force-offset-imp
  (robot calib-poses
   &key (test-poses
         (remove nil (list (send robot :reset-pose)
                           (if (find-method robot :reset-manip-pose) (send robot :reset-manip-pose)))))
        (D-p 25) (M-p 2.5)
        (K-r 1.0) (D-r 1) (M-r 0.1))
  (let ((thre 0.5)
        (rthre (deg2rad 0.5))
        (limbs '(:rarm :larm)))
    (mapcar #'(lambda (x)
                (send *ri* :stop-impedance x))
            limbs)
    (unix:system (format nil "rm -f /tmp/~A-force-moment-offset.l" (send robot :name)))
    (forceCalib-for-limbs robot
                          :fname (format nil "/tmp/~A-force-moment-offset.l" (send robot :name))
                          :poses calib-poses)
    (send *ri* :load-forcemoment-offset-param (format nil "/tmp/~A-force-moment-offset.l" (send robot :name)))
    ;;(progn (send *ri* :set-forcemoment-offset-param :rarm :force-offset #f(0 0 0) :moment-offset #f(0 0 0) :link-offset-mass 0 :link-offset-centroid #f(0 0 0)) (send *ri* :set-forcemoment-offset-param :larm :force-offset #f(0 0 0) :moment-offset #f(0 0 0) :link-offset-mass 0 :link-offset-centroid #f(0 0 0)))
    (apply
     #'append
     (mapcar #'(lambda (av)
                 (mapcar #'(lambda (x)
                             (send *ri* :stop-impedance x))
                         limbs)
                (send robot :angle-vector av)
                (send *ri* :angle-vector (send robot :angle-vector) 2000)
                (send *ri* :wait-interpolation)
                (let ((pc (send robot :copy-worldcoords))
                      (pec
                       (mapcar #'(lambda (x)
                                   (send (send (send robot :force-sensor x) :parent) :copy-worldcoords))
                               limbs)))
                  ;; check hand pos&rot error by ImpedanceController to validate force offset identification
                  (mapcar #'(lambda (x)
                              (send *ri* :start-impedance x
                                    :K-p 25 :D-p D-p :M-p M-p
                                    :K-r K-r :D-r D-r :M-r M-r
                                    :force-gain #f(1 1 1) :moment-gain #f(1 1 1)))
                          limbs)
                  (send *ri* :angle-vector (send robot :angle-vector) 5000) ;; for wait
                  (send *ri* :wait-interpolation) ;; for wait
                  (send robot :angle-vector (send *ri* :state :potentio-vector))
                  (send robot :newcoords pc)
                  (let ((ec
                         (mapcar #'(lambda (x c)
                                     (list
                                      (norm (send c :difference-position
                                                  (send (send (send robot :force-sensor x) :parent) :worldcoords)))
                                      (norm (send c :difference-rotation
                                                  (send (send (send robot :force-sensor x) :parent) :worldcoords)))))
                                 limbs pec)))
                    (warn ";; for pose ~A~%" (position av test-poses))
                    (mapcar #'(lambda (x)
                                (send *ri* :stop-impedance x))
                            limbs)
                    (mapcar #'(lambda (l x)
                                (warn ";;   ~A : ~A < ~A[mm], ~A < ~A[rad]~%" l (car x) thre (cadr x) rthre)
                                (and (< (car x) thre) (< (cadr x) rthre)))
                            limbs ec)
                    )))
            test-poses))))

;; test for hand pos&rot modification by ImpedanceController
(defun tmp-test-impedance-spring
  (rob hpos &key (base-time 1000) (arm :rarm))
  (let ((k-p 500) (d-p 50) (m-p 2.5)
        (k-r 5) (d-r 2) (m-r 0.005))
    (send *ri* :load-forcemoment-offset-param (format nil "/tmp/~A-force-moment-offset.l" (send rob :name)))
    (labels ((init-pose1
              ()
              (send rob :reset-pose))
             (init-pose2
              ()
              (send rob :reset-pose)
              (send rob arm :inverse-kinematics (make-coords :pos hpos)
                    :debug-view :no-message
                    )))
      (apply
       #'append
       (mapcar
        #'(lambda (pose-func)
            (funcall pose-func)
            (send *ri* :angle-vector (send rob :angle-vector) 1000)
            (send *ri* :wait-interpolation)
            (send *ri* :start-impedance arm
                  :ref-force (float-vector 0 0 0) :K-p k-p :M-p m-p :D-p d-p
                  :ref-moment (float-vector 0 0 0) :K-r k-r :M-r m-r :D-r d-r)
            (send *ri* :angle-vector (send rob :angle-vector) 2000)
            (send *ri* :wait-interpolation)
            (append
             (mapcar #'(lambda (force)
                         (send *ri* :start-impedance arm :ref-force (float-vector 0 0 0) :K-p k-p :M-p m-p :D-p d-p)
                         (send *ri* :angle-vector (send rob :angle-vector) (* 2 base-time))
                         (send *ri* :wait-interpolation)
                         (send *ri* :start-impedance arm :ref-force force :K-p k-p :M-p m-p :D-p d-p)
                         (send *ri* :angle-vector (send rob :angle-vector) (* 2 base-time))
                         (send *ri* :wait-interpolation)
                         (let ((pc (send (send (send rob :force-sensor arm) :parent) :copy-worldcoords)))
                           (send rob :angle-vector (send *ri* :state :potentio-vector))
                           (prog1
                               (distance
                                (v- (send (send (send rob :force-sensor arm) :parent) :worldpos) (send pc :worldpos))
                                (scale (/ -1e3 k-p) force))
                             (funcall pose-func)
                             )))
                     (list (float-vector 10 0 0) (float-vector -10 0 0)
                           (float-vector 0 10 0) (float-vector 0 -10 0)
                           (float-vector 0 0 10) (float-vector 0 0 -10)
                           ))
             nil
             (mapcar #'(lambda (moment)
                         (send *ri* :start-impedance arm :ref-moment (float-vector 0 0 0) :K-r k-r :M-r m-r :D-r d-r)
                         (send *ri* :angle-vector (send rob :angle-vector) (* 1.5 base-time))
                         (send *ri* :wait-interpolation)
                         (send *ri* :start-impedance arm :ref-moment moment :K-r k-r :M-r m-r :D-r d-r)
                         (send *ri* :angle-vector (send rob :angle-vector) (* 1.5 base-time))
                         (send *ri* :wait-interpolation)
                         (let ((pc (send (send (send rob :force-sensor arm) :parent) :copy-worldcoords)))
                           (send rob :angle-vector (send *ri* :state :potentio-vector))
                           (prog1
                               (rad2deg
                                (distance
                                 (send pc :rotate-vector
                                       (send pc :difference-rotation
                                             (send (send rob :force-sensor arm) :parent)))
                                 (scale (/ -1.0 k-r) moment)))
                             (funcall pose-func)
                             )))
                     (list (float-vector 1 0 0) (float-vector -1 0 0)
                           (float-vector 0 1 0) (float-vector 0 -1 0)
                           (float-vector 0 0 1) (float-vector 0 0 -1)))
             ))
        (list #'init-pose1 #'init-pose2)))
      )))

;; test for simple walking of AutoBalancer.rtc by checking whether the robot tumbled or not
(defun tmp-test-walk-simple
  (rob)
  (send rob :reset-pose)
  (send rob :fix-leg-to-coords (make-coords))
  (send *ri* :angle-vector (send rob :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *ri* :start-auto-balancer)
  (send *ri* :go-pos 0.3 0.1 5)
  (send *ri* :go-velocity 0 0 0)
  (send *ri* :go-stop)
  (send *ri* :stop-auto-balancer)
  (let ((ypr (car (send (send *ri* :state :imucoords) :rpy-angle))))
    (and (< (rad2deg (abs (elt ypr 1))) 10)
         (< (rad2deg (abs (elt ypr 2))) 10))
    ))

