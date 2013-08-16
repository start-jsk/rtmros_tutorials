#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/test/hrpsys-ros-bridge-test-suits.l")

(deftest test-seq-angle-vector
  (assert (every #'identity (tmp-test-seq-angle-vector *sr*))))

(deftest test-force-offset-imp
 (assert (every #'identity (tmp-test-force-offset-imp *sr*
                                                      (make-default-ForceCalibPoses *sr*)
                                                      :K-r 5 :D-r 5 :M-r 2
                                                      ))))

;;(deftest test-impedance-spring
;;  (assert (every #'(lambda (x) (< (abs x) 1.0)) (tmp-test-impedance-spring *sr* (float-vector 300 -200 900)))))

(deftest test-walk-simple
  (assert (tmp-test-walk-simple *sr*)))

(warn ";; (run-hrpsys-ros-bridge-test \"samplerobot\" :robot-model-pkg-name \"hrpsys_ros_bridge_tutorials\") ;; run-all-tests~%")
(warn ";; (hrpsys-ros-bridge-test-debug-mode \"samplerobot\" :robot-model-pkg-name \"hrpsys_ros_bridge_tutorials\") ;; setup *ri* and robot without error exit~%")
