#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/test/hrpsys-ros-bridge-test-suits.l")

(deftest test-seq-angle-vector
  (assert (every #'identity (tmp-test-seq-angle-vector *hiro*))))

(warn ";; (run-hrpsys-ros-bridge-test \"kawada-hironx\" :robot-model-pkg-name \"hrpsys_ros_bridge_tutorials\") ;; run-all-tests~%")
(warn ";; (hrpsys-ros-bridge-test-debug-mode \"kawada-hironx\" :robot-model-pkg-name \"hrpsys_ros_bridge_tutorials\") ;; setup *ri* and robot without error exit~%")
