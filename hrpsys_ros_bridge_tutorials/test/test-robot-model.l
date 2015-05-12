#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(deftest jaxon-robot-model
  (let (robot)
    (load "package://hrpsys_ros_bridge_tutorials/models/jaxon.l")
    (setq robot (instance jaxon-robot :init))
    (assert robot)
    ;;
    (assert (v= (send robot :reset-pose) (send robot :angle-vector)))

    (send robot :torso :waist-p :joint-angle 15)
    (assert (eps= (send robot :torso :waist-p :joint-angle) 15))

    (dolist (arm '(:larm :rarm))
      (send robot arm :wrist-p :joint-angle 10)
      (assert (eps= (send robot arm :wrist-p :joint-angle) 10)))

    (assert (null (send robot :self-collision-check)))
    ))

(run-all-tests)
(exit)
