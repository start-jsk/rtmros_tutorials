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

    (dolist (j (list "RARM_JOINT6" "RARM_JOINT7" "LARM_JOINT6" "LARM_JOINT7" "CHEST_JOINT0" "CHEST_JOINT1" "HEAD_JOINT0" "HEAD_JOINT1"))
      (let ((diff
             (v- (matrix-row (send (send robot :joint j) :joint-min-max-table) 2)
                 (matrix-row (send (send robot :joint j) :joint-min-max-table) 1))))
        (assert (v> diff (fill (instantiate float-vector (length diff)) 0.0)))))
    ))

(run-all-tests)
(exit)
