#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(setq *all-robots* '(hrp2jsk hrp2jsknt hrp2jsknts hrp2w hrp2g urataleg jaxon jaxon_red chidori ystleg hrp4r staro))

(deftest test-model-gen
  (dolist (rb *all-robots*)
    (load (format nil "package://hrpsys_ros_bridge_tutorials/euslisp/~A-interface.l" rb))
    (format t ";; ~A done~%" (send (funcall rb) :name))
    )
  (assert t))

(deftest test-joint-min-max-check
  (dolist (rb *all-robots*)
    (let ((check0 (not (not (find-method (eval (read-from-string (format nil "*~A*" rb))) :define-min-max-table))))
          (check1 (not (not (find-if #'(lambda (j) (send j :joint-min-max-table)) (send (eval (read-from-string (format nil "*~A*" rb))) :joint-list))))))
      (format t ";; ~10,A (has :define-min-max-table = ~3,A) (has min/max table = ~3,A) (total = ~3,A)~%"
              rb check0 check1 (eq check0 check1))
      (assert (eq check0 check1))
      )))

(run-all-tests)
(exit)
