#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

;; Wait until Ros bridges are prepared.
(ros::roseus "unittest")

;; Wait until odom is published
(warn ";; waiting for odom...~%")
(ros::load-ros-manifest "nav_msgs")
(setq *odom-published*  nil)
(defun odom-cb (msg) (setq *odom-published* t))
(ros::subscribe "/odom" nav_msgs::Odometry #'odom-cb)
(while (and (ros::ok) (not *odom-published*)) (ros::spin-once))
(warn ";; waiting for odom...done~%")

;; Load demo file which is specified next to this file in argument
(warn ";; loading test example file...~%")
(let ((name (elt lisp::*eustop-argument* (1+ (position-if #'(lambda (x) (substringp "samplerobot-unittest.l" x)) lisp::*eustop-argument*)))))
  (defvar *test-prefix* (pathname-name name))
  (load name))
(warn ";; loading test example file... done.~%")
;; wait-interpolation bug
(defmethod rtm-ros-robot-interface
  (:wait-interpolation (&rest args) (unix:sleep 10)))

;; Test init
(warn ";; initializing test...~%")
(let ((func (find-if #'(lambda (x) ;; find init function
                         (substringp (format nil "~A-init" *test-prefix*) (string-downcase x)))
                     (functions))))
 (eval `(deftest ,(read-from-string (format nil "test-~A" func))
          (assert (,func)))))
(warn ";; initializing test... done.~%")

;; Test demos
(dolist (funcs (remove-if-not #'(lambda (x) ;; find demo functions
                                  (and (substringp (format nil "~A-demo" *test-prefix*) (string-downcase x))
                                       (not (string= (format nil "~A-demo" *test-prefix*) (string-downcase x)))))
                              (functions)))
  (eval `(deftest ,(read-from-string (format nil "test-~A" funcs))
           (assert (,funcs)))))

(run-all-tests)
(exit 0)
