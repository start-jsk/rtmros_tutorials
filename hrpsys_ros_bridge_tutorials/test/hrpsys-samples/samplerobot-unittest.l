#!/usr/bin/env roseus

(ros::ros-warn ";; roseus unittest init~%")
(ros::ros-info ";; roseus unittest init~%")
(require :unittest "lib/llib/unittest.l")
(init-unit-test)
(ros::ros-warn ";; roseus unittest init2~%")
(ros::ros-info ";; roseus unittest init2~%")

;; Wait until Ros bridges are prepared.
(ros::roseus "unittest")
;;(unless (boundp '*tfl*)
;;  (defvar *tfl* (instance ros::transform-listener :init)))
;;(send *tfl* :wait-for-transform "WAIST_LINK0" "odom" (ros::time) 20.0)

;; Wait until odom is published
(warn ";; waiting for odom...~%")
(ros::ros-warn ";; waiting for odom...~%")
(ros::ros-info ";; waiting for odom...~%")
(ros::load-ros-manifest "nav_msgs")
(setq *odom-published*  nil)
(defun odom-cb (msg) (setq *odom-published* t))
(ros::subscribe "/odom" nav_msgs::Odometry #'odom-cb)
(while (and (ros::ok) (not *odom-published*)) (ros::spin-once))
(warn ";; waiting for odom...done~%")
;; (ros::ros-warn ";; waiting for odom...done~%")
;; (ros::ros-info ";; waiting for odom...done~%")

;; Load demo file which is specified next to this file in argument
(warn ";; loading...~%")
(let ((name (elt lisp::*eustop-argument* (1+ (position-if #'(lambda (x) (substringp "samplerobot-unittest.l" x)) lisp::*eustop-argument*)))))
  (defvar *test-prefix* (pathname-name name))
  (load name))
(warn ";; loading... done~%")

;; Test init
(warn ";; init...~%")
(let ((func (find-if #'(lambda (x) ;; find init function
                         (substringp (format nil "~A-init" *test-prefix*) (string-downcase x)))
                     (functions))))
 (eval `(deftest ,(read-from-string (format nil "test-~A" func))
          (assert (,func)))))
(warn ";; init...done~%")

;; Test demos
(warn ";; testing~%")
(dolist (funcs (remove-if-not #'(lambda (x) ;; find demo functions
                                  (and (substringp (format nil "~A-demo" *test-prefix*) (string-downcase x))
                                       (not (string= (format nil "~A-demo" *test-prefix*) (string-downcase x)))))
                              (functions)))
  (eval `(deftest ,(read-from-string (format nil "test-~A" funcs))
           (assert (,funcs)))))
(warn ";; testing...done~%")

(run-all-tests)
(exit 0)
