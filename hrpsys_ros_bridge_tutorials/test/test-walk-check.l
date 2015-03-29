;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Test program for walking
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun test-walk-0 ()
  "Test program for 0 step."
  (send *ri* :go-pos 0 0 0)
  )

(defun test-walk-fwd-0 ()
  "Test program for 1[m] fwd walking by default stride."
  (send *ri* :go-pos 1 0 0)
  )

(defun test-walk-fwd-1 ()
  "Test program for 1[m] fwd walking by large stride."
  (send *ri* :set-gait-generator-param :stride-parameter #f(0.2 0.05 10.0 0.05))
  (send *ri* :go-pos 1 0 0)
  (send *ri* :set-gait-generator-param :stride-parameter #f(0.15 0.05 10.0 0.05))
  )

(defun test-walk-bwd ()
  "Test program for 0.4[m] bwd walking."
  (send *ri* :go-pos -0.4 0 0)
  )

(defun test-walk-right ()
  "Test program for right ward walking."
  (send *ri* :set-gait-generator-param :stride-parameter #f(0.15 0.1 10.0 0.05))
  (send *ri* :go-pos 0 -0.3 0)
  (send *ri* :set-gait-generator-param :stride-parameter #f(0.15 0.05 10.0 0.05))
  )

(defun test-walk-left ()
  "Test program for left ward walking."
  (send *ri* :set-gait-generator-param :stride-parameter #f(0.15 0.1 10.0 0.05))
  (send *ri* :go-pos 0 0.3 0)
  (send *ri* :set-gait-generator-param :stride-parameter #f(0.15 0.05 10.0 0.05))
  )

(defun test-walk-right-turn ()
  "Test program for right turn."
  (send *ri* :set-gait-generator-param :stride-parameter #f(0.15 0.05 15.0 0.05))
  (send *ri* :go-pos 0 0 -90)
  (send *ri* :set-gait-generator-param :stride-parameter #f(0.15 0.05 10.0 0.05))
  )

(defun test-walk-left-turn ()
  "Test program for left turn."
  (send *ri* :set-gait-generator-param :stride-parameter #f(0.15 0.05 15.0 0.05))
  (send *ri* :go-pos 0 0 90)
  (send *ri* :set-gait-generator-param :stride-parameter #f(0.15 0.05 10.0 0.05))
  )

(defun test-walk-left-fwd ()
  "Test program for left ward + left turn + fwd walking."
  (send *ri* :go-pos 0.4 0.1 20)
  )

(defun test-walk-right-fwd ()
  "Test program for right ward + right turn + fwd walking."
  (send *ri* :go-pos 0.4 -0.1 -20)
  )

(defun test-walk-all ()
  (if (y-or-n-p ";; check (test-walk-0)?")
      (test-walk-0))
  (if (y-or-n-p ";; check (test-walk-fwd-0)?")
      (test-walk-fwd-0))
  (if (y-or-n-p ";; check (test-walk-bwd)?")
      (test-walk-bwd))
  (if (y-or-n-p ";; check (test-walk-fwd-1)?")
      (test-walk-fwd-1))
  (if (y-or-n-p ";; check (test-walk-right)?")
      (test-walk-right))
  (if (y-or-n-p ";; check (test-walk-left)?")
      (test-walk-left))
  (if (y-or-n-p ";; check (test-walk-right-turn)?")
      (test-walk-right-turn))
  (if (y-or-n-p ";; check (test-walk-left-turn)?")
      (test-walk-left-turn))
  (if (y-or-n-p ";; check (test-walk-right-fwd)?")
      (test-walk-right-fwd))
  (if (y-or-n-p ";; check (test-walk-left-fwd)?")
      (test-walk-left-fwd))
  )

(warn ";; (test-walk-0)~%")
(warn ";; (test-walk-fwd-0)~%")
(warn ";; (test-walk-bwd)~%")
(warn ";; (test-walk-fwd-1)~%")
(warn ";; (test-walk-right)~%")
(warn ";; (test-walk-left)~%")
(warn ";; (test-walk-right-turn)~%")
(warn ";; (test-walk-left-turn)~%")
(warn ";; (test-walk-right-fwd)~%")
(warn ";; (test-walk-left-fwd)~%")
(warn ";; (test-walk-all) ;; test for all walking check~%")

