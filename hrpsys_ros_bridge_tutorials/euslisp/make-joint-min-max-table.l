
(defun gen-min-max-table-hrp2
  (robot margin)
  "Generate min-max table for hrp2 robots.
   robot is robot instance.
   margin is joint limit margin [deg]."
  (dolist (limb '(:rarm :larm))
    (if (send robot limb :wrist-r)
        (send robot :make-joint-min-max-table
              (send robot limb :wrist-r :parent-link)
              (send robot limb :wrist-p :child-link)
              (send robot limb :wrist-r)
              (send robot limb :wrist-p)
              :margin margin)))
  (dolist (limb '(:rleg :lleg))
    (if (send robot limb :crotch-r)
        (send robot :make-joint-min-max-table
              (send robot limb :crotch-r :parent-link)
              (send robot limb :crotch-p :child-link)
              (send robot limb :crotch-r)
              (send robot limb :crotch-p)
              :margin margin)))
  (if (and (send robot :rleg :crotch-y) (send robot :lleg :crotch-y))
      (send robot :make-joint-min-max-table
            (send robot :rleg :crotch-y :child-link)
            (send robot :lleg :crotch-y :child-link)
            (send robot :rleg :crotch-y)
            (send robot :lleg :crotch-y)
            :margin margin))
  (if (and (send robot :torso :waist-y) (send robot :torso :waist-p))
      (send robot :make-joint-min-max-table
            (send robot :torso :waist-y :parent-link)
            (send robot :torso :waist-p :child-link)
            (send robot :torso :waist-y)
            (send robot :torso :waist-p)
            :margin margin))
  )
(defun gen-min-max-table-jaxon
  (robot margin)
  "Generate min-max table for jaxon robots.
   robot is robot instance.
   margin is joint limit margin [deg]."
  (dolist (limb '(:rarm :larm))
    (send robot :make-joint-min-max-table
          (send robot limb :wrist-r :parent-link)
          (send robot limb :wrist-p :child-link)
          (send robot limb :wrist-r)
          (send robot limb :wrist-p)
          :margin margin))
  (if (and (send robot :torso :waist-r) (send robot :torso :waist-p))
      (send robot :make-joint-min-max-table
            (send robot :torso :waist-r :parent-link)
            (send robot :torso :waist-p :child-link)
            (send robot :torso :waist-r)
            (send robot :torso :waist-p)
            :margin 3))
  )
(defun gen-min-max-table-staro
  (robot margin)
  "Generate min-max table for staro robots.
   robot is robot instance.
   margin is joint limit margin [deg]."
  (send robot :make-joint-min-max-table
        (send robot :torso :waist-p :parent-link)
        (send robot :torso :waist-y :child-link)
        (send robot :torso :waist-p)
        (send robot :torso :waist-y)
        :margin margin)
  )

(defun write-min-max-table-to-robot-model-file
  (robot path &key (margin 1.0) (gen-min-max-table-func))
  "Generate and write min-max to robot model file.
   robot is robot instance.
   path is robot model file full path.
   margin is joint limit margin [deg].
   gen-min-max-table-func is function to generate min-max table for each robot."
  (let ((robot-name (send robot :name)))
    (cond ((substringp "hrp2" robot-name) (setq gen-min-max-table-func 'gen-min-max-table-hrp2))
          ((substringp "jaxon" robot-name) (setq gen-min-max-table-func 'gen-min-max-table-jaxon))
          ((substringp "staro" robot-name) (setq gen-min-max-table-func 'gen-min-max-table-staro))
          ))
  (when (remove-if-not #'(lambda (j) (send j :joint-min-max-table)) (send robot :joint-list))
    (warn ";; ~A already has min-max table.~%" (send robot :name))
    (return-from write-min-max-table-to-robot-model-file nil))
  ;; (gen-min-max-table robot (format nil "~Ayaml" (string-right-trim "l" path)))
  (funcall gen-min-max-table-func robot margin)
  (with-open-file
   (f path :direction :output :if-exists :append)
   (format f "(defmethod ~A~%" (send (class robot) :name))
   (format f "  (:init-ending~%")
   (format f "    ()~%")
   (format f "    (prog1~%")
   (format f "      (send-super :init-ending)~%")
   (labels
       ((gen-string-name
         (j)
         (if (stringp (send j :name)) (format nil "\"~A\"" (send j :name)) (send j :name))))
     (dolist (j (remove-if-not #'(lambda (j) (send j :joint-min-max-table)) (send robot :joint-list)))
       (format f "      (send (send self :joint ~A) :joint-min-max-table~%" (gen-string-name j))
       (format f "                              '~A~%" (send j :joint-min-max-table))
       (format f "                              )~%")
       (format f "      (send (send self :joint ~A) :joint-min-max-target (send self :joint ~A))~%"
               (gen-string-name j) (gen-string-name (send j :joint-min-max-target)))
       )
     (format f "      )))~%")
     ))
  t)

(defun write-min-max-table-to-conf-file
  (robot conf-file-path)
  (unless (car (piped-fork-returns-list (format nil "grep joint_limit_table ~A" conf-file-path)))
    (with-open-file
     (of conf-file-path :direction :output :if-exists :append)
     (let ((jmmtl (remove-if-not #'(lambda (x) (send x :joint-min-max-table)) (send robot :joint-list))))
       (when jmmtl
         (format of "~%~%# for joint limit table~%")
         (format of "joint_limit_table: ")
         (dolist (j jmmtl)
           (format of "~A:~A:" (send j :name) (send (send j :joint-min-max-target) :name)) ;; self_joint_name:target_joint_name
           (let* ((mmt (send j :joint-min-max-table))
                  (dims (array-dimensions mmt))
                  (str ""))
             (dotimes (i (car dims))
               (let ((str2 ""))
                 (if (= i 0)
                     (setq str2 (format nil "~A:~A" (round (aref mmt 0 0)) (round (aref mmt 0 (- (cadr (array-dimensions mmt)) 1))))) ;; target_min_angle:target_max_angle
                   (dotimes (j (cadr dims))
                     (setq str2 (if (= j 0) (format nil "~A" (round (aref mmt i j))) (format nil "~A,~A" str2 (round (aref mmt i j))))))) ;; min_table:max_table
                 (setq str (if (= i 0) (format nil "~A" str2) (format nil "~A:~A" str str2)))))
             (format of "~A:" str)
             ))))
     (format of "~%")
     )))

#|
;; How to check robot min-max table

(load "models/hrp2jsk.l")
(defmethod hrp2jsk-robot (:init-ending () (send-super :init-ending))) ;; Clear current min max table
(setq *robot* (hrp2jsk))
(send *robot* :make-joint-min-max-table
            (send *robot* :torso :waist-y :parent-link)
            (send *robot* :torso :waist-p :child-link)
            (send *robot* :torso :waist-y)
            (send *robot* :torso :waist-p)
            :margin 1.0)
(send *robot* :plot-joint-min-max-table (send *robot* :torso :waist-y) (send *robot* :torso :waist-p))
|#
