#|
(defun gen-min-max-table (robot yaml-fname)
  (let ((config-len (read-from-string (car (piped-fork-returns-list (format nil "env python -c 'import yaml;print len(yaml.load(open(\"~A\").read()).get(\"min-max-table-config\"))'" yaml-fname))))))
    (dotimes (i config-len)
      (let ((parent-joint
             (car (piped-fork-returns-list (format nil "env python -c 'import yaml;print yaml.load(open(\"~A\").read()).get(\"min-max-table-config\")[~A][0]'" yaml-fname i))))
            (child-joint
             (car (piped-fork-returns-list (format nil "env python -c 'import yaml;print yaml.load(open(\"~A\").read()).get(\"min-max-table-config\")[~A][1]'" yaml-fname i))))
            (margin
             (read-from-string (car (piped-fork-returns-list (format nil "env python -c 'import yaml;print yaml.load(open(\"~A\").read()).get(\"min-max-table-config\")[~A][2]'" yaml-fname i))))))
        (send robot :make-joint-min-max-table
              (send (send robot :joint parent-joint) :parent-link)
              (send (send robot :joint child-joint) :child-link)
              (send robot :joint parent-joint)
              (send robot :joint child-joint)
              :margin margin)
        ))
    ))
|#

(defun gen-min-max-table (robot margin)
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
  (if (and (send robot :torso :waist-y) (send robot :torso :waist-p))
      (send robot :make-joint-min-max-table
            (send robot :torso :waist-y :parent-link)
            (send robot :torso :waist-p :child-link)
            (send robot :torso :waist-y)
            (send robot :torso :waist-p)
            :margin margin))
  )

;;(defun write-min-max-table (robot path &key (margin 0.0))
(defun write-min-max-table (robot path &key (margin 1.0))
  (unless (find-method robot :make-joint-min-max-table)
    (return-from write-min-max-table nil))
  (if (remove-if-not #'(lambda (j) (send j :joint-min-max-table)) (send robot :joint-list))
      (return-from write-min-max-table nil))
  ;; (gen-min-max-table robot (format nil "~Ayaml" (string-right-trim "l" path)))
  (gen-min-max-table robot margin)
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
  )
