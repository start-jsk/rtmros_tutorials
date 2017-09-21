(defun min-max-table-error (code msg1 form &optional (msg2))
   (format *error-output* "~C[1;3~Cm~A FATAL ERROR on make-joint-min-max-table: ~A"
                #x1b (+ 1 48)   *program-name* msg1)
   (if msg2 (format *error-output* " ~A" msg2))
   (if form (format *error-output* " in ~s" form))
   (format *error-output* ", exitting...~C[0m~%"  #x1b)
   (format *error-output* "~C[1;3~CmPlease comment in (load \"irteus/irtrobot.l\") in make-joint-min-max-table.l for debug~C[0m~%" #x1b (+ 2 48)  #x1b)
   (exit 1))

;; (load "irteus/irtrobot.l")
(setq lisp::*error-handler* 'min-max-table-error)



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
  ;; Hand made ankle-r ankle-p min-max table measured by real robot encoder values.
  ;;  This is for kawasaki-san ankle spacer.
  (when nil
    ;; set org-min-angle and org-max-angle for debug
    (send (send robot :joint "RLEG_JOINT5") :put :org-min-angle -29.0)
    (send (send robot :joint "RLEG_JOINT5") :put :org-max-angle 45.0)
    (send (send robot :joint "LLEG_JOINT5") :put :org-min-angle -45.0)
    (send (send robot :joint "LLEG_JOINT5") :put :org-max-angle 29.0)
    (send (send robot :joint "RLEG_JOINT4") :put :org-min-angle -74.0)
    (send (send robot :joint "RLEG_JOINT4") :put :org-max-angle 42.0)
    (send (send robot :joint "LLEG_JOINT4") :put :org-min-angle -74.0)
    (send (send robot :joint "LLEG_JOINT4") :put :org-max-angle 42.0)
    ;; set joint mm tables
    (send (send robot :joint "RLEG_JOINT5") :joint-min-max-table
          '#2f((-74.0 -73.0 -72.0 -71.0 -70.0 -69.0 -68.0 -67.0 -66.0 -65.0 -64.0 -63.0 -62.0 -61.0 -60.0 -59.0 -58.0 -57.0 -56.0 -55.0 -54.0 -53.0 -52.0 -51.0 -50.0 -49.0 -48.0 -47.0 -46.0 -45.0 -44.0 -43.0 -42.0 -41.0 -40.0 -39.0 -38.0 -37.0 -36.0 -35.0 -34.0 -33.0 -32.0 -31.0 -30.0 -29.0 -28.0 -27.0 -26.0 -25.0 -24.0 -23.0 -22.0 -21.0 -20.0 -19.0 -18.0 -17.0 -16.0 -15.0 -14.0 -13.0 -12.0 -11.0 -10.0 -9.0 -8.0 -7.0 -6.0 -5.0 -4.0 -3.0 -2.0 -1.0 0.0 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 11.0 12.0 13.0 14.0 15.0 16.0 17.0 18.0 19.0 20.0 21.0 22.0 23.0 24.0 25.0 26.0 27.0 28.0 29.0 30.0 31.0 32.0 33.0 34.0 35.0 36.0 37.0 38.0 39.0)
               (-19.0 -19.0 -20.0 -21.0 -21.0 -22.0 -23.0 -23.0 -24.0 -24.0 -25.0 -25.0 -25.0 -26.0 -26.0 -27.0 -27.0 -27.0 -27.0 -27.0 -28.0 -28.0 -28.0 -28.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0)
               (22.0 22.0 23.0 23.0 24.0 24.0 25.0 25.0 26.0 27.0 28.0 28.0 29.0 29.0 30.0 31.0 31.0 32.0 32.0 33.0 34.0 34.0 35.0 36.0 37.0 37.0 38.0 39.0 40.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 41.0 40.0 40.0 39.0 39.0 39.0 40.0 41.0 42.0 43.0 43.0 43.0 43.0 43.0 43.0 42.0 41.0 41.0 40.0 40.0 39.0 38.0 37.0 36.0 36.0 35.0 34.0 33.0 32.0 31.0 30.0 29.0 29.0 28.0 28.0 27.0 27.0 26.0 26.0))
          )
    (send (send robot :joint "RLEG_JOINT5") :joint-min-max-target (send robot :joint "RLEG_JOINT4"))
    (send (send robot :joint "RLEG_JOINT4") :joint-min-max-table
          '#2f((-30.0 -29.0 -28.0 -27.0 -26.0 -25.0 -24.0 -23.0 -22.0 -21.0 -20.0 -19.0 -18.0 -17.0 -16.0 -15.0 -14.0 -13.0 -12.0 -11.0 -10.0 -9.0 -8.0 -7.0 -6.0 -5.0 -4.0 -3.0 -2.0 -1.0 0.0 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 11.0 12.0 13.0 14.0 15.0 16.0 17.0 18.0 19.0 20.0 21.0 22.0 23.0 24.0 25.0 26.0 27.0 28.0 29.0 30.0 31.0 32.0 33.0 34.0 35.0 36.0 37.0 38.0 39.0 40.0 41.0 42.0 43.0 44.0)
               (-49.0 -53.0 -58.0 -60.0 -63.0 -65.0 -67.0 -68.0 -70.0 -71.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -71.0 -69.0 -67.0 -65.0 -64.0 -63.0 -61.0 -59.0 -58.0 -56.0 -54.0 -53.0 -51.0 -50.0 -49.0 -47.0 -46.0 -45.0 -44.0 -24.0 11.0)
               (38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 36.0 34.0 32.0 30.0 29.0 28.0 27.0 26.0 25.0 24.0 22.0 21.0 20.0 19.0 17.0 15.0 14.0))
          )
    (send (send robot :joint "RLEG_JOINT4") :joint-min-max-target (send robot :joint "RLEG_JOINT5"))
    (send (send robot :joint "LLEG_JOINT5") :joint-min-max-table
          '#2f((-74.0 -73.0 -72.0 -71.0 -70.0 -69.0 -68.0 -67.0 -66.0 -65.0 -64.0 -63.0 -62.0 -61.0 -60.0 -59.0 -58.0 -57.0 -56.0 -55.0 -54.0 -53.0 -52.0 -51.0 -50.0 -49.0 -48.0 -47.0 -46.0 -45.0 -44.0 -43.0 -42.0 -41.0 -40.0 -39.0 -38.0 -37.0 -36.0 -35.0 -34.0 -33.0 -32.0 -31.0 -30.0 -29.0 -28.0 -27.0 -26.0 -25.0 -24.0 -23.0 -22.0 -21.0 -20.0 -19.0 -18.0 -17.0 -16.0 -15.0 -14.0 -13.0 -12.0 -11.0 -10.0 -9.0 -8.0 -7.0 -6.0 -5.0 -4.0 -3.0 -2.0 -1.0 0.0 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 11.0 12.0 13.0 14.0 15.0 16.0 17.0 18.0 19.0 20.0 21.0 22.0 23.0 24.0 25.0 26.0 27.0 28.0 29.0 30.0 31.0 32.0 33.0 34.0 35.0 36.0 37.0 38.0 39.0)
               (-22.0 -22.0 -23.0 -23.0 -24.0 -24.0 -25.0 -25.0 -26.0 -27.0 -28.0 -28.0 -29.0 -29.0 -30.0 -31.0 -31.0 -32.0 -32.0 -33.0 -34.0 -34.0 -35.0 -36.0 -37.0 -37.0 -38.0 -39.0 -40.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -41.0 -40.0 -40.0 -39.0 -39.0 -39.0 -40.0 -41.0 -42.0 -43.0 -43.0 -43.0 -43.0 -43.0 -43.0 -42.0 -41.0 -41.0 -40.0 -40.0 -39.0 -38.0 -37.0 -36.0 -36.0 -35.0 -34.0 -33.0 -32.0 -31.0 -30.0 -29.0 -29.0 -28.0 -28.0 -27.0 -27.0 -26.0 -26.0)
               (19.0 19.0 20.0 21.0 21.0 22.0 23.0 23.0 24.0 24.0 25.0 25.0 25.0 26.0 26.0 27.0 27.0 27.0 27.0 27.0 28.0 28.0 28.0 28.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0))
          )
    (send (send robot :joint "LLEG_JOINT5") :joint-min-max-target (send robot :joint "LLEG_JOINT4"))
    (send (send robot :joint "LLEG_JOINT4") :joint-min-max-table
          '#2f((-44.0 -43.0 -42.0 -41.0 -40.0 -39.0 -38.0 -37.0 -36.0 -35.0 -34.0 -33.0 -32.0 -31.0 -30.0 -29.0 -28.0 -27.0 -26.0 -25.0 -24.0 -23.0 -22.0 -21.0 -20.0 -19.0 -18.0 -17.0 -16.0 -15.0 -14.0 -13.0 -12.0 -11.0 -10.0 -9.0 -8.0 -7.0 -6.0 -5.0 -4.0 -3.0 -2.0 -1.0 0.0 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 11.0 12.0 13.0 14.0 15.0 16.0 17.0 18.0 19.0 20.0 21.0 22.0 23.0 24.0 25.0 26.0 27.0 28.0 29.0 30.0)
               (11.0 -24.0 -44.0 -45.0 -46.0 -47.0 -49.0 -50.0 -51.0 -53.0 -54.0 -56.0 -58.0 -59.0 -61.0 -63.0 -64.0 -65.0 -67.0 -69.0 -71.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -71.0 -70.0 -68.0 -67.0 -65.0 -63.0 -60.0 -58.0 -53.0 -49.0)
               (14.0 15.0 17.0 19.0 20.0 21.0 22.0 24.0 25.0 26.0 27.0 28.0 29.0 30.0 32.0 34.0 36.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0))
          )
    (send (send robot :joint "LLEG_JOINT4") :joint-min-max-target (send robot :joint "LLEG_JOINT5"))
    )
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
            :margin 2))
  (send (send robot :joint "HEAD_JOINT0") :put :org-min-angle -61.0)
  (send (send robot :joint "HEAD_JOINT0") :put :org-max-angle 61.0)
  (send (send robot :joint "HEAD_JOINT1") :put :org-min-angle -21.0)
  (send (send robot :joint "HEAD_JOINT1") :put :org-max-angle 48.0)
  (send (send robot :joint "HEAD_JOINT0") :joint-min-max-table
        '#2f((-21 -20 -19 -18 -17 -16 -15 -14 -13 -12 -11 -10 -9 -8 -7 -6 -5 -4 -3 -2 -1 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48)
             (-8 -17 -26 -61 -61 -61 -61 -61 -61 -61 -61 -61 -61 -61 -61 -61 -61 -61 -61 -61 -61 -61 -60 -60 -60 -60 -60 -60 -60 -35 -33 -33 -32 -32 -31 -31 -30 -30 -30 -30 -29 -29 -29 -29 -29 -28 -28 -28 -28 -27 -27 -26 -26 -26 -25 -24 -23 -22 -21 -20 -18 -17 -16 -14 -13 -12 -10 -10 -9 -8)
             (8 17 26 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 60 60 60 60 60 60 60 35 33 33 32 32 31 31 30 30 30 30 29 29 29 29 29 28 28 28 28 27 27 26 26 26 25 24 23 22 21 20 18 17 16 14 13 12 10 10 9 8))
        )
  (send (send robot :joint "HEAD_JOINT0") :joint-min-max-target (send robot :joint "HEAD_JOINT1"))
  (send (send robot :joint "HEAD_JOINT1") :joint-min-max-table
        '#2f((-61 -60 -59 -58 -57 -56 -55 -54 -53 -52 -51 -50 -49 -48 -47 -46 -45 -44 -43 -42 -41 -40 -39 -38 -37 -36 -35 -34 -33 -32 -31 -30 -29 -28 -27 -26 -25 -24 -23 -22 -21 -20 -19 -18 -17 -16 -15 -14 -13 -12 -11 -10 -9 -8 -7 -6 -5 -4 -3 -2 -1 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61)
             (-18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -19 -19 -19 -19 -19 -19 -19 -19 -19 -20 -20 -20 -20 -20 -20 -20 -20 -20 -21 -21 -21 -21 -21 -21 -21 -21 -21 -21 -21 -21 -21 -21 -21 -21 -21 -20 -20 -20 -20 -20 -20 -20 -20 -20 -19 -19 -19 -19 -19 -19 -19 -19 -19 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18 -18)
             (0 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 8 8 10 12 14 18 23 27 29 32 33 34 35 36 37 38 38 39 40 41 41 42 43 44 44 46 47 48 48 48 48 48 48 48 48 48 48 48 48 48 48 48 48 48 47 46 44 44 43 42 41 41 40 39 38 38 37 36 35 34 33 32 29 27 23 18 14 12 10 8 8 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 0))
        )

  (send (send robot :joint "HEAD_JOINT1") :joint-min-max-target (send robot :joint "HEAD_JOINT0"))
   ;; (send robot :make-joint-min-max-table
   ;;    (send robot :head :neck-y :parent-link)
   ;;    (send robot :head :neck-p :child-link)
   ;;    (send robot :head :neck-y)
   ;;    (send robot :head :neck-p)
   ;;    :margin margin)
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
   (format f "     (send self :define-min-max-table)~%")
   (format f "    ))~%")
   (format f "  (:define-min-max-table~%")
   (format f "    ()~%")
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
     (format f "      ))~%")
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
