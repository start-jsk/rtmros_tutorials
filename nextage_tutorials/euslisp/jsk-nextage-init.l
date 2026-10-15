#!/usr/bin/env roseus
;; If you use `Go to init pose` from the rqt_dashboard, you cannot send your command from euslisp scripts befor you reboot the robot and host PC because we use fullbody controller from jsk-nextage-interface.l for some reasons (The biggest one is that if you send the high control rate command from each limb controller then the motion is awkward. Michi-Tsubaki and pazeshun cannot determined why this was.), but the `Go to init pose overwrite the fullbody controller since it sends commands from each limb controller. So it is better using this script!!!

(require "package://nextage_tutorials/euslisp/jsk-nextage-interface.l")
(nextage-init)

(setq *robot* *nextage*)
(objects (list *robot*))
(send *robot* :reset-pose)
(send *robot* :head :angle-vector #f(0 60)) ;; look down
(send *robot* :larm :move-end-pos #f(0 0 100))
(send *robot* :rarm :move-end-pos #f(0 0 100))
(send *ri* :angle-vector (send *nextage* :angle-vector) 10000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)
