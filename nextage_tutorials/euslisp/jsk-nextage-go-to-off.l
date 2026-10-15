#!/usr/bin/env roseus
;; If you use `Go to power off pose` from the rqt_dashboard, you cannot send your command from euslisp scripts befor you reboot the robot and host PC because we use fullbody controller from jsk-nextage-interface.l for some reasons (The biggest one is that if you send the high control rate command from each limb controller then the motion is awkward. Michi-Tsubaki and pazeshun cannot determined why this was.), but the `Go to power off pose overwrite the fullbody controller since it sends commands from each limb controller. So it is better using this script!!!

(load "jsk-nextage-interface.l")
(nextage-init)
(send *nextage* :angle-vector #f(0.0 0.0 -140.0 -158.0 0.0 0.0 0.0 0.0 -140.0 -158.0 0.0 0.0 0.0 0.0 0.0))
(send *ri* :angle-vector (send *nextage* :angle-vector) 15000)

;;  TODO: Currently hrpsys and openrtm cannot be build on ubuntu jammy and noble, so `(send *ri* :servo-off)` cannot be sent from this scripts on the user laptop.
