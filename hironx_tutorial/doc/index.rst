
Types of available sample code
==============================

stampit!
--------

Used at IROS 2013 and iREX 2013 in Tokyo. 
NEXTAGE OPEN robot keeps carrying and stamping on a paper
(`video available on youtube <https://www.youtube.com/watch?v=i051WKKelSw>`_).

Without the real robot, you can run the sample by::

  terminal-1$ source `rospack find openrtm_tools`/scripts/rtshell-setup.sh
  terminal-1$ rtmlaunch hironx_tutorial stampit_sim_nodynamics.launch
  
  terminal-2$ rosrun hironx_tutorial stampit_demo.py 


With the real robot,::

  terminal-1$ rtmlaunch hironx_tutorial stampit_realrobot.launch nameserver:=%HOSTNAME%
  
  terminal-2$ rosrun hironx_tutorial stampit_demo.py - --host %HOSTNAME% --port 15005

`%HOSTNAME%` is the host name that can be pinged at, `hiro014` for example.

HIRONXJSK Picking Demo
----------------------

.. image:: images/hironxjsk_picking_demo_gazebo.gif
   :width: 60%
   :align: center

.. image:: images/hironxjsk_picking_demo_rviz.jpg
   :width: 60%
   :align: center

Installation
++++++++++++

To run this demo, you have to build this package with ``hrp2_models`` (closed package) and ``hrpsys_ros_bridge_tutorials``.
If you compiled ``hrpsys_ros_bridge_tutorials`` before you download ``hrp2_models``, you have to compile ``hrpsys_ros_bridge_tutorials`` again with ``--force-cmake`` option after you download ``hrp2_models``.

Execution
+++++++++

.. code-block:: bash

  $ roslaunch hironx_tutorial hironxjsk_picking_demo.launch
  # Wait until the robot stops moving
  $ rosrun hironx_tutorial hironxjsk-picking-demo.l
