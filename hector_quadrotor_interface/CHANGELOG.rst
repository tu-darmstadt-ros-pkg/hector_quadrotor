^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_quadrotor_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.5 (2015-03-28)
------------------
* updated angular/z controller parameters
* Remove redundant callback queue in twist_controller, use root_nh queue as per ros_control API
* Add controller timeout to allow faster shutdown of spawner
* Contributors: Johannes Meyer, Paul Bovbel

0.3.4 (2015-02-22)
------------------
* improved automatic landing detection and shutdown on rollovers
* slightly updated velocity controller limits and gains
* Contributors: Johannes Meyer

0.3.3 (2014-09-01)
------------------
* fixed some compiler warnings and missing return values
* increased integral gain for attitude stabilization (fix #12)
* make a copy of the root NodeHandle in all controllers
  For some reason deconstructing the TwistController resulted in a pure virtual function call without this patch.
* Contributors: Johannes Meyer

0.3.2 (2014-03-30)
------------------
* Fix boost 1.53 issues
  changed boost::shared_dynamic_cast to boost::dynamic_pointer_cast and
  boost::shared_static_cast to boost::static_pointer_cast
* use a separate callback queue thread for the TwistController
* added optional twist limit in pose controller
* Contributors: Christopher Hrabia, Johannes Meyer

0.3.1 (2013-12-26)
------------------
* New controller implementation using ros_control
* Added pose controller
* Added motor controller that controls motor voltages from wrench commands
* Contributors: Johannes Meyer

0.3.0 (2013-09-11)
------------------
* Catkinized stack hector_quadrotor and integrated hector_quadrotor_demo package from former hector_quadrotor_apps stack
* Propulsion and aerodynamics models are in hector_quadrotor_model package now.
* Gazebo plugins are in hector_quadrotor_gazebo_plugins package now.
