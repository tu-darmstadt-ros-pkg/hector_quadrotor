^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_quadrotor_model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.3 (2014-09-01)
------------------
* fixed target_link_libraries() line for the quadrotor_aerodynamics library
* fixed missing cmake commands to link with boost and roscpp
* always use newest available motor command in propulsion plugin
* added cmake_modules dependency
* Contributors: Johannes Meyer, Rasit Eskicioglu

0.3.2 (2014-03-30)
------------------

0.3.1 (2013-12-26)
------------------
* fixed copy&paste error in quadrotor_aerodynamics.cpp
* output drag forces in body coordinate system and added orientation input
* fixed configuration namespace and plugin cleanup
* added boolean return value for configure() methods
* accept hector_uav_msgs/MotorCommand messages directly for the propulsion model/plugin
* copied generated C code within the cpp classes to not require Matlab for compilation
* fixed sign of aerodynamic forces
* use precompiled codegen libraries if available
* Contributors: Johannes Meyer

0.3.0 (2013-09-11)
------------------
* Catkinized stack hector_quadrotor and integrated hector_quadrotor_demo package from former hector_quadrotor_apps stack
