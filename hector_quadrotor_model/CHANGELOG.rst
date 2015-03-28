^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_quadrotor_model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.5 (2015-03-28)
------------------

0.3.4 (2015-02-22)
------------------
* make models more robust against irregular input values
  Especially on collisions with walls and obstacles Gazebo can output very high
  velocity values for some reason, which caused the propulsion and aerodynamic
  models to output infinite values or NaN as resulting forces and torques, with
  the consequence that Gazebo effectively stopped simulation and showed the quadrotor
  at the origin of the world frame.
  With this patch the simulation is much more stable in case of collisions or
  hard landings.
* disabled quadratic term (CT2s) in propeller speed to performance factor J
  Because of this term the model output a non-zero thrust even if the propeller speed was zero.
  The quadratic term is due to drag and therefore covered in the aerodynamics model.
* Contributors: Johannes Meyer

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
