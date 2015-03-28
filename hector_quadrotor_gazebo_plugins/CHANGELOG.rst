^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_quadrotor_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.5 (2015-03-28)
------------------

0.3.4 (2015-02-22)
------------------
* added dynamic_reconfigure server to gazebo_ros_baro plugin
  See https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/commit/e1698e1c7bfa5fce6a724ab0a922a88bd49c9733 for
  the equivalent commit in hector_gazebo_plugins.
* publish propulsion and aerodynamic wrench as WrenchStamped
  This is primarily for debugging purposes.
  The default topic for the propulsion plugin has been changed to propulsion/wrench.
* disabled detection of available plugins in cmake
  The aerodynamics and propulsion plugins are built unconditinally now in hector_quadrotor_gazebo_plugins and the detection is obsolete.
  Additionally we used platform-specific library prefixes and suffixes in find_libary() which caused errors on different platforms.
* Contributors: Johannes Meyer

0.3.3 (2014-09-01)
------------------
* fixed some compiler warnings and missing return values
* added separate update timer for MotorStatus output in propulsion plugin
* Contributors: Johannes Meyer

0.3.2 (2014-03-30)
------------------

0.3.1 (2013-12-26)
------------------
* disabled separate queue thread for the aerodynamics plugin
* fixed configuration namespace and plugin cleanup
* aerodynamics plugin should apply forces and torques in world frame
* accept hector_uav_msgs/MotorCommand messages directly for the propulsion model/plugin
* deleted deprecated export section from package.xml
* abort with a fatal error if ROS is not yet initialized + minor code cleanup
* fixed commanded linear z velocity upper bound for auto shutdown in simple controller plugin
* improved auto shutdown to prevent shutdowns while airborne
* added motor engage/shutdown, either automatically (default) or using ROS services /engage and /shutdown
  (std_srvs/Empty)
* using ROS parameters to configure state topics
* use controller_manager in gazebo_ros_control instead of running standalone pose_controller
* Contributors: Johannes Meyer

0.3.0 (2013-09-11)
------------------
* Catkinized stack hector_quadrotor and integrated hector_quadrotor_demo package from former hector_quadrotor_apps stack
* added wrench publisher to the quadrotor_simple_controller plugin
* created new package hector_quadrotor_model and moved all gazebo plugins to hector_quadrotor_gazebo_plugins
