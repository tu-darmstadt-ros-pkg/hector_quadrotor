^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_quadrotor_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.5 (2015-03-28)
------------------

0.3.4 (2015-02-22)
------------------
* added missing run_depend hector_quadrotor_pose_estimation to package.xml
* set pose_estimation/publish_world_nav_transform parameter to true explicitly
* updated package for the latest version of hector_pose_estimation
  * Geographic reference latitude and longitude set to 49.860246N 8.687077E (Lichtwiese).
  * Reenabled auto_elevation, auto_reference and auto_heading parameters for hector_pose_estimation.
  hector_pose_estimation will publish the world->nav transform depending on its reference pose.
* added parameter file for hector_quadrotor_pose_estimation for a simulated quadrotor
  The parameter file disables the auto_elevation, auto_reference, auto_heading modes of hector_pose_estimation and sets the corresponding values
  manually with what is simulated in the gazebo sensor plugins. This simplifies the comparison of estimated poses with ground truth information.
* explicitly set the pose_estimation/nav_frame parameter in spawn_quadrotor.launch
* disabled detection of available plugins in cmake
  The aerodynamics and propulsion plugins are built unconditinally now in hector_quadrotor_gazebo_plugins and the detection is obsolete.
  Additionally we used platform-specific library prefixes and suffixes in find_libary() which caused errors on different platforms.
* Contributors: Johannes Meyer

0.3.3 (2014-09-01)
------------------
* cleaned up launch files and fixed argument forwarding to spawn_quadrotor.launch
* removed all RTT related plugins
* added separate update timer for MotorStatus output in propulsion plugin
* added launch file argument to enable/disable pose estimation
* moved simulation package dependencies from hector_quadrotor metapackage to hector_quadrotor_gazebo
* Contributors: Johannes Meyer

0.3.2 (2014-03-30)
------------------

0.3.1 (2013-12-26)
------------------
* also check if a target exists when searching available plugins
* enables aerodynamics plugin in default configuration
* limit controlPeriod to 100Hz
* a few fixes for RTT integration in hector_quadrotor. Added urdf macro for rtt_gazebo_plugin macro.
* deprecated quadrotor_simple_controller.gazebo.xacro
* fixed node type for static_transform_publisher in spawn_quadrotor.launch
* changed frame_id for gazebo fixed frame to /world and added a static_transform_publisher for world->nav
* increased drift for the barometric pressure sensor
* added some command input ports to quadrotor_controller.gazebo.xacro
* Contributors: Johannes Meyer

0.3.0 (2013-09-11)
------------------
* Catkinized stack hector_quadrotor and integrated hector_quadrotor_demo package from former hector_quadrotor_apps stack
* increased drift for the barometric pressure sensor
* added some command input ports to quadrotor_controller.gazebo.xacro
* added launch file for two quadrotors
