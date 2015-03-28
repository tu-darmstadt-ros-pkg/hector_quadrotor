^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_quadrotor_pose_estimation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.5 (2015-03-28)
------------------

0.3.4 (2015-02-22)
------------------
* added missing install rule for hector_quadrotor_pose_estimation_nodelets.xml
  Many thanks to Bernd Kast for pointing me to this issue.
* update raw baro height as position.z component in sensor pose
  See https://github.com/tu-darmstadt-ros-pkg/hector_localization/commit/bd334f0e30c42fb5833fa8ffa249dfd737d43ddc.
* updated package for the latest version of hector_pose_estimation
  * Geographic reference latitude and longitude set to 49.860246N 8.687077E (Lichtwiese).
  * Reenabled auto_elevation, auto_reference and auto_heading parameters for hector_pose_estimation.
  hector_pose_estimation will publish the world->nav transform depending on its reference pose.
* added parameter file for hector_quadrotor_pose_estimation for a simulated quadrotor
  The parameter file disables the auto_elevation, auto_reference, auto_heading modes of hector_pose_estimation and sets the corresponding values
  manually with what is simulated in the gazebo sensor plugins. This simplifies the comparison of estimated poses with ground truth information.
* shutdown the height subscriber (in favor of topic alitimeter) to not have two height updates
* Contributors: Johannes Meyer

0.3.3 (2014-09-01)
------------------

0.3.2 (2014-03-30)
------------------

0.3.1 (2013-12-26)
------------------

0.3.0 (2013-09-11)
------------------
* hector_quadrotor_pose_estimation: added cmake target dependency on hector_uav_msgs_generate_messages_cpp
* hector_quadrotor: added package hector_quadrotor_pose_estimation
* Contributors: Johannes Meyer
