^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_body_filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.2 (2024-01-22)
------------------
* Do not segfault when a mesh resource is not found.
* Added full_example.
* Contributors: Martin Pecka

1.3.1 (2023-07-14)
------------------
* Explicitly specify minimum versions of required libraries.
* Contributors: Martin Pecka

1.3.0 (2023-04-12)
------------------
* Removed local implementation of oriented bounding boxes in favor of geometric_shapes/obb. **This change breaks API and ABI**, but I hope nobody explicitly used the OBB part of this library. This change requires geometric_shapes version 0.6.6+ (Melodic) or 0.7.5+ (Noetic) (released April 2023).
* Add example launch & config for ease of use. Thanks Doan Nguyen for the contribution!
* Changed xmlrpc_traits variables to constexpr static instead of inline static to decrease the required C++ language standard for this part. Changed stringType from std::string to const char*.
* Improved xmlrpc_traits to recognize more types of valid structures.
* Make filter_utils FilterBase::getParamVerbose() methods const.
  Allowed by https://github.com/ros/filters/pull/35 (released in Melodic filters 1.8.2 (October 2021) and Noetic filters 1.9.1 (September 2021)).
* Contributors: Doan Nguyen, Martin Pecka

1.2.2 (2021-08-25)
------------------
* Change ROS_WARN to ROS_INFO when loading a value of an undefined parameter
* Add link to the youtube tutorial
* Contributors: Martin Pecka

1.2.1 (2021-08-06)
------------------
* Merge pull request `#15 <https://github.com/peci1/robot_body_filter/issues/15>`_ from universal-field-robots/master
  TFFramesWatchdog and RayCastingShapeMask need to be installed in the CMakeLists.txt
* Added RayCastingShapeMask and TFFramesWatchdog to install targets in cmake
* Contributors: Josh Owen, Martin Pecka

1.2.0 (2021-07-30)
------------------
* Merge pull request `#11 <https://github.com/peci1/robot_body_filter/issues/11>`_ from peci1/correct-pointcloud-transforms
  Add possibility to specify pointcloud channels that should be transformed together with positional data.
* Merge pull request `#14 <https://github.com/peci1/robot_body_filter/issues/14>`_ from peci1/per_link_scaling
  Add support for scaling/padding each link differently
* Short-circuit classification of NaN points.
* Warn about missing collision elements only for non-ignored links and only if they have at least one visual.
* Fixed bounding shapes computation
* Added filter/max_shadow_distance for great performance increase
* Added possibility to scale/pad collisions separately for computation of bounding box and sphere.
* Unique-ify test target name to allow building with geometric_shapes.
* Use correct Eigen allocator.
* Reflected the newly added support for non-uniformly scaled meshes.
* Contributors: Martin Pecka

1.1.9 (2021-04-17)
------------------
* Compatibility with Noetic.
* Contributors: Martin Pecka

1.1.8 (2020-04-06)
------------------
* Fixed typo.
* Contributors: Martin Pecka

1.1.7 (2020-04-05)
------------------
* When sensor frame is empty (autodetected), do not add it as initial monitored frame to watchdog.
* Added configuration examples.
* Make sure use_sim_time is set to false for tests.
* Fixed computation of pointcloud without local bounding box.
* Added tests for RobotBodyFilter.
* No longer require the index pointcloud field in computeMask(), as it is not used anywhere.
* Surprise! CropBox::setTransform() doesn't set transform of the cropbox, but instead a transform of the points. Fixed that.
* Fixed copy-paste bug in bounding sphere publisher.
* Fix incorrect usage of fixed frame at many places (substituted with filtering frame).
* Make sure orientation.w = 1 is set in all published markers.
* Correctly exclude the ignored bounding shapes from the published markers and shapes.
* Make sure the published bounding shapes are computed as they appear at the time of the beginning of the scan.
* Fix getShapeTransform() to correctly interpolate between before- and after-scan times. The logic was twisted, so with cacheLookupBetweenScansRation was zero, it returned the transform after scan.
* Fix copy-paste bug in transform cache.
* Make use of the newly optional scanFrame argument of computeMask()
* Prevent division by zero in point-by-point scans with zero duration (which is weird, but well...)
* Only compute sensor->filtering transform for all-at-once scans, because point-by-point scans have viewpoints instead.
* Reworked how TFFramesWatchdog is initialized to allow easier reconfiguring of the filter.
* Added RobotBodyFilter::failWithoutRobotDescription which aids in tests.
* Clarified documentation of RobotBodyFilter regarding frames.
* Add TFFramesWatchdog::isRunning().
* Workaround PCL 1.8 ignoring CropBox::setKeepOrganized().
* Added tests for filter utils.
* Improved filter_utils documentation.
* Improved getParamVerbose() with recursive resolution of slash-separated param names.
* Make different performance test requirements for release and debug builds.
* Added tests for RayCastingShapeMask and improved its documentation.
* Added utility functions to compare transforms and generate random ones.
* Implemented TF watchdog tests.
* Prevented some more SIGABRTs from TFFramesWatchdog.
* Added unit tests for utils.
* Fixed a bug in OBB.contains(OBB) which compared against vertices of the wrong OBB.
* Improved efficiency of Eigen expression by not using auto type.
* Fix the reported transformation of default-constructed OBBs.
* Fixed constructShapeFromBody() to correctly process meshes.
* Fixed eigen and urdf includes.
* Fixed printing of std containers in string_utils.
* CREATE_FILTERED_CLOUD now prepends robot_body_filter stuff with a global namespace to be usable even from other namespaces.
* Make sure sensor frame is monitored by the watchdog because we need to use it during the filtering. Fixes `#6 <https://github.com/peci1/robot_body_filter/issues/6>`_.
* Added parameter transforms/require_all_reachable.
* Added to_string(bool) specialization.
* Contributors: Martin Pecka

1.1.6 (2019-11-11)
------------------
* Avoid subtracting from ROS time.
  This may lead to exceptions in simulation, where current time is 0. ros::Time cannot be negative.
* Contributors: Martin Pecka

1.1.5 (2019-11-08)
------------------
* refactor(RobotBodyFilter): Made frames/sensor parameter optional (`#2 <https://github.com/peci1/robot_body_filter/issues/2>`_)
  The sensor frame will now be derived from the incoming sensor messages.
  This way, messages from different sensors can be processed and the
  filter can be configured without any knowledge of the sensor.
* Improved readme.
* Reorganized the readme.
* Added support for non-uniform scaling of collision meshes.
* Contributors: Martin Pecka, Rein Appeldoorn

1.1.4 (2019-08-07)
------------------
* Fixed a bug in filtering unorganized point clouds.
* Contributors: Martin Pecka

1.1.1 (2019-07-30)
------------------
* Fixed dependencies
* Contributors: Martin Pecka

1.1.0 (2019-07-18)
------------------
* Increasing performance.
* More descriptive frame configurations. Added the possibility to leave out clipping, contains or shadow tests.
* Repurposed param ~transforms/timeout/reachable to be always used with remainingTime.
* More efficient point transformation
* Little improvements. Renamed the utils library to robot_body_filter_utils.
* Removed compiler pragmas - they seem no longer needed.
* Provided a fix for wrong box-ray intersections.
* Fixed bodies.h header guard.
* Fixed bug in cylinder shape creation.
* Fixed a bug in constructMarkerFromBody() - references do not correctly support polymorphism.
* Fixes for robot body filter. Added new bounding box types.
* Fixed a few bugs while running under a nodelet.
* Prepared remainingTime() to a situation where ROS time hasn't yet been initialized.
* Added possibility to update only robot pose only with every n-th point in point_by_point scans.
  Added possibility to publish bounding sphere and box markers.
* Renamed to robot_body_filter.
* Reworked as a laser filter combining contains and shadow tests.
* Make use of bodies.h and shapes.h from geometric_shapes.
* Enabling generic point types, removed PCL dependency, removed unnecessary params.
* Using all collision elements for each link instead of only the first one.
* Testing all intersections instead of only the first one.
* Merge branch 'master' into indigo-devel
* Add robot_self_filter namespace before bodies and shapes namespace.
  geometric_shapes package also provides bodies and shapes namespace
  and same classes and functions. If a program is linked with
  geometric_shapes and robot_self_filter, it may cause strange behavior
  because of symbol confliction.
* Contributors: Martin Pecka, Ryohei Ueda, Tomas Petricek

0.1.31 (2018-11-24)
-------------------
* update CHANGELOG
* Merge pull request `#16 <https://github.com/peci1/robot_body_filter/issues/16>`_ from mikaelarguedas/tinyxml_dependency
  depends on tinyxml and link against it
* Merge branch 'indigo-devel' into tinyxml_dependency
* Merge pull request `#18 <https://github.com/peci1/robot_body_filter/issues/18>`_ from k-okada/add_travis
  update travis.yml
* update travis.yml
* depend on tinyxml and link against it
* Merge pull request `#14 <https://github.com/peci1/robot_body_filter/issues/14>`_ from traclabs/indigo-devel
  Minor changes to indigo-devel CMake allow this to be used in kinetic and indigo
* Changes for kinetic
* Contributors: Devon Ash, Kei Okada, Mikael Arguedas, Patrick Beeson

0.1.30 (2017-01-20)
-------------------
* Update CHANGELOG.rst
* Merge pull request `#15 <https://github.com/peci1/robot_body_filter/issues/15>`_ from PR2/fix-typo-cmakelists
  Fix typo in CMakeLists.txt: CATKIN-DEPENDS -> CATKIN_DEPENDS
* Fix typo in CMakeLists.txt: CATKIN-DEPENDS -> CATKIN_DEPENDS
* Merge pull request `#12 <https://github.com/peci1/robot_body_filter/issues/12>`_ from garaemon/max-queue-size
  Add ~max_queue_size parameter for subscription queue size
* Add ~max_queue_size parameter for subscription queue size
* Contributors: Devon Ash, Kentaro Wada, Ryohei Ueda

0.1.29 (2015-12-05)
-------------------
* Re-create changelog for robot_self_filter
* Merge pull request `#10 <https://github.com/peci1/robot_body_filter/issues/10>`_ from garaemon/pr-4-indigo-devel
  Add robot_self_filter namespace before bodies and shapes namespace.
* Add robot_self_filter namespace before bodies and shapes namespace.
  geometric_shapes package also provides bodies and shapes namespace
  and same classes and functions. If a program is linked with
  geometric_shapes and robot_self_filter, it may cause strange behavior
  because of symbol confliction.
* Contributors: Ryohei Ueda

0.1.28 (2015-12-04)
-------------------
* Merge pull request `#8 <https://github.com/peci1/robot_body_filter/issues/8>`_ from wkentaro/indigo-devel-merge-master
  Merge master branch to indigo-devel
* Merge remote-tracking branch 'origin/master' into indigo-devel
* Added indigo devel
* Merge pull request `#7 <https://github.com/peci1/robot_body_filter/issues/7>`_ from wkentaro/self_filter-timestamp
  Set correct timestamp for self filtered cloud
* Set correct timestamp for self filtered cloud
  This is needed because pcl drops some value of timestamp.
  So pcl::fromROSMsg and pcl::toROSMsg does not work to get correct timestamp.
* Merge pull request `#5 <https://github.com/peci1/robot_body_filter/issues/5>`_ from garaemon/use-protected-member
  Protected member variables in SelfMask for subclass of SelfMask
* Protected member variables in SelfMask for subclass of SelfMask
* Contributors: Devon Ash, Kentaro Wada, Ryohei Ueda, TheDash

0.1.27 (2015-12-01)
-------------------
* Merge pull request `#1 <https://github.com/peci1/robot_body_filter/issues/1>`_ from garaemon/robot-self-filter
  Porting robot_self_filter from pr2_navigation_self_filter
* Porting robot_self_filter from pr2_navigation_self_filter
* Initial commit
* Contributors: Devon Ash, Ryohei Ueda
