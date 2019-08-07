^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_body_filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
