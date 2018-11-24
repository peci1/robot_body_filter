^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_self_filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.31 (2018-11-24)
-------------------
* Merge pull request `#16 <https://github.com/PR2/robot_self_filter/issues/16>`_ from mikaelarguedas/tinyxml_dependency
  depends on tinyxml and link against it
* Merge branch 'indigo-devel' into tinyxml_dependency
* Merge pull request `#18 <https://github.com/PR2/robot_self_filter/issues/18>`_ from k-okada/add_travis
  update travis.yml
* update travis.yml
* depend on tinyxml and link against it
* Merge pull request `#14 <https://github.com/PR2/robot_self_filter/issues/14>`_ from traclabs/indigo-devel
  Minor changes to indigo-devel CMake allow this to be used in kinetic and indigo
* Changes for kinetic
* Contributors: Devon Ash, Kei Okada, Mikael Arguedas, Patrick Beeson

0.1.30 (2017-01-20)
-------------------
* Fix typo in CMakeLists.txt: CATKIN-DEPENDS -> CATKIN_DEPENDS
* Add ~max_queue_size parameter for subscription queue size
* Contributors: Devon Ash, Kentaro Wada, Ryohei Ueda

0.1.29 (2015-12-05)
-------------------
* pr2_navigation_self_filter -> robot_self_filter
* Add robot_self_filter namespace before bodies and shapes namespace.
  geometric_shapes package also provides bodies and shapes namespace
  and same classes and functions. If a program is linked with
  geometric_shapes and robot_self_filter, it may cause strange behavior
  because of symbol confliction.
* Contributors: Ryohei Ueda

0.1.28 (2015-12-04)
-------------------
* Added indigo devel
* Set correct timestamp for self filtered cloud
  This is needed because pcl drops some value of timestamp.
  So pcl::fromROSMsg and pcl::toROSMsg does not work to get correct timestamp.
  Protected member variables in SelfMask for subclass of SelfMask
* Protected member variables in SelfMask for subclass of SelfMask
* Contributors: Devon Ash, Kentaro Wada, Ryohei Ueda, TheDash

0.1.27 (2015-12-01)
-------------------
* Porting robot_self_filter from pr2_navigation_self_filter
* Initial commit
* Contributors: Devon Ash, Ryohei Ueda
