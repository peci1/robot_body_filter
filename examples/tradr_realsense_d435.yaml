# This is an example filter config for tracked vehicle Absolem from Czech Technical University's
# VRAS team. The robot is equipped with a Realsense D435.
# The field of view of the sensor can only capture a few links, so the `only_links` parameter is set
# to increase performance. Also, the `filter/keep_clouds_organized` parameter is important here to
# retain the 2D structure of the RGBD camera output.
cloud_filter_chain:
  - name: RobotBodyFilter
    type: robot_body_filter/RobotBodyFilterPointCloud2
    params:
      frames/sensor: 'camera_color_optical_frame'
      filter/keep_clouds_organized: True
      filter/do_clipping: True
      filter/do_contains_test: True
      filter/do_shadow_test: False
      sensor/point_by_point: False
      sensor/min_distance: 0.3
      sensor/max_distance: 5.0
      only_links: ["front_left_flipper", "front_right_flipper", "laser"]
      ignored_links/everywhere: ["front_left_flipper::front_left_flipper_collision_large_wheel", "front_right_flipper::front_right_flipper_collision_large_wheel"]
      body_model/inflation/scale: 1.07
      body_model/inflation/padding: 0.01
      body_model/robot_description_param: '/nifti_robot_description'
      transforms/buffer_length: 15.0
      transforms/timeout/reachable: 0.2
      transforms/timeout/unreachable: 0.2
      bounding_sphere/compute: False
      bounding_box/compute: False
      oriented_bounding_box/compute: False
      local_bounding_box/compute: False
      debug/pcl/inside: False
      debug/pcl/clip: False
      debug/pcl/shadow: False
      debug/marker/contains: False
      debug/marker/shadow: False
