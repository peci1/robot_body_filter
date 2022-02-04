<?xml version="1.0"?>
<launch>
    <!-- Prerequisite: sensor_filters -->
    <!-- http://wiki.ros.org/sensor_filters -->
    <node name="laser_filter" pkg="sensor_filters" type="pointcloud2_filter_chain" output="screen">
        <rosparam command="load" file="$(dirname)/vlp16_params.yaml" />
        <remap from="~input" to="velodyne_points" />
        <remap from="~output" to="velodyne_points_filtered" />
    </node>
</launch>
