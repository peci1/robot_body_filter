# robot_body_filter

Filters the robot's body out of point clouds and laser scans.

## Changes vs [PR2/robot_self_filter](https://github.com/PR2/robot_self_filter):
- Now the package is a normal `filters::FilterBase` filter and not a standalone node.
- Using both containment and ray-tracing tests. 
- Using all collision elements for each link instead of only the first one.
- Enabling generic point type, removing PCL dependency and unnecessary params.
- Using bodies.h and shapes.h from geometric_shapes.
- As a by-product, the filter can compute robot's bounding box or sphere.

## Build Status

### Continuous integration

| ROS Version | Build Status |
|---|---|
| __Melodic__ | [![Build Status](https://github.com/peci1/robot_body_filter/workflows/CI/badge.svg?branch=master)](https://github.com/peci1/robot_body_filter/actions?query=workflow%3ACI) <br/> [![Build Status](https://api.travis-ci.com/peci1/robot_body_filter.svg?branch=master&status=passed)](https://travis-ci.com/github/peci1/robot_body_filter) |


### Dev job

| ROS Version | Build Status |
|---|---|
| __Melodic__ | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__robot_body_filter__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__robot_body_filter__ubuntu_bionic_amd64) |
| __Noetic__ | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ndev__robot_body_filter__ubuntu_focal_amd64)](http://build.ros.org/job/Ndev__robot_body_filter__ubuntu_focal_amd64) |

### Release jobs

| ROS Version | Ubuntu amd64 | Ubuntu armhf | Ubuntu arm64 | Debian amd64 | Debian arm64 |
|---|---|---|---|---|---|
| __Melodic__ | Bionic [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__robot_body_filter__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__robot_body_filter__ubuntu_bionic_amd64__binary) | Bionic [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ubhf_uBhf__robot_body_filter__ubuntu_bionic_armhf__binary)](http://build.ros.org/job/Mbin_ubhf_uBhf__robot_body_filter__ubuntu_bionic_armhf__binary) | Bionic [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ubv8_uBv8__robot_body_filter__ubuntu_bionic_arm64__binary)](http://build.ros.org/job/Mbin_ubv8_uBv8__robot_body_filter__ubuntu_bionic_arm64__binary) | Stretch [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_ds_dS64__robot_body_filter__debian_stretch_amd64__binary)](http://build.ros.org/job/Mbin_ds_dS64__robot_body_filter__debian_stretch_amd64__binary) | Stretch [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_dsv8_dSv8__robot_body_filter__debian_stretch_arm64__binary)](http://build.ros.org/job/Mbin_dsv8_dSv8__robot_body_filter__debian_stretch_arm64__binary) |
| __Noetic__ | Focal [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__robot_body_filter__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__robot_body_filter__ubuntu_focal_amd64__binary) | Focal [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_ufhf_uFhf__robot_body_filter__ubuntu_focal_armhf__binary)](http://build.ros.org/job/Nbin_ufhf_uFhf__robot_body_filter__ubuntu_focal_armhf__binary) | Focal [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_ufv8_uFv8__robot_body_filter__ubuntu_focal_arm64__binary)](http://build.ros.org/job/Nbin_ufv8_uFv8__robot_body_filter__ubuntu_focal_arm64__binary) | Buster [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_db_dB64__robot_body_filter__debian_buster_amd64__binary)](http://build.ros.org/job/Nbin_db_dB64__robot_body_filter__debian_buster_amd64__binary) | Buster [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_dbv8_dBv8__robot_body_filter__debian_buster_arm64__binary)](http://build.ros.org/job/Nbin_dbv8_dBv8__robot_body_filter__debian_buster_arm64__binary) |

## Basic Operation

### `filters::FilterBase` API

The basic workings of this filter are done via the [`filters::FilterBase` API](http://wiki.ros.org/filters) implemented for `sensor_msgs::LaserScan` and `sensor_msgs::PointCloud2` types. This means you can load this filter into a FilterChain along other filters as usual. Different from the standard filters, this one can also publish several interesting topics and subscribes to TF.

### Subscribed Topics

- `/tf`, `/tf_static`

    Transforms
- `dynamic_robot_model_server/parameter_updates` (`dynamic_reconfigure/Config`)

    Dynamic reconfigure topic on which updated robot model can be subscribed. 
    The model is read from a field defined by parameter 
    `body_model/dynamic_robot_description/field_name`.

### Published Topics

- `scan_point_cloud_no_bbox` (`sensor_msgs/PointCloud2`)

    Point cloud with points inside body bounding box removed.
    Turned on by `bounding_box/publish_cut_out_pointcloud` parameter.
- `scan_point_cloud_no_bsphere` (`sensor_msgs/PointCloud2`)
    
    Point cloud with points inside body bounding sphere removed.
    Turned on by `bounding_sphere/publish_cut_out_pointcloud` parameter.
- `robot_bounding_sphere` (`robot_body_filter/SphereStamped`) 

    Bounding sphere of the robot body. Turned on by 
    `bounding_sphere/compute` parameter.
- `robot_bounding_box` (`geometry_msgs/PolygonStamped`) 

    Bounding box of the robot body. First point is the minimal point, second one
    is the maximal point. Turned on by `bounding_box/compute` parameter.

### Provided Services

- `~reload_model` (`std_srvs/Trigger`)

    Call this service to re-read the URDF model from parameter server.

### Filter Parameters

Have a look in the [examples](examples) folder to get inspiration for 
configuration of your filter. 

- `sensor/point_by_point` (`bool`, default: `false` for PointCloud2 version, 
   `true` for LaserScan)
    
    If true, suppose that every point in the scan was captured at a
    different time instant. Otherwise, the scan is assumed to be taken at
    once. If this is true, the processing pipeline expects the pointcloud
    to have fields int32 index, float32 stamps, and float32 vp_x, vp_y
    and vp_z viewpoint positions. If one of these fields is missing,
    computeMask() throws runtime exception.
- `frames/fixed` (`string`, default: `"base_link"`)

    The fixed frame. Usually base_link for stationary robots (or sensor
    frame if both robot and sensor are stationary). For mobile robots, it
    can be e.g. odom or map. Only needed for point-by-point scans.
- `frames/sensor` (`string`, default `""`)

    Frame of the sensor. If set to empty string, it will be read from
    `header.frame_id` of each incoming message. In LaserScan version, if
    nonempty, it has to match the `frame_id` of the incoming scans.
    In PointCloud2 version, the data can come in a different frame from
    `frames/sensor`.
- `frames/filtering` (`string`, default: `frames/fixed`)

    Frame in which the filter is applied. For point-by-point scans, it
    has to be a fixed frame, otherwise, it can be the sensor frame or
    any other frame. Setting to sensor frame will save some computations,
    but this frame cannot be empty string, so sensor frame can only be
    used if all data are coming from a single sensor and you know the scan
    frame in advance.
- `frames/output` (`string`, default: `frames/filtering`)

    Frame into which output data are transformed. Only applicable in
    PointCloud2 version.
- `sensor/min_distance` (`float`, default: `0.0 m`)

    The minimum distance of points from the laser to keep them.
- `sensor/max_distance` (`float`, default: `0.0 m`)

    The maximum distance of points from the laser to keep them. Set to zero
    to disable this limit.
- `body_model/inflation/scale` (`float`, default `1.0`)

    A scale that is applied to the collision model for the purposes of
    collision checking.
- `body_model/inflation/padding` (`float`, default `0.0 m`)

    Padding to be added to the collision model for the purposes of collision 
    checking.
- `body_model/robot_description_param` (`string`, default: `"robot_description"`)

    Name of the parameter where the robot model can be found.
- `filter/keep_clouds_organized` (`bool`, default `true`)

    Whether to keep pointclouds organized (if they were). Organized cloud has 
    `height > 1`.
- `filter/model_pose_update_interval` (`float`, default `0.0 s`)

    The interval between two consecutive model pose updates when processing a 
    pointByPointScan. If set to zero, the model will be updated for each point 
    separately (might be computationally exhaustive). If non-zero, it will only 
    update the model once in this interval, which makes the masking algorithm a 
    little bit less precise but more computationally affordable.
    
- `filter/do_clipping` (`bool`, default `true`)

    If `true`, the filter will mark points outside `sensor/(min|max)_distance` 
    as `CLIP`. If `false`, such points will be marked `OUTSIDE`.
    
- `filter/do_contains_test` (`bool`, default `true`)

    If `true`, the filter will mark points inside robot body 
    as `INSIDE`. If `false`, such points will be marked `OUTSIDE`.
    
- `filter/do_shadow_test` (`bool`, default `true`)

    If `true`, the filter will mark points shadowed by robot body 
    as `SHADOW`. If `false`, such points will be marked `OUTSIDE`.
    
- `transforms/buffer_length` (`float`, default `60.0 s`)

    Duration for which transforms will be stored in TF buffer.
- `transforms/timeout/reachable` (`float`, default `0.1 s`)

    How long to wait while getting reachable TF (i.e. transform which was 
    previously available). Please note that this timeout is computed not from
    the lookup start time, but from the scan timestamp - this allows you to tell
    how old scans you still want to process.
- `transforms/timeout/unreachable` (`float`, default `0.2 s`)
    
    How long to wait while getting unreachable TF.
- `transforms/require_all_reachable` (`bool`, default `false`)

   If true, the filter won't publish anything until all transforms are reachable.
- `bounding_sphere/compute` (`bool`, default `false`)

    Whether to compute and publish bounding sphere.
- `bounding_box/compute` (`bool`, default `false`)
 
    Whether to compute and publish bounding box.
- `bounding_sphere/publish_cut_out_pointcloud` (`bool`, default `false`)

    Whether to compute and publish pointcloud from which points in the
    bounding sphere are removed. Will be published on 
    `scan_point_cloud_no_bsphere`. Implies `bounding_box/compute`.
- `bounding_box/publish_cut_out_pointcloud` (`bool`, default `false`)

    Whether to compute and publish pointcloud from which points in the
    bounding box are removed. Will be published on `scan_point_cloud_no_bbox`.
    Implies `bounding_sphere/compute`.
- `ignored_links/bounding_sphere` (`list[string]`, default `[]`)

    List of links to be ignored in bounding sphere computation. Can be either 
    names of links or a combination of link name and collision index, e.g. 
    `link::1`, or link name and collision name, e.g. `link::my_collision`.
- `ignored_links/bounding_box` (`list[string]`, default `[]`)

    List of links to be ignored in bounding box computation. Same naming rules 
    as above.
- `ignored_links/contains_test` (`list[string]`, default `[]`)

    List of links to be ignored when testing if a point is inside robot body. 
    Same naming rules as above.
- `ignored_links/shadow_test` (`list[string]`, default `['laser']`)

    List of links to be ignored when testing if a point is shadowed by robot 
    body. Same naming rules as above. It is essential that this list contains 
    the sensor link - otherwise all points would be shadowed by the sensor 
    itself. 
- `ignored_links/everywhere` (`list[string]`, default `[]`)

    List of links to be completely ignored. Same naming rules as above.
- `only_links` (`list[string]`, default `[]`)

    If non-empty, only the specified links will be processed. The 
    above-mentioned `ignored_links/*` filters will still be applied.
- `body_model/dynamic_robot_description/field_name` (`string`, 
    default `robot_model`)

    If robot model is published by dynamic reconfigure, this is the name of the 
    Config message field which holds the robot model.
- `cloud/point_channels` (`list[string]`, default `["vp_"]`)

    List of channels of the incoming pointcloud that should be transformed as
    positional data. The 3D positions channel given by fields `x`, `y` and `z`
    is always transformed. This list contains prefixes of fields that form another
    channel(s). E.g. to transform the channel given by fields `vp_x`, `vp_y` and
    `vp_z`, add item `"vp_"` to the list. If a channel is not present in the cloud,
    nothing happens. This parameter is only available in the `PointCloud2` version
    of the filter.
- `cloud/direction_channels` (`list[string]`, default `["normal_"]`)

    List of channels of the incoming pointcloud that should be transformed as
    directional data. This list contains prefixes of fields that form channel(s).
    E.g. to transform the channel given by fields `normal_x`, `normal_y` and
    `normal_z`, add item `"normal_"` to the list. If a channel is not present in the cloud,
    nothing happens. This parameter is only available in the `PointCloud2` version
    of the filter.

## Debug Operation

These options are there to help correctly set up and debug the filter operation and should be turned off in production environments since they can degrade performance of the filter.

### Published Topics

- `robot_bounding_sphere_marker` (`visualization_msgs/Marker`) 

    Marker of the bounding sphere of the robot body. Turned on by 
    `bounding_sphere/marker` parameter.
- `robot_bounding_box_marker` (`visualization_msgs/Marker`) 

    Marker of the bounding box of the robot body. Turned on by 
    `bounding_box/marker` parameter.
- `robot_bounding_sphere_debug` (`visualization_msgs/MarkerArray`)

    Marker array containing the bounding sphere for each collision element.
    Turned on by `bounding_sphere/debug` parameter.
- `robot_bounding_box_debug` (`visualization_msgs/MarkerArray`)

    Marker array containing the bounding box for each collision element.
    Turned on by `bounding_box/debug` parameter.
- `robot_model_for_contains_test` (`visualization_msgs/MarkerArray`)

    Marker array containing the exact robot model used for contains tests.
    Turned on by `debug/marker/contains` parameter.
- `robot_model_for_shadow_test` (`visualization_msgs/MarkerArray`)

    Marker array containing the exact robot model used for shadow tests.
    Turned on by `debug/marker/shadow` parameter.
- `scan_point_cloud_inside` (`sensor_msgs/PointCloud2`)

    Debugging pointcloud with points classified as `INSIDE`. Turned on by
    `debug/pcl/inside` parameter.
- `scan_point_cloud_clip` (`sensor_msgs/PointCloud2`)

    Debugging pointcloud with points classified as `CLIP`. Turned on by
    `debug/pcl/clip` parameter.
- `scan_point_cloud_shadow` (`sensor_msgs/PointCloud2`)

    Debugging pointcloud with points classified as `SHADOW`. Turned on by
    `debug/pcl/shadow` parameter.

### Filter Parameters

- `bounding_sphere/debug` (`bool`, default `false`)

    Whether to compute and publish debug bounding spheres (marker array of 
    spheres for each collision).
- `bounding_box/debug` (`bool`, default `false`)

    Whether to compute and publish debug bounding boxes (marker array of boxes
    for each collision).
- `debug/pcl/inside` (`bool`, default `false`)

    Whether to publish debugging pointcloud with points marked as `INSIDE`.
- `debug/pcl/clip` (`bool`, default `false`)

    Whether to publish debugging pointcloud with points marked as `CLIP`.
- `debug/pcl/shadow` (`bool`, default `false`)

    Whether to publish debugging pointcloud with points marked as `SHADOW`.
- `debug/marker/contains` (`bool`, default `false`)

    Whether to publish debugging marker array containing the exact robot body 
    model used for containment test.
- `debug/marker/shadow` (`bool`, default `false`)

    Whether to publish debugging marker array containing the exact robot body 
    model used for shadow test.
