# robot_body_filter

Filters the robot's body out of point clouds and laser scans.

## Tutorial

Check out the webinar recording where a lot of options for this filter are explained and demonstrated! https://www.youtube.com/watch?v=j0ljV0uZy3Q

## Changes vs [PR2/robot_self_filter](https://github.com/PR2/robot_self_filter):
- Now the package is a normal `filters::FilterBase` filter and not a standalone node.
- Using both containment and ray-tracing tests. 
- Using all collision elements for each link instead of only the first one.
- Enabling generic point type, removing PCL dependency and unnecessary params.
- Using bodies.h and shapes.h from geometric_shapes.
- As a by-product, the filter can compute robot's bounding box or sphere.

## Build Status




Development versions:
[![Github Actions](https://github.com/peci1/robot_body_filter/workflows/CI/badge.svg?branch=master)](https://github.com/peci1/robot_body_filter/actions?query=workflow%3ACI)
[![Dev melodic](http://build.ros.org/job/Mdev__robot_body_filter__ubuntu_bionic_amd64/badge/icon?subject=melodic)](http://build.ros.org/job/Mdev__robot_body_filter__ubuntu_bionic_amd64)
[![Dev noetic](http://build.ros.org/job/Ndev__robot_body_filter__ubuntu_focal_amd64/badge/icon?subject=noetic)](http://build.ros.org/job/Ndev__robot_body_filter__ubuntu_focal_amd64)

Release jobs Melodic
[![Melodic version](https://img.shields.io/ros/v/melodic/robot_body_filter)](http://packages.ros.org/ros/ubuntu/pool/main/r/ros-melodic-robot-body-filter/):
[![Bin melodic-amd64](https://build.ros.org/job/Mbin_uB64__robot_body_filter__ubuntu_bionic_amd64__binary/badge/icon?subject=bionic+amd64)](https://build.ros.org/job/Mbin_uB64__robot_body_filter__ubuntu_bionic_amd64__binary/)
[![Bin melodic-arm64](https://build.ros.org/job/Mbin_ubv8_uBv8__robot_body_filter__ubuntu_bionic_arm64__binary/badge/icon?subject=bionic+arm64)](https://build.ros.org/job/Mbin_ubv8_uBv8__robot_body_filter__ubuntu_bionic_arm64__binary/)
[![Bin melodic-armhf](https://build.ros.org/job/Mbin_ubhf_uBhf__robot_body_filter__ubuntu_bionic_armhf__binary/badge/icon?subject=bionic+armhf)](https://build.ros.org/job/Mbin_ubhf_uBhf__robot_body_filter__ubuntu_bionic_armhf__binary/)

Release jobs Noetic
[![Noetic version](https://img.shields.io/ros/v/noetic/robot_body_filter)](http://packages.ros.org/ros/ubuntu/pool/main/r/ros-noetic-robot-body-filter/):
[![Bin noetic focal-amd64](https://build.ros.org/job/Nbin_uF64__robot_body_filter__ubuntu_focal_amd64__binary/badge/icon?subject=focal+amd64)](https://build.ros.org/job/Nbin_uF64__robot_body_filter__ubuntu_focal_amd64__binary/)
[![Bin noetic focal-arm64](https://build.ros.org/job/Nbin_ufv8_uFv8__robot_body_filter__ubuntu_focal_arm64__binary/badge/icon?subject=focal+arm64)](https://build.ros.org/job/Nbin_ufv8_uFv8__robot_body_filter__ubuntu_focal_arm64__binary/)
[![Bin noetic focal-armhf](https://build.ros.org/job/Nbin_ufhf_uFhf__robot_body_filter__ubuntu_focal_armhf__binary/badge/icon?subject=focal+armhf)](https://build.ros.org/job/Nbin_ufhf_uFhf__robot_body_filter__ubuntu_focal_armhf__binary/)

## Basic Operation

### `filters::FilterBase` API

The basic workings of this filter are done via the [`filters::FilterBase` API](http://wiki.ros.org/filters) implemented for `sensor_msgs::LaserScan` and `sensor_msgs::PointCloud2` types. This means you can load this filter into a FilterChain along other filters as usual. Different from the standard filters, this one can also publish several interesting topics and subscribes to TF.

### General overview

This filter reads robot model and the filter config, subscribes to TF, waits
for data (laserscans or point clouds) and then cleans them from various
artifacts (this is called data filtering).

It can perform 3 kinds of data filters: **clip** the data based on the provided
sensor limits (parameter `filter/do_clipping`), **remove points that are inside
or on the surface** of the robot body (parameter `filter/do_contains_test`) and
**remove points that are seen through a part of the robot body** (parameter
`filter/do_shadow_test`). These kinds of tests are further referenced as 
"clipping", "contains test" and "shadow test".

If working with point clouds, the filter automatically recognizes whether it
works with organized or non-organized clouds. In organized clouds, it marks
the filtered-out points as `NaN`. In non-organized clouds, it removes the
filtered-out points. In laserscans, removal is not an option, so the
filtered-out points are marked with `NaN` (some guides suggest that
`max_range + 1` should be used for marking invalid points, but this filter uses
`NaN` as a safer option).

#### Performance tips
In general, the filter will be computationally expensive (clipping is fast,
contains test is medium CPU intensive and shadow test is the most expensive
part, because it basically performs raytracing). You can limit the required CPU
power by limiting the filter only to parts that matter. E.g. if the robot has a
link that can never be seen by the sensor, put it in the list of ignored links.
The less links are processed, the better performance. If you're only interested
in removing a few links, consider using the `only_links` parameter.

To speed up shadow filtering, you can set `filter/max_shadow_distance`, which
limits the number of points considered for shadow tests just to points close to
the sensor. Setting this to e.g. three times the diameter of the robot should
remove all of the shadow points caused by refraction by a part of the robot body.
But you have to test this with real data.

Performance also strongly depends on representation of the robot model.
The filter reads `<collision>` tags from the robot URDF. You can use boxes,
spheres and cylinders (which are fast to process), or you can use **convex**
meshes (these are much worse performance-wise). If you pass a non-convex mesh,
its convex hull will be used for the tests. Don't forget that each link
can have multiple `<collision>` tags. If you do not have time to convert your
meshes to the basic shapes, try to at least reduce the number of triangles
in your meshes. You can use your high-quality meshes in `<visual>` tags. To simplify your model to primitive shapes, you can either manually edit the URDF, or you can utilize [ColliderGen](https://github.com/cole-bsmr/collidergen).

#### Model inflation

You can utilize the builtin model inflation mechanism to slightly alter the
size of the model. You will probably want to add a bit "margin" to the contains
and shadow tests so that points that are millimeters outside the robot body will
anyways get removed. You can set a default scale and padding which are used for
all collisions. Different inflation can be used for contains tests and for
shadow tests. Inflation can also be specified differently for each link. Look
at the `body_model/inflation/*` parameters for details.

Scaling means multiplying the shape dimensions by the given
factor (with its center staying in the same place). Padding means adding the
specified metric distance to each "dimension" of the shape. Padding a sphere
by `p` just adds `p` to its radius; padding a cylinder adds `p` to its radius
and `2p` to its length (you pad both the top and the bottom of the cylinder);
padding a box adds `2p` to all its extents (again, you pad both of the opposing
sides); padding a mesh pads each vertex of its convex hull by `p` along the
direction from the mesh center to the vertex (mesh center is the center
specified in the mesh file, e.g. DAE). Have a good look at the effects of mesh
padding, as the results can be non-intuitive.

#### Data acquisition modes

The filter supports data captured in two modes - all at once (e.g. RGBD cameras),
or each point at a different time instant (mostly lidars). This is handled by
`sensor/point_by_point` setting. Each mode supports both laser scans and point
cloud input (although all-at-once laserscans aren't that common). Point-by-point
pointclouds are e.g. the output of 3D lidars like Ouster (where more points can
have the same timestamp, but not all). If you want to use the point-by-point
mode with pointclouds, make sure they contain not only the usual `x`, `y` and
`z` fields, but also a `float32` field `stamps` (with time difference from the
time in the header) and `float32` fields `vp_x`, `vp_y` and `vp_z` which contain
the viewpoint (position of the sensor in filtering frame) from which the robot
saw that point. 

When filtering in the point-by-point mode, the robot posture has to be updated
several times during processing a single scan (to reflect the motion the robot
has performed during acquiring the scan). The frequency of these updates can
also have a significant impact on performance. Use parameter
`filter/model_pose_update_interval` to set the interval for which the robot is
considered stationary. The positions of the robot at the beginning and at the
end of the scan are queried from TF. The intermediate positions are linearly
interpolated between these two positions.

#### TF frames

The filter recognizes four logical TF frames (some of which may be the same
physical frames).

**Fixed frame** is a frame that doesn't change within the duration of the
processed scan. This means for all-at-once scans, this frame is not needed
because the duration of the scan is zero. For point-by-point scans, it depends
on the particular scenario. In static installations (like manipulators with
sensors not attached to them), it can be the sensor frame. For stationary robots
with the sensor attached to a movable part of their body, `base_link` will be a
good choice. For completely mobile robots, you will need an external frame, e.g.
`odom` or `map` (beware of cyclic dependencies - if the map is built from the
filtered scans, you obviously cannot use `map` as the fixed frame for filtering
the scans...).

**Sensor frame** is the frame in which the data were captured. Generally, it
would be whatever is in `header.frame_id` field of the processed messages. You
can use the filter for data from multiple sensors - in that case, you can leave
the sensor frame unfilled and each message will be processed in the frame it has
in its header.

**Filtering frame** is the frame in which the data filtering is done. For
point-by-point scans, it has to be a fixed frame (possibly different from the
fixed frame set in `frames/fixed`). For pointcloud scans, it should be the
sensor frame (if all data are coming from a single sensor), or any other frame.
It is also used as the frame in which all debugging outputs are published.

**Output frame** can only be used with pointcloud scans, and allows to transform
the filtered pointcloud to a different frame before being published. It is just
a convenience which can save you launching a transformation nodelet. By default,
filtered pointclouds are output in the filtering frame.

#### Bounding shapes

As a byproduct, the filter can also compute various bounding shapes of the robot
model. There are actually four robot models - one for contains test, one
for shadow test, one for bounding sphere computation and one for bounding box
(these models can differ by inflation and considered links).
All bounding shapes are published in the filtering frame. For point-by-point scans,
the bounding shapes correspond to the
time instant specified in the header of the processed scan. The computation of
bounding shapes is off by default, but enabling it is cheap (performance-wise).

The **bounding sphere** is easy - the smallest sphere that contains the whole
collision model for bounding sphere computation (with the specified exclusions removed).

The **bounding box** is the smallest axis-aligned bounding box aligned to the
filtering frame. It is built from the model for bounding box computation.

The **local bounding box** is the smallest axis-aligned bounding box aligned to
the frame specified in `local_bounding_box/frame_id`. It is especially useful
with mobile robots when the desired frame is `base_link`. It is built from the model
for bounding box computation.

The **oriented bounding box** should be the smallest box containing the
collision model. However, its computation is very bad conditioned, so the
results can be very unsatisfactory. Currently, the oriented bounding box of each
of the basic collision shapes is "tight", but merging the boxes is not optimal.
A good algorithm would probably require costly and advanced iterative methods.
The current implementation uses FCL in the background and merges the boxes using
`fcl::OBB::operator+=()` without any further optimizations. It is built from the
model for bounding box computation.

The filter also supports publishing auxiliary pointclouds which "cut out" each
of these bounding shapes. These are the input data converted to pointcloud in
filtering frame from which all points belonging to the bounding shape are
removed. Please note that the "base" used for cutting out is the input
pointcloud, not the filtered one.

#### First setup/debugging

The filter offers plenty of debugging outputs to make sure it does exactly
what you want it to do. All the options are described in the last part of
this page. Generally, you should look at the pointclouds visualizing which
points got filtered out and you should also check the robot models used for
filtering.

Also, have a look in the [examples] folder to get some inspiration.

### Usage

This is a standard `filters::FilterBase<T>`-based filter which implements the
`configure()` and `update(const T&, T&)` methods. This means it can be loaded
e.g. as a part of a filter chain via the
[`laser_filters`](https://github.com/ros-perception/laser_filters) package or
the relatively new [`sensor_filters`](https://github.com/ctu-vras/sensor_filters).
This means the input and output data are not supplied in the form of topics,
but they are instead passed to the `update()` method via the C++ API.

This filter is a bit unusual - it subscribes and publishes several topics.
Normally, filters only operate via the `update()` method and are not expected
to publish anything. This filter is different, it requires a working ROS node
handle, and can publish a lot of auxiliary or debug data.

### Subscribed Topics

- `/tf`, `/tf_static`

    Transforms
- `dynamic_robot_model_server/parameter_updates` (`dynamic_reconfigure/Config`)

    Dynamic reconfigure topic on which updated robot model can be subscribed. 
    The model is read from a field defined by parameter 
    `body_model/dynamic_robot_description/field_name`.

### Published Topics

- `scan_point_cloud_no_bsphere` (`sensor_msgs/PointCloud2`)
    
    Point cloud with points inside body bounding sphere removed.
    Turned on by `bounding_sphere/publish_cut_out_pointcloud` parameter.
    Published in filtering frame.
- `scan_point_cloud_no_bbox` (`sensor_msgs/PointCloud2`)

    Point cloud with points inside filtering frame axis-aligned bounding box
    removed. Turned on by `bounding_box/publish_cut_out_pointcloud` parameter.
    Published in filtering frame.
- `scan_point_cloud_no_oriented_bbox` (`sensor_msgs/PointCloud2`)

    Point cloud with points inside oriented bounding box removed. Turned on by
    `oriented_bounding_box/publish_cut_out_pointcloud` parameter.
    Published in filtering frame. Please read the remarks above in the overview -
    the results can be non-satisfying.
- `scan_point_cloud_no_local_bbox` (`sensor_msgs/PointCloud2`)

    Point cloud with points inside `local_bounding_box/frame_id` axis-aligned
    bounding box removed. Turned on by
    `local_bounding_box/publish_cut_out_pointcloud` parameter. Published in
    filtering frame.
- `robot_bounding_sphere` (`robot_body_filter/SphereStamped`) 

    Bounding sphere of the robot body. Turned on by 
    `bounding_sphere/compute` parameter. Published in filtering frame.
- `robot_bounding_box` (`geometry_msgs/PolygonStamped`) 

    Axis-aligned bounding box of the robot body aligned to the filtering frame.
    First point is the minimal point, second one is the maximal point. Turned on
    by `bounding_box/compute` parameter. Published in filtering frame.
- `robot_oriented_bounding_box` (`robot_body_filter/OrientedBoundingBoxStamped`)

    Oriented bounding box of the robot body. Turned on by
    `oriented_bounding_box/compute` parameter. Please read the remarks above
    in the overview - the results can be non-satisfying. Published in filtering
    frame.
- `robot_local_bounding_box` (`geometry_msgs/PolygonStamped`) 

    Axis-aligned bounding box of the robot body aligned to frame
    `local_bounding_box/frame_id`. First point is the minimal point, second one
    is the maximal point. Turned on by `local_bounding_box/compute` parameter.
    Published in frame `local_bounding_box/frame_id`.

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
- `filter/max_shadow_distance` (`float`, default is the value of `sensor/max_distance`)

    If greater than zero, specifies the maximum distance of a point from the sensor
    frame within which the point can be considered for shadow testing. All further
    points are classified as `OUTSIDE`. Setting this parameter to a low value may 
    greatly improve performance of the shadow filtering.
- `body_model/inflation/scale` (`float`, default `1.0`)

    A scale that is applied to the collision model for the purposes of
    collision checking.
- `body_model/inflation/padding` (`float`, default `0.0 m`)

    Padding to be added to the collision model for the purposes of collision 
    checking.
- `body_model/inflation/contains_test/scale` (`float`, default `body_model/inflation/scale`)

    A scale that is applied to the collision model used for contains tests.
- `body_model/inflation/contains_test/padding` (`float`, default `body_model/inflation/padding`)

    Padding to be added to the collision model used for contains tests.
- `body_model/inflation/shadow_test/scale` (`float`, default `body_model/inflation/scale`)

    A scale that is applied to the collision model used for shadow tests.
- `body_model/inflation/shadow_test/padding` (`float`, default `body_model/inflation/padding`)

    Padding to be added to the collision model used for shadow tests.
- `body_model/inflation/bounding_sphere/scale` (`float`, default `body_model/inflation/scale`)

    A scale that is applied to the collision model used for bounding sphere computation.
- `body_model/inflation/bounding_sphere/padding` (`float`, default `body_model/inflation/padding`)

    Padding to be added to the collision model used for bounding sphere computation.
- `body_model/inflation/bounding_box/scale` (`float`, default `body_model/inflation/scale`)

    A scale that is applied to the collision model used for bounding box computation.
- `body_model/inflation/bounding_box/padding` (`float`, default `body_model/inflation/padding`)

    Padding to be added to the collision model used for bounding box computation.
- `body_model/inflation/per_link/scale` (`dict[str:float]`, default `{}`)

    A scale that is applied to the specified links for the purposes of
    collision checking. Links not specified here will use the default scale
    set in `body_model/inflation/contains_test/scale` or
    `body_model/inflation/shadow_test/scale`. Keys are names, values are scale.
    Names can be either names of links (`link`), names of collisions (
    `*::my_collision`), a combination of link name and zero-based collision
    index (`link::1`), or link name and collision name, e.g.
    `link::my_collision`. Any such name can have suffix `::contains`, 
    `::shadow`, `::bounding_sphere` or `::bounding_box`, which will only change
    the scale for contains or shadow tests or for bounding sphere or box computation.
    If a collision is matched by multiple entries, they have priority
    corresponding to the order they were introduced here (the entries do not
    "add up", but replace each other).
- `body_model/inflation/per_link/padding` (`dict[str:float]`, default `{}`)

    Padding to be added to the specified links for the purposes of collision 
    checking. Links not specified here will use the default padding set in
    `body_model/inflation/contains_test/padding` or
    `body_model/inflation/shadow_test/padding`. Keys are names, values are
    padding. Names can be either names of links (`link`), names of collisions (
    `*::my_collision`), a combination of link name and zero-based collision
    index (`link::1`), or link name and collision name, e.g.
    `link::my_collision`. Any such name can have suffix `::contains`,
    `::shadow`, `::bounding_sphere` or `::bounding_box`, which will only change
    the padding for contains or shadow tests or for bounding sphere or box computation.
    If a collision is matched by multiple entries, they have priority
    corresponding to the order they were introduced here (the entries do not
    "add up", but replace each other).
    
- `body_model/robot_description_param` (`string`, default: `"robot_description"`)

    Name of the parameter where the robot model can be found.
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
- `ignored_links/bounding_sphere` (`list[string]`, default `[]`)

    List of links to be ignored in bounding sphere computation. Can be either 
    names of links (`link`), names of collisions (`*::my_collision`), a
    combination of link name and zero-based collision index (`link::1`), or link
    name and collision name, e.g. `link::my_collision`.
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
- `ignored_links/bounding_sphere` (`list[string]`, default `[]`)

    List of links to be ignored when computing the bounding sphere.
    Same naming rules as above.
- `ignored_links/bounding_box` (`list[string]`, default `[]`)

    List of links to be ignored when computing the bounding box.
    Same naming rules as above.
- `ignored_links/everywhere` (`list[string]`, default `[]`)

    List of links to be completely ignored. Same naming rules as above.
- `only_links` (`list[string]`, default `[]`)

    If non-empty, only the specified links will be processed. The 
    above-mentioned `ignored_links/*` filters will still be applied.
- `bounding_sphere/compute` (`bool`, default `false`)

    Whether to compute and publish bounding sphere.
- `bounding_box/compute` (`bool`, default `false`)
 
    Whether to compute and publish axis-aligned bounding box aligned to
    filtering frame.
- `oriented_bounding_box/compute` (`bool`, default `false`)
 
    Whether to compute and publish oriented bounding box (see the remarks in
    the overview above; the result might be pretty bad).
- `local_bounding_box/compute` (`bool`, default `false`)
 
    Whether to compute and publish axis-aligned bounding box aligned to
    frame `local_bounding_box/frame_id`.
- `local_bounding_box/frame_id` (`str`, default: `frames/fixed`)
 
    The frame to which local bounding box is aligned.
- `bounding_sphere/publish_cut_out_pointcloud` (`bool`, default `false`)

    Whether to compute and publish pointcloud from which points in the
    bounding sphere are removed. Will be published on 
    `scan_point_cloud_no_bsphere`. Implies `bounding_sphere/compute`.
    The "base" point cloud before cutting out are the input data,
    not the filtered data.
- `bounding_box/publish_cut_out_pointcloud` (`bool`, default `false`)

    Whether to compute and publish pointcloud from which points in the
    bounding box are removed. Will be published on `scan_point_cloud_no_bbox`.
    Implies `bounding_box/compute`. The "base" point cloud before cutting out
    are the input data, not the filtered data.
- `oriented_bounding_box/publish_cut_out_pointcloud` (`bool`, default `false`)

    Whether to compute and publish pointcloud from which points in the
    oriented bounding box are removed. Will be published on
    `scan_point_cloud_no_oriented_bbox`. Implies `oriented_bounding_box/compute`.
    The "base" point cloud before cutting out are the input data, not the
    filtered data.
- `local_bounding_box/publish_cut_out_pointcloud` (`bool`, default `false`)

    Whether to compute and publish pointcloud from which points in the local
    bounding box are removed. Will be published on
    `scan_point_cloud_no_local_bbox`. Implies `local_bounding_box/compute`. The
    "base" point cloud before cutting out are the input data, not the filtered
    data.
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
    `bounding_sphere/marker` parameter. Published in filtering frame.
- `robot_bounding_box_marker` (`visualization_msgs/Marker`) 

    Marker of the bounding box of the robot body. Turned on by 
    `bounding_box/marker` parameter. Published in filtering frame.
- `robot_oriented_bounding_box_marker` (`visualization_msgs/Marker`) 

    Marker of the oriented bounding box of the robot body. Turned on by 
    `oriented_bounding_box/marker` parameter. Published in filtering frame.
- `robot_local_bounding_box_marker` (`visualization_msgs/Marker`) 

    Marker of the local bounding box of the robot body. Turned on by 
    `local bounding_box/marker` parameter. Published in frame
    `local_bounding_box/frame_id`.
- `robot_bounding_sphere_debug` (`visualization_msgs/MarkerArray`)

    Marker array containing the bounding sphere for each collision element.
    Turned on by `bounding_sphere/debug` parameter. Published in filtering frame.
- `robot_bounding_box_debug` (`visualization_msgs/MarkerArray`)

    Marker array containing the bounding box for each collision element.
    Turned on by `bounding_box/debug` parameter. Published in filtering frame.
- `robot_oriented_bounding_box_debug` (`visualization_msgs/MarkerArray`)

    Marker array containing the oriented bounding box for each collision element.
    Turned on by `oriented_bounding_box/debug` parameter. Published in filtering frame.
- `robot_local_bounding_box_debug` (`visualization_msgs/MarkerArray`)

    Marker array containing the local bounding box for each collision element.
    Turned on by `local_bounding_box/debug` parameter. Published in frame
    `local_bounding_box/frame_id`.
- `robot_model_for_contains_test` (`visualization_msgs/MarkerArray`)

    Marker array containing the exact robot model used for contains tests.
    Turned on by `debug/marker/contains` parameter.
- `robot_model_for_shadow_test` (`visualization_msgs/MarkerArray`)

    Marker array containing the exact robot model used for shadow tests.
    Turned on by `debug/marker/shadow` parameter.
- `robot_model_for_bounding_sphere` (`visualization_msgs/MarkerArray`)

    Marker array containing the exact robot model used for computation of
    bounding sphere. Turned on by `debug/marker/bounding_sphere` parameter.
- `robot_model_for_bounding_box` (`visualization_msgs/MarkerArray`)

    Marker array containing the exact robot model used for computation of
    bounding box. Turned on by `debug/marker/bounding_box` parameter.
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

- `bounding_sphere/marker` (`bool`, default `false`)

    Whether to publish a marker representing the bounding sphere.
- `bounding_box/marker` (`bool`, default `false`)

    Whether to publish a marker representing the axis-aligned bounding box
    aligned to the filtering frame.
- `oriented_bounding_box/marker` (`bool`, default `false`)

    Whether to publish a marker representing the oriented bounding box.
- `local_bounding_box/marker` (`bool`, default `false`)

    Whether to publish a marker representing the axis-aligned bounding box
    aligned to frame `local_bounding_box/frame_id`.
- `bounding_sphere/debug` (`bool`, default `false`)

    Whether to compute and publish debug bounding spheres (marker array of 
    spheres for each collision).
- `bounding_box/debug` (`bool`, default `false`)

    Whether to compute and publish debug bounding boxes (marker array of
    axis-aligned boxes for each collision).
- `oriented_bounding_box/debug` (`bool`, default `false`)

    Whether to compute and publish debug oriented bounding boxes (marker array
    of oriented boxes for each collision).
- `local_bounding_box/debug` (`bool`, default `false`)

    Whether to compute and publish debug axis-aligned bounding boxes aligned to
    frame `local_bounding_box/frame_id` (marker array of axis-aligned boxes for
    each collision).
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
- `debug/marker/bounding_sphere` (`bool`, default `false`)

    Whether to publish debugging marker array containing the exact robot body 
    model used for computing the bounding sphere.
- `debug/marker/bounding_box` (`bool`, default `false`)

    Whether to publish debugging marker array containing the exact robot body 
    model used for computing the bounding box.
