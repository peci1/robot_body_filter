# robot_self_filter

Filters the robot's body out of point clouds.

## Changes vs [PR2/robot_self_filter](https://github.com/PR2/robot_self_filter):
- Branch indigo-devel merged into master.
- Using all ray intersections instead of only the first one.
- Using all collision elements for each link instead of only the first one.
