include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "laser_frame",
  published_frame = "laser_frame",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 5e-2,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.min_range = 0.2
TRAJECTORY_BUILDER_2D.max_range = 25.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 30.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

return options

-- ./sf45b
-- ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 laser laser_frame
--
-- ros2 run cartographer_ros cartographer_node -configuration_directory . -configuration_basename sf45b_2d.lua --ros-args -p provide_odom_frame:=true -p expected_sensor_ids:="[scan]" -r scan:=/scan_fixed
--
-- ros2 run slam_bridge scan_restamper --ros-args -p in_topic:=/scan -p out_topic:=/scan_fixed
--
-- ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05 -publish_period_sec 1.0
--
-- rviz2
--
