include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_frame = "map",
  tracking_frame = "chassis",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry = true,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 30,
  num_point_clouds = 0,
  map_builder = MAP_BUILDER,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  use_nav_sat = false,
  trajectory_builder = TRAJECTORY_BUILDER,
  trajectory_publish_period_sec = 30e-3,
  pose_publish_period_sec = 5e-3,
  publish_frame_projected_to_2d = false,
  use_landmarks = false,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 50
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 8.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1


return options
