include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",  -- ✅ `base_link`로 변경
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,  -- ✅ Odom 프레임 생성 활성화
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = true  -- ✅ IMU 데이터 활성화
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- ✅ Scan Matching 활성화
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.05)

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.optimize_every_n_nodes = 90  -- ✅ SLAM 최적화 활성화

POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5

return options

