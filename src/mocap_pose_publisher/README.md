mocap_pose_publisher

This package publishes pose information for rigid bodies received from a mocap system.

New: Odometry topics

- For each rigid body named <name> the node now publishes an odometry message on:
  - `rigidbody_<name>/odom`
  - The message's header is copied from the incoming mocap message.
  - `child_frame_id` is set to the rigid body name.
  - Pose is copied from the mocap pose. Twist is set to zero (no velocity data available).
  - Pose covariance is copied from the published PoseWithCovarianceStamped (placeholder values).
    - Twist (odom.twist) is now estimated via finite-differences of pose over time. This is a simple numerical derivative and may be noisy. Consider filtering or using mocap-provided velocities when available.
      - The node averages the last N twist estimates (default N=5) and publishes that mean on `odom.twist`. The window size is configurable via the `twist_window` parameter. This simple moving average reduces noise; consider more advanced filters if needed.

Topic naming follows the existing pose topic style: `rigidbody_<name>/pose` and `rigidbody_<name>/odom`.
