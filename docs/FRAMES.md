# Estimator inputs and OCP coordinates

The core accepts world position and linear velocity, body-to-world quaternion (wxyz), and body angular velocity in rad/s. World axes are ENU and body axes are FLU. OCP selection never changes the estimator subscription.

Descriptors select `World` or `TargetRelative`. Target-relative subtracts target position and velocity with world-aligned axes; target yaw does not rotate the state. RH and MPC share the conversion. RH commands reconstruct with the live target; planned paths reconstruct with constant-acceleration target prediction.

ROS1 defaults are `drone_odom_topic=/mavros/local_position/odom`, `world_frame=map`, `state_velocity_frame=body`, `target_odom_topic=/target/odom`, `target_velocity_frame=world`, and `command_frame=map`. Velocity conventions accept only `body` or `world`. The synthetic ROS1 target publishes in `map`. ROS2 Crazyflie retains its native pose/body-velocity adapter in `world`; a configured external odometry source supplies full SI state.

For another estimator, select its odometry topic and velocity convention. Provide a calibrated static TF from its pose frame to the configured world frame, and align target odometry and acceleration likewise. Input transforms are queried at message timestamps. Missing frames or TF reject the message; freshness checks prevent continued use of stale vehicle state. MAVROS full-state commands transform back into the PX4 local ENU command frame. Moving coordinate frames require additional velocity transport terms and are outside this contract.

Mocap can feed PX4 through MAVROS vision/mocap for fused local odometry, or feed an external full-state estimator. Raw pose alone lacks velocity and body rates. Vehicle and target origins and axes must be calibrated consistently.

Removed: shifted and rotating relative input modes, relative vehicle topic switches, and target-publisher planning-frame output switches. New solver logs use `absolute` or `relative`; historical CSVs remain readable. Frame choice does not certify physical constraint feasibility.
