"""
Usage:
    ros2 launch autonomous_driving_simulators mpc.launch.py

ros2 launch trajectory_following_ros2 mpc.launch.py use_sim_time:=True robot_frame:=ego_vehicle global_frame:=map horizon:=30 frequency:=20.0 sample_time:=0.05 publish_twist_topic:=False wheelbase:=2.87528 max_steer:=69.99999284118222 min_steer:=-69.99999284118222 max_steer_rate:=120.0 max_speed:=20.5 min_speed:=-20.5 Q_diagonal:='[10.0, 10.0, 1.0, 0.01]' R_diagonal:='[0.01, 0.01]' Rd_diagonal:='[10., 100.]' Qf:='[4., 4., 1., 0.01]' max_iterations:=30 termination_condition:=0.00001 distance_tolerance:=15.0 speed_tolerance:=20.0 load_waypoints:=False mpc_toolbox:=do_mpc odom_topic:="/carla/ego_vehicle/odometry" ackermann_cmd_topic:="/drive" path_topic:="/trajectory/path" speed_topic:='/trajectory/speed"
"""