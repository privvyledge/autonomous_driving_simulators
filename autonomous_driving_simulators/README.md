# Obstacle Avoidance Pipeline
## Obstacle State (Ground Truth)
### ROS Topics
* `/carla/objects` - ObjectArray message [http://docs.ros.org/en/lunar/api/derived_object_msgs/html/msg/ObjectArray.html]. Obstacle state [Pose, Twist, Accel, Polygon, Shape, Class, Certainty]
* `/carla/markers` - MarkerArray message [http://docs.ros.org/en/lunar/api/visualization_msgs/html/msg/MarkerArray.html]. Obstacle state [Pose, Shape, Class, Certaint/]
* `/carla/ego_vehicle/collision` - Collision message [http://docs.ros.org/en/lunar/api/carla_msgs/html/msg/CollisionEvent.html]. Collision state [ActorId, OtherActorId, NormalImpulse, TimeStamp]
* `/carla/egovehicle/laneinvasion` - LaneInvasion message [http://docs.ros.org/en/lunar/api/carla_msgs/html/msg/LaneInvasionEvent.html]. LaneInvasion state [ActorId, OtherActorId, Type, TimeStamp]

To spawn an obstacle in CARLA, use the following command in the docker container:
```bash
$ ros2 launch autonomous_driving_simulators carla_bringup.launch.py
$ python3 ~/carlasimulator/PythonAPI/examples/generatetraffic.py --safe
```