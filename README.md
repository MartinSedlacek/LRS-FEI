# LRS-FEI

## Setup simulation with LRS-Ubuntu image
1. Open terminator with LRS layout. 
2. In 1st terminal launch gazebo: `gazebo <path_to_world>/fei_lrs_gazebo.world`
3. In 2nd terminal launch ArduPilot SITL: 
```
cd ardupilot/ArduCopter
sim_vehicle.py -f gazebo-iris --console -l 48.15084570555732,17.072729745416016,150,0
```
4. Launch mavros `ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://127.0.0.1:14551@14555`

## Simple takeoff and position control in GUIDED mode

1. Set mode.
2. Arm. 
3. Take off. 
4. Position control by waypoints.

```
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: GUIDED}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: True}"
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{min_pitch: 0, yaw: 90, altitude: 2}"

ros2 topic pub /mavros/setpoint_raw/local mavros_msgs/msg/PositionTarget '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "",}, coordinate_frame: 1, type_mask: 0, position: {x: 0.0, y: 0.0, z: 0.0}, velocity: {x: 0.0, y: 0.0, z: 0.0}, acceleration_or_force: {x: 0.0, y: 0.0, z: 0.0}, yaw: 0.0, yaw_rate: 0.0}'
```

In the last command you need to set coordinate_frame and type_mask.
Refer to mavlink manual, that will be used in the class often.

https://mavlink.io/en/messages/common.html#POSITION_TARGET_LOCAL_NED
