# Swarm control
In this assignment you will have to control UAV swarm consisting of at least 3 UAVs.

## Project specification
1. Setup the swarm simulation.
- You will need x instances of SITL and MAVROS, your each simulated UAV will have unique ros namespace. 

2. Implement drone control ROS node to handle several UAVs. 
- You need to handle important messages and services for each of the UAV - local position, arm, land, takeoff. 

3. Implement mechanism to control the swarm.
- You can implement "follow me mode" (you can control only the leader and all other UAV will follow it with some offset) or you can implement swarm control (you will control all UAV in the swarms with the same commands).

4. Implement safety checks. 
- You need to monitor each incoming local pose, if some UAV will stop sending its position you need to handle the event with a stop to all drones.

 
### **Assignment points**
- Each section will be awarded 1/4 of total points (35), points for the last section will be awarded only if other sections are completed.
- **Minimal requirement is to have at least of 50% of points from this assignment, it is possible to achieve this by completing 1. and 2. part of the mission and demostrating automatic takeoff of all of the UAVs.**
- To get all the points your drone should be able to complete your predefined mission.

Video of example swarm in simulation:   ([video](../resources/swarm.mp4))

### Tasks explained

#### Setup swarm simulation.
- You will use different world to simulate the swarm.
- Follow this tutorial to setup the world:

1. Clone latest FEI-LRS repository and test this command `gazebo LRS-FEI/worlds/iris_arducopter_runway.world`.
2. Update path to new models. 
- if the project was cloned to home folder you can add this line `export GAZEBO_MODEL_PATH="/home/lrs-ubuntu/LRS-FEI/models:$GAZEBO_MODEL_PATH"` to .bashrc
3. Open 3 terminal windows and launch 3 ardupilot sitl.
```
cd ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I1
```
```
cd ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I2 --sysid 2
```
```
cd ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I3 --sysid 3
```
4. Open another 3 terminals and launch mavros 3 times.

```
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://127.0.0.1:14561@14561 -p tgt_system:=1 --remap __ns:=/drone1
```
```
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://127.0.0.1:14571@14575 -p tgt_system:=2 --remap __ns:=/drone2
```
```
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://127.0.0.1:14581@145581 -p tgt_system:=3 --remap __ns:=/drone3
```
5. Now you are able to obtain informations from each UAV individually, please beware that the topic names are different, you can check them with `ros2 topic list`
