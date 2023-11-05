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

1. Setup swarm simulation.
- You will use different world to simulate the swarm.
- Follow this tutorial to setup the world:
