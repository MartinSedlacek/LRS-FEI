# Improved project simple automatic mission

In this assignemnt the goal is to further improve 1st assignemnt. In the next section there is a list of features that you need to integrate into the simple automatic mission (you need to pick at least 2).

## Features to add

1. Use more advanced planning algorithm possible choices are: RRT, A* (we can discuss with you to add more algorithms).
2. Use pointcloud to plan the trajectory, the trajectory planning needs to be done in 3D. 
3. Implement motion control with velocity inputs. 
4. Implement additional mission tasks: 
   - Stop (the UAV will stop and wait for command to continue, during the stop other mission commands must work) 
   - Continue (continue with the predifined trajectory)
   - Circle (the UAV will make a circle around the position from which the command was called)  

To satisfy this project you can choose to implement any combination of these tasks (but at least 2 needs to be done). Also in this assignemnt you need to explain the algorithms used in detail.

### **Assignment points**
- Each feature will be awarded 1/2 of total points (35).
- **Minimal requirement is to have at least of 50% of points from this assignment, it is possible to achieve this by completing only one feature fully functional.**
- To get all the points your drone should be able to complete the mission "autonomously". 

### Tasks explained

#### Advanced planning algorithm
Implement or integrate the algorithm into the planning trajectory algorithm from your 1st assignemnt. 

RRT explained in 2D: https://theclassytim.medium.com/robotic-path-planning-rrt-and-rrt-212319121378
RRT and RRT* implemented in 3D: https://github.com/motion-planning/rrt-algorithms

#### Pointcloud and planning in 3D
You have to work with this map: https://github.com/MartinSedlacek/LRS-FEI/blob/2nd_project/maps/FEI_LRS_PCD/map.pcd
To load this map you can use PCL library. 

To install the library you can use: `sudo apt install libpcl-dev`, to use the library to load the pointcloud you can refer to the tutorial for example: https://pcl.readthedocs.io/projects/tutorials/en/latest/reading_pcd.html. 

To further process the pointcloud to voxel grid, you can refer to: https://pcl.readthedocs.io/projects/tutorials/en/latest/voxel_grid.html

#### Position control with velocity inputs
TBD
#### Additional mission tasks
You need to implement all of these tasks:
   - Stop (the UAV will stop and wait for command to continue, during the stop other mission commands must work) 
   - Continue (continue with the predifined trajectory)
   - Circle (the UAV will make a circle around the position from which the command was called)  

The command Continue should be triggered by external topic or service or another input to your program, it is up to you.
