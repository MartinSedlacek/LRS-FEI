# ROS2 cheatsheet for the purposes of the LRS

## ROS2 tutorials

ROS2 Nodes -> https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html

ROS2 Topics -> https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html

Practical example for publisher and subscriber -> https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html


## Preliminary: Launch the Simulation
To start with examples please launch the simulation as descibed in [README](../README.md).

## Section 1: Creating and Utilizing a ROS2 Workspace

ROS2 workspace should have folder structure like this:
```
workspace_folder/              # Workspace root directory
├── src/                       # Source directory where all packages are stored
│   ├── package_1/             # First ROS 2 package
│   │   ├── src/               # Source files (like .cpp for C++)
│   │   ├── include/           # Header files
│   │   ├── msg/               # Custom message definitions (if any)
│   │   ├── srv/               # Custom service definitions (if any)
│   │   ├── action/            # Custom action definitions (if any)
│   │   ├── launch/            # Launch files
│   │   └── package.xml        # Package manifest
│   │
│   ├── package_2/             # Second ROS 2 package
│   │   ├── src/               # Source files
│   │   ├── include/           # Header files
│   │   ├── msg/               # Custom message definitions (if any)
│   │   ├── srv/               # Custom service definitions (if any)
│   │   ├── action/            # Custom action definitions (if any)
│   │   ├── launch/            # Launch files
│   │   └── package.xml        # Package manifest
│   │
│   └── ...
│
├── log/                       # Log files from builds and runs
├── build/                     # Build artifacts, not typically stored in VCS
├── install/                   # Install artifacts, not typically stored in VCS
└── build_and_install_logs/    # Optional: Local logs and build/install artifacts
```

1. Create your workspace using this general structure. 
2. Add your first package into src folder.
3. Go to the top of your workspace.
4. Run colcon build.
5. Your packages within src/<packages>, should be build.
6. Source your worskpace using `source install/setup.bash`
7. Now you can run packages with the command `ros2 run <package_1> <name_of_the_executable>` in our case it can be `ros2 run template_drone_control template_drone_control_node`
8. If you have changed some of the code, you need to build your workspace again using colcon build from workspace root directory.

## Section 2: Fundamental ROS2 Console Commands

Open new terminal, source your workspace with 'source <path_to_your_ws>/install/setup.bash

### List all available topic
Command `ros2 topic list`

Expected output: 
```
...
/mavros/local_position/accel
/mavros/local_position/odom
/mavros/local_position/pose
/mavros/local_position/pose_cov
/mavros/local_position/velocity_body
/mavros/local_position/velocity_body_cov
/mavros/local_position/velocity_local
/mavros/manual_control/control
/mavros/manual_control/send
...

```

This command will list all the available topic in ros2 instance. In general you can use this to check if the system is running as you would expect, or to verify the names of the topics.

There is similar command to list all the services -> `ros2 service list` 

### Topics Management

From the topic list lets choose important topic for LRS -> `/mavros/local_position/pose`.

In this topic the local position of the simulated UAV will be located.

#### Topics Management from command line
##### Check data available within this topic -> `ros2 topic echo /mavros/local_position/pose`

Expected output:
```
header:
  stamp:
    sec: 1696755038
    nanosec: 391547345
  frame_id: map
pose:
  position:
    x: 0.013228480704128742
    y: -0.013274122960865498
    z: -0.000252618920058012
  orientation:
    x: -4.2999677455014015e-05
    y: 0.0006802187081805384
    z: -0.6987245170632879
    w: -0.7153905120339602
---

```

##### Check the frequency of the topic -> `ros2 topic hz /mavros/local_position/pose`
Expected output:
```
average rate: 1.362
	min: 0.710s max: 0.750s std dev: 0.01760s window: 3
average rate: 1.350
	min: 0.710s max: 0.761s std dev: 0.01711s window: 5
average rate: 1.343
	min: 0.710s max: 0.761s std dev: 0.01567s window: 7
average rate: 1.344
	min: 0.710s max: 0.761s std dev: 0.01397s window: 9
```
##### Check the data type of the message (this can be used as a reference in the code) -> `ros2 topic info  /mavros/local_position/pose`

Expected output:
```
Type: geometry_msgs/msg/PoseStamped
Publisher count: 1
Subscription count: 1
```

As you can see, there is a type of the message. You can use this information to include the right header file for a message as well as data type of the message.

##### Check data field of the message (this can be used as a reference in the code) -> `ros2 interface show geometry_msgs/msg/PoseStamped`

Expected output:
```
# A Pose with reference coordinate frame and timestamp

std_msgs/Header header
Pose pose
```

Now you know that the message of type is composed of `std_msgs/msg/Header` and `geometry_msgs/msg/Pose`. You can further check the message for example using `ros2 interface show geometry_msgs/msg/Pose`. This way you can check all the available fields etc., for the message definition of standard types you can as well use documentation for example here you can check definition for `geometry_msgs` https://docs.ros2.org/latest/api/geometry_msgs/index-msg.html. For custom packages you can refer to their code in the case of mavros you can check definitions on their github: https://github.com/mavlink/mavros/tree/ros2/mavros_msgs/msg.

## Section 3: Implementing Topics in C++ Code

To integrate some topic into your code, you need to follow a few steps. 

### Determining Topic Type and Source Package

You need to decide, what is the topic type and what package is it coming from. 

From our example our message is of type `geometry_msgs/msg/PoseStamped`. This means that we need to add `geometry_msgs` package into our `CMakeLists.txt` and `package.xml`. We need to include it as a header too into our implementation file.

The line added in `CMakeLists.txt` should look like this: [Example from template](../template_drone_control/CMakeLists.txt#L16)

The line added in `package.xml` should look like this: [Example from template](../template_drone_control/package.xml#L12)

The include in the implementation file `template_drone_control_node.cpp` should look like this: [Example from template](../template_drone_control/src/template_drone_control_node.cpp#L2)

### Creating Subscribers and Publishers

Now you need to decide, if you want to publish the topic or subscribe it. Publish means, that the topic will be available for other `ros2 nodes` subscribing means, that we will obtain data from the topic (similar as we did in the console woth `ros2 topic echo`).


#### Subscriber
To create a subscriber to a topic you need to create the subsciber and then add callback that will handle the incoming data.

The object subsciber can be created like this: [Example from template](../template_drone_control/src/template_drone_control_node.cpp#L81)

You can initialize subsciber like this, please note, that the callback and topic name are used as parameters: [Example from template](../template_drone_control/src/template_drone_control_node.cpp#L23-L24)

Callback can be defined like this, please note that in implementation of the callback there is showed how work with incoming message (this information can be optain from command line using `ros2 interface show <name of the topic>` or documentation): [Example from template](../template_drone_control/src/template_drone_control_node.cpp#L56-L68)

#### Publisher

To create publisher for a specific topic you need to create publisher and then initialize (it is similar as the subsciber examples can be found in the `template_drone_contol`). 

If you want to publish some ros2 message, you can implement it like this, we will use `local_pos_pub_` available from the template:

```
// this is defined somewhere in the code  -> local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);

// now we will create message of type geometry_msgs::msg::PoseStamped
geometry_msgs::msg::PoseStamped setpoint_to_send;

setpoint_to_send.pose.position.x = 1;

// now we will publish the message

local_pos_pub_.publish(current_local_pos_);

// we can have implement this in the loop with some sleep, that will ensure frequency of the publishing of the topic
```

Please note, that in the template, there is not created the publisher that you need to control the UAV position.

## Conclusion
This tutorial can be used for different topics, that you need to work with. To use ros2 services, please refer to the template_drone_control, everything should be implemented here.