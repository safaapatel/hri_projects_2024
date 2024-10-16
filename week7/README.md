# Week7/8 Tutorial

Major deliverables this week:

- Use a message from a prior package
- Get familiar with the stage simulator
- Issue some robot commands for stage
- Practice a ROS connection from one computer to another
- Obstacle avoidance

## How this package was created

You don't need to run this command, but just so that you know how this package was created, here it is:

```
catkin_create_pkg week7 week1 rospy geometry_msgs std_msgs stage_ros
```

## Get familiar with the stage simulator

First off, make sure that stage is installed:

```
sudo apt install ros-noetic-stage-ros
```

I've added a world file. you can run this using a launch file:

```
roslaunch week7 basic_world.launch
```

If you don't already have a roscore node started, then launching the launch file will start one of those (which will close when you close the launch file), it also sets a parameter that you'll need when using simulators, telling ROS to get the time from the simulator.

The simulator window should pop up. The simulator is a ROS node that has some publishers and subscribers. Here are some ways of figuring out what's going on with a node:

```
rostopic list
rosnode info stage
```

## Issue some robot commands for stage


### Sensors

Let's start by looking at some of the robot's sensor output. You can see what these topics are publishing using rostopic:

```
rostopic type /base_pose_ground_truth
```

We can read from the robot's sensors and its location using the command rostopic:

```
rostopic echo /base_pose_ground_truth
rostopic echo /base_scan
```

Remember that we can see how a message type is constructed by using the `rosmsg` command:

```
rosmsg show sensor_msgs/LaserScan
```


- Goal 1: Create a new node (you can copy the listener node from week0/1) that listens to the base_scan topic. Listen on the /base_scan topic and print the closest reading (you'll have to loop through all the sensor readings).

### Actuators

Next up, let's look at how we can make the robot move. We can command the robot to move using the /cmd_vel topic. You can see how that message gets contructed:

```
rosmsg show geometry_msgs/Twist
```

If we change linear.x, then we can make the robot move forward and backward. If you change angular.z, you can make the robot turn.

- Goal 2: Write some nodes to make the robot move:
	- In a square
	- In a figure-eight
	- In a triangle


## Obstacle avoidance

Make one node that both reads from the laser message as well as publishes the cmd_vel topic as well. Now we want to make a robot that can move using the sensor data.

- Goal 3: Make a robot that can move forward, unless there's an obstacle close by, then it would stop.

- Goal 4: Make a robot that can move forward, then turn if there is an obstacle in front of the robot. It should turn in the direction with fewer obstacles.
