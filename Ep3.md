## Episode 3

Welcome to the final episode of Frosty Winter!

In this week, we first focus on an example of autonomous navigation and then look at the various applications ROS can have, along with some pointers to resources to help you look beyond this workshop on your journey in robotics.

To start off, we look at an example of autonomous navigation in a grid. You might have seen some naive methods for this while doing the first task. As usual, we will be using Turtlebot3 as the bot for exploring this weeks content. Compared to earlier weeks, this week would be lighter, and we will only be tying up any loose ends from previous weeks.


### Navigation in a Closed Maze 

First, create a new package in the src directory of your catkin workspace named ep3, with dependencies on rospy, std_msgs and gazebo_ros. In this package, create folders for launch, script and worlds. Copy [this file](https://github.com/govindsaju/Frosty-Week3/blob/main/Week3_Files/maze.world) and put this into the worlds folder, and copy [this file](https://github.com/govindsaju/Frosty-Week3/blob/main/Week3_Files/maze_world.launch) and put it into the launch folder. This was just to load the map in gazebo and spawn the turtlebot.

To ensure everything is working, go to your catkin_workspace, and run catkin_make . Then run

```
roslaunch ep3 maze_world.launch
```
This should launch gazebo with a closed maze and turtlebot inside it.

Now, we shall write a script to automate the process of navigation, i.e. to keep travelling within this maze.

In scripts.py, create a file named 'navigator.py'. We are going to create a class named AutoNavigator to handle this navigation for us. Looking into the code snippet by snippet,

```
# Importing libraries

import rospy
from geometry_msgs.msg import Twist    #For /cmd_vel
from sensor_msgs.msg import LaserScan  #For /scan

```

Now, to define the skeleton of the class and the main function:

```
class AutoNavigator:
	def __init__(self):
		pass
	
	def scan_callback(self,msg):
		pass
	
	def run(self):
		pass
		
if __name__=='__main__':
	navigator = AutoNavigator()
	navigator.run()
```


First, we need to define the constructor. We need to add publishers to /cmd_vel topic, we need to subscribe to /scan topic. We also declare helper variables to act as a command to the /cmd_vel topic, and store the minimum distances in the front, left and right based on the callback from the /scan topic. We initialise all these variables in the constructor.

```
def __init__(self):
    self.cmd_vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
    self.scan_sub = rospy.Subscriber('scan',LaserScan,self.scan_callback)
    self.node = rospy.init_node('navigator')
    self.command = Twist()
    self.command.linear.x = 0
    self.command.angular.z = 0
    self.rate = rospy.Rate(10)
    self.near_wall = False
    self.min_front = 1
    self.min_right = 1
    self.min_left = 1
    self.min_range = 1
 ```


We then proceed to write the scan_callback function. The ranges variable in the message returned by laser scan contains a list of distances of objects with indices based on the relative angle of the point with respect to the front of the robot. For frontal distances, we take distances between -5 and 5 degrees, for the rightside we consider distances between 300 and 345 degrees, and for left we consider between 15 and 60 degrees. We then take the minimum of each of these distances and update them in the variables declared earlier.
```
def scan_callback(self,msg):
    allranges = msg.ranges
    frontal = allranges[0:5] + allranges[-1:-5:-1]
    rightside = allranges[300:345]
    leftside = allranges[15:60]
    self.min_left = min(leftside)
    self.min_right = min(rightside)
    self.min_front = min(frontal)
    self.min_range = min( self.min_left , self.min_front , self.min_right )

```

Now, we need to define the run function of this class. The algorithm we're using for navigation is fairly simple:
* First we travel forward until we reach near a wall.
* If we reach a wall, then we start following the left side wall. 
* If it gets too close to the wall, we reverse a bit.
* Else, continue following the wall by reversing a little bit.
* If the front is too close to a wall, then turn until its clear.

The code below uses few parameters for the linear and angular velocities. Do feel free to change them and experiment with the results. 

```
def run(self):
    while not rospy.is_shutdown():
    
    	#This following while loop runs only in the start while it is moving towards a wall after which near_wall becomes true, and is never reset.
        while self.near_wall==False and not rospy.is_shutdown():
            print("Moving to wall")
            
            # If not near any wall, continue travelling
            if self.min_range > 0.2:
                self.command.angular.z = -0.1
                self.command.linear.x = 0.15
                
            # If wall on left side, break out of loop and follow left side
            elif self.min_left < 0.2:
                self.near_wall = True
                
            # Rotate until wall is on left
            else:
                self.command.angular.z = -0.25
                self.command.linear.x = 0

            self.cmd_vel_pub.publish(self.command)
            self.rate.sleep()
            
            
        #Once we have found the wall for the first time, this is always executed.
        else:
        
        	#If there is space in front of robot, we move in zig zag format by staying near left wall.
            if (self.min_front > 0.2):
                if (self.min_left < 0.12):
                    print("Range: {:.2f}m - Too close. Backing up.".format(self.min_left))
                    self.command.angular.z = -1.2
                    self.command.linear.x = -0.1
                    
                #If too far from wall, go near wall
                elif self.min_left > 0.15:
                    print("Range: {:.2f}m - Wall-following; turn left.".format(self.min_left))
                    self.command.angular.z = 1.2
                    self.command.linear.x = 0.15
                    
                # If not too far, then go away from wall
                else:
                    print("Range: {:.2f}m - Wall-following; turn right.".format(self.min_left))
                    self.command.angular.z = -1.2
                    self.command.linear.x = 0.15
                    
            # If there's an obstacle in the front, rotate clockwise until that obstacle becomes the left wall.
            else:
                print("Front obstacle detected. Turning away.")
                self.command.angular.z = -1.0
                self.command.linear.x = 0.0
                self.cmd_vel_pub.publish(self.command)
            self.cmd_vel_pub.publish(self.command)
        self.rate.sleep()
```



To summarise, your navigator.py will look like (without the comments added here to help you understand):

```
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class AutoNavigator:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
        self.scan_sub = rospy.Subscriber('scan',LaserScan,self.scan_callback)
        self.node = rospy.init_node('navigator')
        self.command = Twist()
        self.command.linear.x = 0
        self.command.angular.z = 0
        self.rate = rospy.Rate(10)
        self.near_wall = False
        self.min_front = 1
        self.min_right = 1
        self.min_left = 1
        self.min_range = 1
        
    def scan_callback(self,msg):
        # This message has indices based on the angle relative to front of robot
        allranges = msg.ranges
        frontal = allranges[0:5] + allranges[-1:-5:-1]
        rightside = allranges[300:345]
        leftside = allranges[15:60]
        self.min_left = min(leftside)
        self.min_right = min(rightside)
        self.min_front = min(frontal)
        self.min_range = min(self.min_left,self.min_front,self.min_right)

    def run(self):
        while not rospy.is_shutdown():
            while self.near_wall==False and not rospy.is_shutdown():
                print("Moving to wall")
                if self.min_range > 0.2:
                    self.command.angular.z = -0.1
                    self.command.linear.x = 0.15
                elif self.min_left < 0.2:
                    self.near_wall = True
                else:
                    self.command.angular.z = -0.25
                    self.command.linear.x = 0

                self.cmd_vel_pub.publish(self.command)
                self.rate.sleep()
            else:
                if (self.min_front > 0.2):
                    if (self.min_left < 0.12):
                        print("Range: {:.2f}m - Too close. Backing up.".format(self.min_left))
                        self.command.angular.z = -1.2
                        self.command.linear.x = -0.1
                    elif self.min_left > 0.15:
                        print("Range: {:.2f}m - Wall-following; turn left.".format(self.min_left))
                        self.command.angular.z = 1.2
                        self.command.linear.x = 0.15
                    else:
                        print("Range: {:.2f}m - Wall-following; turn right.".format(self.min_left))
                        self.command.angular.z = -1.2
                        self.command.linear.x = 0.15
                else:
                    print("Front obstacle detected. Turning away.")
                    self.command.angular.z = -1.0
                    self.command.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.command)
                self.cmd_vel_pub.publish(self.command)
            self.rate.sleep()





if __name__=='__main__':
    navigator = AutoNavigator()
    navigator.run()
```

Now, to run this, execute

```
rosrun ep3 navigator.py
```

The bot should start moving inside the maze following the walls.

This summarises one example of autonomous navigation (in this case using wall following). 

### Looking Ahead

Through this workshop, you were introduced to some tools used in Robotics centered around ROS. You saw the basics of ROS, simulation and visualization tools like RViz and Gazebo, image processing software - OpenCV, ArUco markers and many more. You also saw a basic autonomous navigation of a grid using the turtlebot by travelling along the walls of the grid. 

Before we proceed to the task for this week, we first take a bird's eye view of the various applications of ROS and where you can continue exploring its applications in robotics.


 
