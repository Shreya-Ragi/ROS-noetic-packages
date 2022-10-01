# ROS-noetic-packages
This repository is a detailed guide on creating a ROS Package using catkin

This README.md contains:
- Creating a package inside catkine workspace
- Compiling the catkin workspace
- Running your own node in the created package

## Creating a package inside catkin workspace
This tutorial will demonstrate how to use catkin_create_pkg to create a new package.

Upon opening the terminal, change to the source directory of the catkin workspace:
```bash
cd DNT_ws/
cd src/
```

catkin_create_pkg requires you to give a a package name and optionally a few dependencies while creating the package:
```
# catkin_create_pkg package_name depend1 depend2 depend3
```
Now use catkin_create_pkg to create a new package called "stage3_yourname" which depends rospy and std_msgs:
```bash
catkin_create_pkg stage3_yourname rospy std_msgs
```
Replace "stage3_yourname" with your package name according to instructions
This will create a folder named stage3_yourname which contains an src folder, a CMakeLists.txt file and a package.xml file.

There are several other dependenices, like roscpp, that can be used based on your requirements while creating a package using catkin_create_pkg script.

## Compiling the catkin workspace
Now you need to build and compile the catkin workspace:
```bash
cd ..
catkin_make
```

## Running your own node in the created workspace
Create any node that you would like to run inside the src folder of your stage3_yourname folder using the code editor.
Here, a listener.py file and a talker.py file have been created to run in the new workspace using the code editor:

### listener.py:
```py
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def chatter_callback(message):
    #get_caller_id(): Get fully resolved name of local node
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", message.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, chatter_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```
### talker.py:
```py
#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    #create a new publisher. we specify the topic name, then type of message then the queue size
    pub = rospy.Publisher('chatter', String, queue_size=10)
    #we need to initialize the node
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node 
    rospy.init_node('talker', anonymous=True)
    #set the loop rate
    rate = rospy.Rate(1) # 1hz
    #keep publishing until a Ctrl-C is pressed
    i = 0
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % i
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
        i=i+1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```
NOTE: Change the message to be published according to the task.
Remember to save both files.

Now change the permission of the new nodes to "Executable" using the command chmod:
```bash
chmod u+x ~/[file_location]/[file_name]
```
Where, u+x will grant the user permission to execute the file.
```bash
cd ..
chmod u+x ~/DNT_ws/src/stage3_yourname/src/listener.py
chmod u+x ~/DNT_ws/src/stage3_yourname/src/talker.py
```
Replace "stage3_yourname" with your package name.

Then, edit the CMakeLists.txt file and add the following:
```
catkin_install_python(PROGRAMS src/talker.py src/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```
Save the changes.

Now, to finally run your ROS node:

### First run the master node:
```bash
roscore
```
### In a new terminal, run your listener node:
```bash
rosrun stage3_yourname listener.py
```
Replace "stage3_yourname" with your package name.

### In another terminal, run your talker node:
```bash
rosrun stage3_yourname talker.py
```
Replace "stage3_yourname" with your package name.

If you followed the instructions correctly, you will have made a ROS package and run your own nodes in the created catkin workspace!

For more ROS repositaries check out:
- https://github.com/dreadnoughtrobotics
- https://github.com/Aranyaa-k
- https://github.com/sacchinbhg
- https://github.com/Shreya-Ragi
