
# AttachablePlugin
=======
# AttachablePlugin (AttachableJoint)
This is a plugin for Ignitions that generates a joint dynamically during simulation with a topic where you send a string that contains parent model, parent link, child model and child link.
It can be used to grab things in gazebo and for modular robots.


1. Include the plugin in the .sdf world.

It does not need to be inside a model. 
~~~
<plugin filename=" <path_to_your_build_folder> /libAttachableJoint.so" name="attachable_joint::AttachableJoint">
</plugin>
~~~
 for example:
     <plugin filename="/home/ega/examples_ws/install/attachable_joint/lib/attachable_joint/libattachable_joint_plugin.so" name="attachable_joint::AttachableJoint"/>


2. Create and Destroy the Link Dinamically

To create a link you have to send a ignition::msgs::StringMsg with this architecture:
[ParentModel][ParentLink][ChildModel][ChildLink][attach]

If you want to detach use [detach]

like this:
~~~
ign topic -t /AttachableJoint -m ignition.msgs.StringMsg -p 'data:"[parentModel][ParentLink][ChildModel][ChildLink][attach]"'
~~~

You can send it from ROS2, see https://github.com/ignitionrobotics/ros_ign/tree/melodic/ros_ign_bridge


Use a link that has a body, not an empty link, otherwise it somehow doesn't work.

In world/diff_drive.sdf: use 
```
ros2 topic pub /box2/attach std_msgs/msg/String 'data: "[diff_drive][lidar_link][box2][box_body][attach]"' --once  
```




# Installation

0. Clone the version of [this](https://github.com/gazebosim/ros_gz_project_template/)  repository corresponding to your gazebo version

For example:
```bash
git clone -b fortress https://github.com/gazebosim/ros_gz_project_template.git
```

1. Clone this repository e.g.

```bash
git clone https://github.com/kas-lab/Ign-gazebo-AttachablePlugin.git
```

2. Build your workspace using colcon
