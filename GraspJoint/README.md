# grasp plugin

'Simple' plugin for 'grasping' objects. The plugin moves the child object to the offset specified in the sdf and then next update loop adds an detachablejoint.

Copied code from upstream fork (also in AttachableJoint folder) and used https://github.com/gazebosim/ros_gz_project_template/ to create package of it.

# Usage:
```xml
 <plugin
    filename="GraspPlugin" name="grasp_plugin::GraspJoint"> 
    <attachtopic>/grasp/attach</attachtopic>
    <offset_x>1.0</offset_x>
    <offset_y>0.0</offset_y>
    <offset_z>1.5</offset_z>
</plugin>
```

Bridge config:
```yaml 
- ros_topic_name: "/grasp/attach"
  gz_topic_name: "/grasp/attach"
  ros_type_name: "std_msgs/msg/String"
  gz_type_name: "gz.msgs.StringMsg"
  direction: ROS_TO_GZ
```

See worlds/diff_drive.sdf for example world.


Attaching and detaching:
```bash
# ign topic .....
ros2 topic pub /grasp/attach std_msgs/msg/String 'data: "[diff_drive][lidar_link][box1][box_body][attach]"' --once
ros2 topic pub /grasp/attach std_msgs/msg/String 'data: "[diff_drive][lidar_link][box1][box_body][detach]"' --once
```


# Notes:
- make sure that the offset is far enough, self-collision will crash Gazebo
- Use a link of the child with the model/collision, otherwise it won't work (don't use the AttachableLinks in the sdf)