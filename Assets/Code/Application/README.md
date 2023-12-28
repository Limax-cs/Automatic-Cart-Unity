# Automatic Cart
 
 Commands:

Open roobot in RViz:
`roslaunch automatic_cart display.launch model:='$(find automatic_cart)/models/shopping_cart/automatic_cart.urdf.xacro'`

Open robot in gazebo:
`roslaunch automatic_cart gazebo.launch`

Open robot with telemetry:
`roslaunch automatic_cart control_gazebo.launch`

See odometry:
`rostopic list`
`rostopic echo /cart_diff_drive_controller/odom`

Following the structure of Turtlebot3:

Open a world with the robot:
`roslaunch automatic_cart cart_house.launch`
`roslaunch automatic_cart cart_world.launch`
`roslaunch automatic_cart cart_shop1.launch`

Open SLAM:
`roslaunch automatic_cart cart_slam.launch slam_methods:=gmapping`

Open telemetry aside:
`roslaunch automatic_cart cart_telemetry.launch`

