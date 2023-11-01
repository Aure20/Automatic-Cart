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

