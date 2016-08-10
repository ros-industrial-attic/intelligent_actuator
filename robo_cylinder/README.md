# robo_cylinder
This is a ROS package for controlling an IAI Robo Cylinder through serial communication.

The package contains two nodes and one launch file that control multiple services for the Robo Cylinder. One node (robo_cylinder_services.py) starts services for powering the servos on/off, homing the system, an absolute move in either meters or pulses, changing the velocity and acceleration, and retrieving current position status. This node also publishes the current position at a rate of 10Hz to the topic /car/pos. The second node (car_tf_broadcaster.py) subscribes to the /car/pos topic and broadcasts a transfrom from the robot base to the current position of the carriage.

Either node (robo_cylinder_services.py and car_tf_broadcaster.can be started individually rosrun.

e.g. rosrun robo_cylinder robo_cylinder_services.py 

Or the nodes can be started together using roslaunch with the launch file robo_cylinder.launch. The parameters for port name, axis number, and screw lead can also be changed for robo_cylinder_services and the base_link name can be changed for car_tf_broadcaster. The parameters can be changed by manually editing the launch file or from the command line. If they are not changed, they will revert to the default values listed below.

e.g. with default values: roslaunch robo_cylinder robo_cylinder.launch
  
e.g. with changed parameters: roslaunch robo_cylinder robo_cylinder.launch port:='/dev/ttyUSB1' base_link:='your_base_link_name' axis:='1' lead:='8'

**Robo Cylinder setup paramters that may need to be changed:**

The default value for the serial port name is '/dev/ttyUSB0' but this may be different for your computer (e.g. typically 'COM #' for Windows machines or '/dev/ttyUSB#' for Linux).

The screw lead has a default value of 10 but depends on the model of Robo Cylinder you are using. The model number can be found on the side of the track of the Robo Cylinder and the corresponding data sheet can be found online. (e.g. http://www.intelligentactuator.com/pdf/robo/RCS-SM-100.pdf)

The axis number is set to a default of 0 but depends on the arrangement of the bit switches on the Robo Cylinder controller. Additionally, the Robo Cylinder needs to be powered on and plugged into the manufacturer's software in order to connect to the correct Axis. After it has been connected, you can exit the software and run this package to control it.

Additional information regarding specific Robo Cylinder serial protocol: http://www.intelligentactuator.com/pdf/manuals/RC_Serial_Communication.pdf

The python scripts also need to be made executables by using the code "chmod +x $SCRIPT_NAME" after building the package.
