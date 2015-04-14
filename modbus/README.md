
modbus
======

This package is a wrapper for ROS integrating the Modbus TCP Protocol.
You can either connect to already existing modbus servers or create your own.

To communicate with the other modbus module, standardized ROS messages are used.

The scripts have been developed and tested on Ubuntu 14.04 with ROS Indigo and Python 2.7.6

## Dependencies

This wrapper package is based on the pymodbus library developed by: Galen Collins
[github: https://github.com/bashwork/pymodbus](github: https://github.com/bashwork/pymodbus)

## Prerequisities

In order to have all dependencies installed, you need to install Collin's pymodbus, as well as the
twisted server implementation, his work is based on.
```
	$ sudo apt-get install python-pymodbus
	$ sudo apt-get install python-pyasn1 python-twisted-conch
```

## Quickstart

Start a modbus server or use an existing one:
```
	$ rosrun modbus_wrapper modbus_server.py _port:=1234
```
Start the corresponding modbus client on the same or another computer. 
If started on another computer, replace localhost with the IP of the modbus server
```
	$ rosrun modbus_wrapper modbus_client.py _ip:="localhost" _port:=1234
```

## Contributors

This package was developed by Sven Bock and Wagdi Ben Yaala at [Generation Robots](http://www.generationrobots.com/en/)/[Humarobotics](http://www.humarobotics.com) in Bordeaux, France.
