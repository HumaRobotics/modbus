modbus
=========

The modbus stack provides a wrapper from the modbus communication to standardized ROS messages. The modbus package is based on pymodbus and is also written in Python.

After a catkin_make the Python modbus classes are also available from the outside and can be easily integrated in other packages.

This stack was used in a quality inspection project with the Baxter robot interfacing with a Siemens PLC and a Cognex In-Sight camera. 

* The package modbus is the basic python wrapper for a modbus server and client for ROS
* The package modbus_cognex_insight inherits the modbus client base class and which uses specific registers. The client can send job_ids to the camera and retrieve the results of the jobs as well as barcodes and strings.
* The package modbus_plc_siemens inherits the modbus client base class and changes the register size.