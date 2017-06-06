
########################################################################### 
# This software is graciously provided by HumaRobotics 
# under the BSD License on
# github: https://github.com/Humarobotics/modbus_wrapper
# HumaRobotics is a trademark of Generation Robots.
# www.humarobotics.com 
#
# Copyright (c) 2013, Generation Robots.
# All rights reserved.
# www.generationrobots.com
#
# This wrapper package is based on the pymodbus library developed by:
# Galen Collins
# github: https://github.com/bashwork/pymodbus
#   
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation 
#  and/or other materials provided with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS 
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
# THE POSSIBILITY OF SUCH DAMAGE. 
# 
# The views and conclusions contained in the software and documentation are 
# those of the authors and should not be interpreted as representing official 
# policies, either expressed or implied, of the FreeBSD Project.
import rospy
try:
    from pymodbus.client.sync import ModbusTcpClient
except Exception,e:
    print "pymodbus does not seem to be installed.\nInstall it by:\nsudo apt-get install python-pymodbus"
    print e
    exit()
from std_msgs.msg import Int32MultiArray as HoldingRegister
from post_threading import Post
from threading import Lock

NUM_REGISTERS = 20
ADDRESS_READ_START = 40000
ADDRESS_WRITE_START = 40020

class ModbusWrapperClient():
    """
        Wrapper that integrates python modbus into standardized ros msgs.
        The wrapper is able to read from and write to a standard modbus tcp/ip server.
    """
    def __init__(self,host,port=502,rate=50,reset_registers=True,sub_topic="modbus_wrapper/output",pub_topic="modbus_wrapper/input"):
        """
            Use subscribers and publisher to communicate with the modbus server. Check the scripts for example code.
            :param host: Contains the IP adress of the modbus server
            :type host: string
            :param port: The port number, where the modbus server is runnning
            :type port: integer
            :param rate: How often the registers on the modbusserver should be read per second
            :type rate: float
            :param reset_registers: Defines if the holding registers should be reset to 0 after they have been read. Only possible if they are writeable
            :type reset_registers: bool
        """
        try:
            self.client = ModbusTcpClient(host,port)
        except Exception, e:
            rospy.logwarn("Could not get a modbus connection to the host modbus. %s", str(e))
            raise e
            return
        self.__rate = rate
        self.__reading_delay = 1/rate
        self.post = Post(self)
        
        self.__reset_registers = reset_registers
        self.__reading_register_start = 0
        self.__num_reading_registers = 20
#         self.input_size = 16
        self.__input = HoldingRegister()
        self.__input.data = [0 for i in xrange(self.__num_reading_registers )]

        self.__writing_registers_start = ADDRESS_WRITE_START
        self.__num_writing_registers =  20
#         self.output_size = 16 
        self.__output = [None for i in range(self.__num_writing_registers)]
        
        self.__last_output_time = rospy.get_time()
        self.__mutex = Lock()
        
        self.__sub = rospy.Subscriber(sub_topic,HoldingRegister,self.__updateModbusOutput,queue_size=500)
        self.__pub = rospy.Publisher(pub_topic,HoldingRegister,queue_size=500, latch=True)
        
         
        
        rospy.on_shutdown(self.closeConnection)
    
    def startListening(self):
        """
            Non blocking call for starting the listener for the readable modbus server registers 
        """
        #start reading the modbus
        self.post.__updateModbusInput()
        
    def stopListening(self):
        """
            Stops the listener loop
        """
        self.stop_listener = True
        while not rospy.is_shutdown() and self.listener_stopped is False:
            rospy.sleep(0.01)
        
    def setReadingRegisters(self,start,num_registers):
        """
            Sets the start address of the registers which should be read and their number
            :param start: First register that is readable
            :type start: int
            :param num_registers: Amount of readable registers
            :type num_registers: int
        """
        self.__reading_register_start = start
        self.__num_reading_registers = num_registers

    def setWritingRegisters(self,start,num_registers):
        """
            Sets the start address of the registers which are writeable and their number
            :param start: First register that is writeable
            :type start: int
            :param num_registers: Amount of writeable registers
            :type num_registers: int
        """
        self.__writing_registers_start = start
        self.__num_writing_registers = num_registers
    
    def getReadingRegisters(self):
        """
            :return: Returns the first address of the readable registers and the number of registers
            :rtype: int,int
        """
        return self.__reading_register_start,self.__num_reading_registers
        
    def getWritingRegisters(self):
        """
            :return: Returns the first address of the writeable registers and the number of registers
            :rtype: int,int
        """
        return self.__writing_registers_start,self.__num_writing_registers
        
    def __updateModbusInput(self,delay=0):
        """                
            Loop that is listening to the readable modbus registers and publishes it on a topic
            :param delay: The delay time until the loop starts
            :type delay: float 
        """
        rospy.sleep(delay)
        self.listener_stopped = False
        self.stop_listener = False
        update = True
        while not rospy.is_shutdown() and self.stop_listener is False:
            try: 
                if not rospy.is_shutdown() :
                    tmp =  self.readRegisters()
                    if tmp is None:
                        rospy.sleep(2)
                        continue
                    # rospy.logwarn("__updateModbusInput tmp is %s ", str(tmp))
                    # rospy.logwarn("__updateModbusInput self.__input.data is %s ", str(self.__input.data))

                    if tmp != self.__input.data:
                        update = True
                        self.__input.data = tmp
                    else:
                        update = False 
            except Exception,e:
                rospy.logwarn("Could not read holding register. %s", str(e))
                raise e
                rospy.sleep(2)
        
            if update:
                if self.__pub.get_num_connections() > 0:
                    try:
                        self.__pub.publish(self.__input)
                    except Exception,e:
                        rospy.logwarn("Could not publish message. Exception: %s",str(e))
                        raise e
            rospy.Rate(self.__rate).sleep()
        self.listener_stopped = True
    
    def __updateModbusOutput(self,msg):
        """
            Callback from the subscriber to update the writeable modbus registers
            :param msg: value of the new registers
            :type msg: std_msgs.Int32MultiArray
        """
        output_changed = False
        for index in xrange(self.__num_writing_registers):
            if self.__output[index] != msg.data[index]:
                output_changed = True
                break
        if not output_changed:
            return
        self.__writeRegisters(self.__writing_registers_start,msg.data)
        
    def __writeRegisters(self,address,values):
        """
            Writes modbus registers
            :param address: First address of the values to write
            :type address: int
            :param values: Values to write
            :type values: list
        """
        with self.__mutex:
            try:
                if not rospy.is_shutdown() :
    #                 print "writing address",address,"value"
                    self.client.write_registers(address, values)
                    self.output = values
            except Exception, e:
                rospy.logwarn("Could not write values %s to address %d. Exception %s",str(values),address, str(e))
                raise e

    def readRegisters(self,address=None,num_registers=None):
        """
            Reads modbus registers
            :param address: First address of the registers to read
            :type address: int
            :param num_registers: Amount of registers to read
            :type num_registers: int
        """
        if address is None:
            address = self.__reading_register_start
        if num_registers is None:
            num_registers = self.__num_reading_registers

        tmp = None
        with self.__mutex:            
            try:
                tmp = self.client.read_holding_registers(address,num_registers).registers
            except Exception, e:
                rospy.logwarn("Could not read on address %d. Exception: %s",address,str(e))
                raise e
            
            
            if self.__reset_registers:
                try:
                    self.client.write_registers(address, [0 for i in xrange(num_registers)])
                except Exception, e:
                    rospy.logwarn("Could not write to address %d. Exception: %s", address,str(e))
                    raise e
            
        return tmp
    
    def setOutput(self,address,value,timeout=0):
        """
            Directly write one register
            :param address: The exact register address to write
            :type address: int
            :param value: What is written in the register
            :type value: int
            :param timeout: If set, the register is set after this time to 0
            :type timeout: float
        """
        if not type(value) is list:
            value = [int(value)]
        self.__writeRegisters(address, value)
        if timeout > 0:
            self.post.__reset(address,timeout)
            
    def __reset(self,address,timeout):
        """
            Resets a register to 0 after a specific amount of time
            :param address: The register address to reset
            :type address: int
            :param timeout: The delay after the register is reset
            :type timeout: float
        """
        rospy.sleep(timeout)
        self.__writeRegisters(address,[0])
    
    def closeConnection(self):
        """
            Closes the connection to the modbus
        """
        self.client.close()


    