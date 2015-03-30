
########################################################################### 
# This software is graciously provided by HumaRobotics 
# under the Simplified BSD License on
# github: git@www.humarobotics.com:baxter_tasker
# HumaRobotics is a trademark of Generation Robots.
# www.humarobotics.com 

# Copyright (c) 2013, Generation Robots.
# All rights reserved.
# www.generationrobots.com
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

from modbus.modbus_wrapper_client import ModbusWrapperClient
class InsightModbusClient(ModbusWrapperClient):
    """
        Wrapper that integrates python modbus into standardized ros msgs.
        The wrapper is able to read from and write to a modbus server running on a Cognex Insight 7200.
        The corresponding Insight Explorer project should also be contained in this ros package
    """
    def __init__(self,host,sim=True,rate=50):
        """
            :param host: Contains the IP adress of the modbus server
            :type host: string
            :param rate: How often the registers on the modbusserver should be read per second
            :type rate: float
            :param reset_registers: Defines if the holding registers should be reset to 0 after they have been read. Only possible if they are writeable
            :type reset_registers: bool
        """
        print("Use the appropriate Cognex Insight Project to enable the Modbus Server")
        self.sim = sim
        ModbusWrapperClient.__init__(self,host,502,rate,reset_registers=False)
        # All registers
        self.write_start = 50000
        self.setReadingRegisters(30010,30127-30010)
        # Use only a few registers for writing
        self.setWritingRegisters(self.write_start,2)

        # Note: Listener not started as reading is requested manually, but a heartbeat is required to keep the modbus server alive
        self.startHeartbeat()
        

    def startVisualInspection(self,job_id,timeout=5.0):
        """
            Sends a modbus request to change the part detection. job_id 0 is reserved for the heartbeat
            :param job_id: part id in the insight explorer that should be detected
            :type job_id: int
            :param timeout: Duration how long this method should wait for a positive response from the part inspection. The default value should be sufficient.
            :type timeout: float
        """
        rospy.loginfo("Setting new cognex job id: %d", job_id) 
        self.setOutput(self.write_start,[job_id])            
        self.changeTrigger()
        rospy.sleep(0.5)            
        start_time = rospy.get_time()
        return self.__getResponse(job_id,start_time,timeout)
    
    def __getResponse(self,id,start_time,timeout):
        """
            Reads the registers from the camera in the correct order:
            is alive
            job_id
            job_pass
            job_fail
            barcode
            
            :param job_id: The job from which we expect an answer
            :type job_id: int
            :param start_time: Contains the time when the request was sent
            :type start_time: float
            :param timeout: Duration until the latest response of the inspection
            :type timeout: float
        """
        if start_time + timeout < rospy.get_time():
            rospy.loginfo("Timeout in getting response from Insight")
            return False
        tmp = self.readRegisters()
        
        if tmp is None:
            rospy.logwarn("Nothing received from modbus server")
            return False
        is_alive = tmp[0]
        job_id = tmp[1]
        job_pass = tmp[2]
        job_fail = tmp[3]
        string_length = 25
        my_string = self.getString(30014,string_length)
        barcode = self.getString(30014+string_length,string_length)[:-1]
        response = [is_alive, job_id,job_pass,job_fail,my_string,barcode]
#         print "Response for debugging:",response
        return self.__evaluateResponse(id,response,start_time,timeout)

    def __evaluateResponse(self,job_id,response,start_time,timeout):
        """
            Evaluates the response
            
            :param job_id: The job from which we expect an answer
            :type job_id: int
            :param response: Contains the response from the modbus registers [is_alive,job_id,job_pass,job_fail,barcode]
            :type response: list
            :param start_time: Contains the time when the request was sent
            :type start_time: float
            :param timeout: Duration until the latest response of the inspection
            :type timeout: float
            :return: Depending on the job if it has passed or failed
            :rtype: bool
        """
        if not response[0]:
            return False
        elif job_id == 0: # job_id 0 is the is_alive req    
            return True
        if job_id == response[1]:            
            if job_id == 3:
                my_string = response[4]
                if len(my_string) > 0:
                    rospy.loginfo(response[4])
                    return True
            elif job_id == 4: # job id 1 is used for the barcode in the example
                my_string = response[5]
                if len(my_string) > 0:
                    rospy.loginfo(response[5])
                    return True
            elif response[2]:
                return True
            # False is returned if the timeout is triggered and not by the fail, that is passed by the camera
        rospy.sleep(0.1)
        return self.__getResponse(job_id,start_time,timeout)

    def __decodeString(self,ascii):
        """    
            Converts an ascii number to a string
            :param ascii: Integer code of a string encoded in ascii
            :type ascii: int
            :return: Returns two characters as string
            :rtype: string
        """
        second = ascii%256
        first = (ascii-second)/256
        return str(chr(first))+str(chr(second)) 
                
    def getString(self,start,num_registers):
        """
            Reads num_registers from start and tries to recover the string that has been send by the modbus server
            :param start: Starting address of the modbus register to read the string
            :type: int
            :param:number of registers to read
            :type: int
            :return: Returns a string
            :rtype: string
        """
        msg = ""
        tmp = self.readRegisters(start,num_registers)
        for ascii in tmp:
            if ascii == 0:
                continue
            msg += self.__decodeString(ascii)
        return msg
    
    def changeTrigger(self):
        """
            Changes the trigger on the camera to manual and back to continous. This gives the camera some time to update its parameters
        """
        def reg(letter,number):
            """
                Returns an integer for a cell value
                
                Usable triggers on the insight explorer software
                * 0: Camera
                * 1: Continous
                * 2: External
                * 3: Manual
                * 4: Network
                
            """
            v=ord(letter.lower())-ord('a')
            v=v<<10
            v+=number
            return v
        v = reg("A",3)
        rospy.sleep(0.1)
        self.setOutput(v, [3])
        rospy.sleep(0.6)
        if self.sim:
            self.setOutput(v, [4])
        else:
            self.setOutput(v, [1])
        
        
    
    def startHeartbeat(self):
        """
            Non blocking call to start the heartbeat loop
        """
        self.post.__sendHeartbeat()
            
    
    def __sendHeartbeat(self):
        """
            Sends an is alive signal to the cognex camera. If this is not send, the camera times out 1 hour after the last message in the best case scenario
        """
        
        while not rospy.is_shutdown():
            rospy.sleep(5)
            self.setOutput(self.write_start+1,0)
    


    