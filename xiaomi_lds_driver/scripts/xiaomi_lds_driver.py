#! /usr/bin/env python3
import serial
import matplotlib.pyplot as plt
import statistics
import math
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

class LidarData():
    
    def __init__(self):        
        self.DATA_LENGTH = 7        # data length : angle, speed, distance 1 - 4, checksum
        self.MAX_DISTANCE = 3000    # in mm
        self.MIN_DISTANCE = 100     # in mm
        self.port = '/dev/ttyUSB0'          # SPECIFY serial port 
        self.MAX_DATA_SIZE = 360    # resolution : 1 degree
        self.ser = None
        self.BAUDRATE = 115200

        self.data = {   # sensor data 
            'angles'    : [],
            'distances' : [],
            'speed'     : [],
            'signal_strength' : [], # TODO:
            'checksum'  : [],
            'scan_ranges'  : [np.nan]*360 #size 360 array for ROS LaserScan msgs

        }
	
        self.angles_rad = [i*math.pi/180 for i in range(1,361)]        
        
        
        # setup plot
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.ax.set_rmax(300)
        
        self.connectSerial(self.port, self.BAUDRATE)   # setup serial communication
        
        rospy.init_node('xiaomi_lds_driver')
        self.prev_time = rospy.Time.now()
        self.scan_pub = rospy.Publisher("/scan",LaserScan,queue_size=10)
        
    def connectSerial(self, port: str, baudrate: int) -> bool:
        try: 
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.ser.reset_input_buffer()
            print(f'Serial connection established @ {port}')
            return True # return True to confirm connection
        except Exception as e: 
            print(e)
            return False
        
    def plotData(self) -> None: # plot data on a polar plot
        angles, distances = [], []
        self.data['scan_ranges']  = [np.nan]*360 #size 360 array for ROS LaserScan msgs
        
        for p in range(3, len(self.data['angles'])-3):
            
            # ------ not sure what I'm doing here... ----------------
            # TODO: implement some sort of filter to remove outliers
            if (p > len(self.data['angles']) - 3) : break
            sample = self.data['distances'][p-3:p+3]
            std = statistics.stdev(sample)
            if abs(self.data['distances'][p]-statistics.mean(sample)) < std:
                angle = self.data['angles'][p]
                angles.append(angle)
                distances.append(self.data['distances'][p])
                idx = round(angle* 180/math.pi)-1
                if idx < 360:
                    self.data['scan_ranges'][idx] = self.data['distances'][p]/1000
            # ------------ filter END -------------------------------
        
        #self.ax.clear() # clear current plot
        #plt.plot(angles, distances, ".")    # plot the points
        #plt.plot(self.angles_rad, self.data['scan_ranges'], ".")    # plot the points
        #self.ax.set_rmax(self.MAX_DISTANCE)
        self.data['angles'].clear()
        self.data['distances'].clear()
        #plt.draw()
        #plt.pause(0.001)
        
        msg = LaserScan()
        msg.header.stamp = rospy.Time(0)
        msg.header.frame_id = 'lidar_frame'
        msg.angle_min= -math.pi
        msg.angle_max= math.pi
        msg.angle_increment= 1/180 * math.pi
        #msg.time_increment= (60/int(self.data['speed'][-1]))/360
        #msg.scan_time= 60/int(self.data['speed'][-1])
        msg.time_increment= (rospy.Time.now().to_sec()-self.prev_time)/360
        msg.scan_time= (rospy.Time.now().to_sec()-self.prev_time)
        msg.range_min= 0.1
        msg.range_max= 3.0
        msg.ranges = self.data['scan_ranges']
        self.scan_pub.publish(msg)
                                
                
    def updateData(self) -> None: 
        prev_angle=0
        while not rospy.is_shutdown():
            try: 
                if self.ser.in_waiting > 0:
                    
                    try: # try read serial inputs; if unsuccessful, ignore the current input data
                        line = self.ser.readline().decode().rstrip()
                        sensorData = line.split('\t')  
                        
                    except: # ignore if the data is invalid 
                        continue
                    
                    # convert string to float, then publish topic
                    if len(sensorData) == self.DATA_LENGTH:
                        
                        for i in range(2,6):    # split into four data points
                            try:           
                                # note: angles comes in increment of 4 degrees as each packet contains 4 readings                    
                                angle = (int(sensorData[0]) + i - 1) * math.pi / 180  # angle in radians
                                #angle = (int(sensorData[0]) + i - 1)			 # angle in degrees
                                dist = float(sensorData[i])   # distance in mm
                                print(f'speed : {int(sensorData[1])} RPM, angle : {round(angle* 180 / math.pi)}, dist : {round(dist)}')
                                
                            except: continue
                            
                            # if the data is valid, update sensor data
                            if dist >= self.MIN_DISTANCE and dist <= self.MAX_DISTANCE:
                                
                                # store angular data in radians
                                self.data['angles'].append(angle)
                                
                                # store radial data in mm
                                self.data['distances'].append(dist)
                                
                                # store checksum data for each measurement
                                self.data['checksum'].append(sensorData[-1])

                                # store speed data in RPM
                                self.data['speed'].append(sensorData[1])
                                
                                
                            #if len(self.data['angles']) == self.MAX_DATA_SIZE:  # if enough data is available, plot data
                            if angle < prev_angle:# and len(self.getDistances()) > 360: #new scan
                          #  print(self.data['scan_ranges'])
                                self.plotData()
                            self.prev_time = rospy.Time.now().to_sec()
                            prev_angle = angle
                            
                        #if len(self.data['angles']) == self.MAX_DATA_SIZE:  # if enough data is available, plot data
                            #self.plotData()
                                #print("scan")
                                
            except KeyboardInterrupt:
                exit()

    
    def getDistances(self) -> list: return self.data['distances']
    
    def getAngles(self) -> list: return self.data['angles']
    
    
if __name__ == '__main__':
    sensor = LidarData()
    sensor.updateData()

    	
