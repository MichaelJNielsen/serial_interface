#!/usr/bin/env python
import rospy, serial, time, signal
from std_msgs.msg import String
from serial_interface.msg import Razorimu

ser1 = serial.Serial('/dev/ttyS2',115200)

def keyboardInterruptHandler(signal,frame):
    exit(0)
signal.signal(signal.SIGINT,keyboardInterruptHandler)

cycle = 0
i1 = 0
j1 = 0

def read_from_serial1():
    global i1, j1, serial_data1, cycle
    bytesToRead = ser1.inWaiting()
    while bytesToRead > 209:
        j1 = j1+1
        clearer = ser1.readline()
        bytesToRead = ser1.inWaiting()
        
    if bytesToRead < 65:
        i1 = i1+1
    else:
        bytes = ser1.readline()
        string = bytes.decode()
        splitline = string.split(',')
        temp_data = []
        for x in splitline:
            temp_data.append(float(x))
        serial_data1 = temp_data

if __name__ == '__main__':
    pub1 = rospy.Publisher('/Razor_IMU/SafeEye', Razorimu, queue_size=10)
    rospy.init_node('Razor_IMUs', anonymous=True)
    rate = rospy.Rate(100)
    msg1 = Razorimu()
    
    ser1.read(ser1.inWaiting()-210)
    
    serial_data1 = [0,0,0,0,0,0,0,0,0,0]

    while True:
        cycle = cycle+1
        read_from_serial1()

        if (cycle > 100) and (i1 > 0.5*cycle):
            exit(0)
        
        msg1.time_stamp = serial_data1[0]
        msg1.acc_x = serial_data1[1]
        msg1.acc_y = serial_data1[2]
        msg1.acc_z = serial_data1[3]
        msg1.gyro_x = serial_data1[4]
        msg1.gyro_y = serial_data1[5]
        msg1.gyro_z = serial_data1[6]
        msg1.mag_x = serial_data1[7]
        msg1.mag_y = serial_data1[8]
        msg1.mag_z = serial_data1[9]
        
        pub1.publish(msg1)
        rate.sleep()
        
        

        
        
        
