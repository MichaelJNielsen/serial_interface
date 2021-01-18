#!/usr/bin/env python
import rospy, serial, time, signal
from std_msgs.msg import String
from serial_interface.msg import Razorimu

ser1 = serial.Serial('/dev/ttyS2',115200)
ser2 = serial.Serial('/dev/ttyACM0',115200)

def keyboardInterruptHandler(signal,frame):
    print("\ninterrupted")
    print("Times cleared - SafeEye:", j1, ",Accel1:", j2)
    print("Times failed - SafeEye:", i1, ",Accel1:", i2)
    exit(0)
signal.signal(signal.SIGINT,keyboardInterruptHandler)

cycle = 0
i1 = 0
j1 = 0
i2 = 0
j2 = 0

def read_from_serial1():
    global i1, j1, serial_data1, cycle
    bytesToRead = ser1.inWaiting()
    while bytesToRead > 209:
        j1 = j1+1
        print(cycle, "- clearing SafeEye: ", j1)
        print(cycle, "-before clearing: ", bytesToRead)
        clearer = ser1.readline()
        bytesToRead = ser1.inWaiting()
        print(cycle, "-after clearing: ", bytesToRead)
        
    if bytesToRead < 65:
        i1 = i1+1
        print(cycle, "-SafeEye - Not enough serial input, using last available",i1)
        print(cycle, "-Bytes available = ", bytesToRead)
    else:
        bytes = ser1.readline()
        string = bytes.decode()
        splitline = string.split(',')
        temp_data = []
        for x in splitline:
            temp_data.append(float(x))
        serial_data1 = temp_data

def read_from_serial2():
    global i2, j2, serial_data2, cycle
    bytesToRead = ser2.inWaiting()
    while bytesToRead > 209:
        j2 = j2+1
        print(cycle, "-clearing Accel1: ", j2)
        print(cycle, "-before clearing: ", bytesToRead)
        clearer = ser2.readline()
        bytesToRead = ser2.inWaiting()
        print(cycle, "-after clearing: ", bytesToRead)
        
    if bytesToRead < 65:
        i2 = i2+1
        print(cycle, "-Accel1 - Not enough serial input, using last available",i2)
        print(cycle, "-Bytes available = ", bytesToRead)
    else:
        bytes = ser2.readline()
        string = bytes.decode()
        splitline = string.split(',')
        temp_data = []
        for x in splitline:
            temp_data.append(float(x))
        serial_data2 = temp_data

if __name__ == '__main__':
    pub1 = rospy.Publisher('/Razor_IMU/SafeEye', Razorimu, queue_size=10)
    pub2 = rospy.Publisher('/Razor_IMU/Accel1', Razorimu, queue_size=10)
    rospy.init_node('Razor_IMUs', anonymous=True)
    rate = rospy.Rate(50)
    msg1 = Razorimu()
    msg2 = Razorimu()
    
    ser1.read(ser1.inWaiting()-210)
    ser2.read(ser2.inWaiting()-210)
    
    serial_data1 = [0,0,0,0,0,0,0,0,0,0]
    serial_data2 = [0,0,0,0,0,0,0,0,0,0]

    while True:
        cycle = cycle+1
        read_from_serial1()
        read_from_serial2()
        
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
        
        msg2.time_stamp = serial_data2[0]
        msg2.acc_x = serial_data2[1]
        msg2.acc_y = serial_data2[2]
        msg2.acc_z = serial_data2[3]
        msg2.gyro_x = serial_data2[4]
        msg2.gyro_y = serial_data2[5]
        msg2.gyro_z = serial_data2[6]
        msg2.mag_x = serial_data2[7]
        msg2.mag_y = serial_data2[8]
        msg2.mag_z = serial_data2[9]
        
        pub1.publish(msg1)
        pub2.publish(msg2)
        rate.sleep()
        
        

        
        
        
