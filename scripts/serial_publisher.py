#!/usr/bin/env python
# license removed for brevity
import rospy, serial, time, signal
from std_msgs.msg import String
from serial_interface.msg import Razorimu

ser1 = serial.Serial('/dev/ttyS2',115200)
latest_received1 = '0,0,0,0,0,0,0,0,0,0'
buffer_bytes1 = b''

ser2 = serial.Serial('/dev/ttyACM0',115200)
latest_received2 = '0,0,0,0,0,0,0,0,0,0'
buffer_bytes2 = b''

def keyboardInterruptHandler(signal,frame):
    print("\ninterrupted")
    exit(0)

signal.signal(signal.SIGINT,keyboardInterruptHandler)

def read_from_serial1():
    global latest_received1, buffer_bytes1
    serial_data = []
    bytesToRead = ser1.inWaiting()
    temp_bytes = ser1.read(bytesToRead)
    buffer_bytes1 = buffer_bytes1 + temp_bytes
    buffer_string = buffer_bytes1.decode()
    lines = buffer_string.split('\r\n')
    filter_lines = list(filter(None,lines))
    if len(filter_lines) > 1:
        latest_received1 = filter_lines[-2]
        buffer_bytes1 = temp_bytes
    else:
        print("Not enough serial input, using last available")
    splitline = latest_received1.split(',')
    for x in splitline:
        serial_data.append(float(x))
    if len(serial_data) < 10:
        print("exitting due to error on serial line 1")
        exit(0)
    else:
        return(serial_data)

def read_from_serial2():
    global latest_received2, buffer_bytes2
    serial_data = []
    bytesToRead = ser2.inWaiting()
    print("bytesToRead")
    print(bytesToRead)
    temp_bytes = ser2.read(bytesToRead)
    print("temp_bytes")
    print(temp_bytes)
    buffer_bytes2 = buffer_bytes2 + temp_bytes
    print("buffer_bytes2")
    print(buffer_bytes2)
    buffer_string = buffer_bytes2.decode()
    print("buffer_string")
    print(buffer_string)
    lines = buffer_string.split('\r\n')
    print("lines")
    print(lines)
    filter_lines = list(filter(None,lines))
    print("filter lines")
    print(filter_lines)
    if len(filter_lines) > 1:
        latest_received2 = filter_lines[-2]
        buffer_bytes2 = temp_bytes
    else:
        print("Not enough serial input, using last available")
    print("latest received")
    print(latest_received2)
    splitline = latest_received2.split(',')
    print("splitline")
    print(splitline)
    for x in splitline:
        serial_data.append(float(x))
    print("serial_data")
    print(serial_data)
    if len(serial_data) < 10:
        exit(0)
    else:
        return(serial_data)
        
def read_from_serial1_v2():
    global latest_received1, buffer_bytes1
    serial_data = []
    bytesToRead = ser1.inWaiting()
    if bytesToRead < 67:
        print("Not enough serial input, using last available")
    else:
        temp_bytes = ser1.read(bytesToRead)
        buffer_bytes1 = buffer_bytes1 + temp_bytes
        buffer_string = buffer_bytes1.decode()
        lines = buffer_string.split('\r\n')
        filter_lines = list(filter(None,lines))
        if len(filter_lines) > 1:
            latest_received1 = filter_lines[-2]
            buffer_bytes1 = temp_bytes
        else:
            print("Not enough serial input, using last available (due to lines)")      
    splitline = latest_received1.split(',')
    for x in splitline:
        serial_data.append(float(x))
    if len(serial_data) < 10:
        exit(0)
    else:
        return(serial_data)

def read_from_serial2_v2():
    global latest_received2, buffer_bytes2
    serial_data = []
    bytesToRead = ser2.inWaiting()
    if bytesToRead < 67:
        print("Not enough serial input, using last available")
    else:
        temp_bytes = ser2.read(bytesToRead)
        buffer_bytes2 = buffer_bytes2 + temp_bytes
        buffer_string = buffer_bytes2.decode()
        lines = buffer_string.split('\r\n')
        filter_lines = list(filter(None,lines))
        if len(filter_lines) > 1:
            latest_received2 = filter_lines[-2]
            buffer_bytes2 = temp_bytes
        else:
            print("Not enough serial input, using last available (due to lines)")      
    splitline = latest_received2.split(',')
    for x in splitline:
        serial_data.append(float(x))
    if len(serial_data) < 10:
        exit(0)
    else:
        return(serial_data)

if __name__ == '__main__':
    pub1 = rospy.Publisher('/Razor_IMUs/IMU1', Razorimu, queue_size=10)
    pub2 = rospy.Publisher('/Razor_IMUs/IMU2', Razorimu, queue_size=10)
    rospy.init_node('Razor_IMUs', anonymous=True)
    rate = rospy.Rate(50)
    msg1 = Razorimu()
    msg2 = Razorimu()
    while True:
        serial_data1 = read_from_serial1_v2()
        #serial_data1 = [0,1,2,3,4,5,6,7,8,9]
        serial_data2 = read_from_serial2_v2()
        
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
        
        rospy.loginfo(msg1)
        rospy.loginfo(msg2)
        pub1.publish(msg1)
        pub2.publish(msg2)
        rate.sleep()
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
