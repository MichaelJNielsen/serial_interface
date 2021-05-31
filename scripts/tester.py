#!/usr/bin/env python
import rospy, serial, time, signal
i = 0
j = 0
ser = serial.Serial('/dev/ttyS2',115200)
failed_bytes = 0
mini = 100
maxi = 0

def keyboardInterruptHandler(signal,frame):
    print("\ninterrupted")
    print("cleared: ", i, " times")
    print("failed: ", j, " times")
    print("failed bytes: ", failed_bytes)
    print("maximum bytes: ", maxi)
    print("minimum bytes: ", mini)
    exit(0)
signal.signal(signal.SIGINT,keyboardInterruptHandler)

def read_from_serial():
    global i, j, serial_data
    bytesToRead = ser.inWaiting()
    while bytesToRead > 140:
        j = j+1
        print("clearing: ", j)
        print("before clearing: ", bytesToRead)
        clearer = ser.readline()
        bytesToRead = ser.inWaiting()
        print("after clearing: ", bytesToRead)
        
    if bytesToRead < 65:
        i = i+1
        print("X - Not enough serial input, using last available",i)
    else:
        bytes = ser.readline()
        string = bytes.decode()
        splitline = string.split(',')
        temp_data = []
        for x in splitline:
            temp_data.append(float(x))
        serial_data = temp_data


if __name__=='__main__':
    rospy.init_node('Razor_tester',anonymous=True)
    rate = rospy.Rate(50)
    ser.read(ser.inWaiting()-140)
    serial_data = []

while True:
    read_from_serial()
    print(serial_data)
    rate.sleep()
    
    
    
    
    
    
    
    
    
    
    
    
    


