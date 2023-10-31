import serial
import time
import threading
#pip install pynput
from pynput.keyboard import Key, Listener

ser = serial.Serial("/dev/ttyAMA0", 19200)
# output_ser = serial.Serial("/dev/ttyAMA1", 9600)
temp = "ST:"
euler = "SE:"
gyro = "SG:"
alt = "SA:"

alivethread = True
catch = False
def readthread(stream):
    global alivethread
    global catch
    line = ''
    while alivethread:
        for c in stream.read():
            print(c)
            if c == 72:
                catch = True
                stream.write("E".encode())

# thread = threading.Thread(target=readthread, args=(output_ser, ))
# thread.start()

#Send warning message with keyboard interrupt
def warning_release(key):
    if key == Key.space:
        warn = '0' * 55
        ser.write(warn.encode())

listener = Listener(on_release=warning_release)
listener.start()

while True:
    
    alt_buffer = ""
    temp_buffer = "" 
    euler_buffer = []
    gyro_buffer = ""
    received_data = ser.readline()
    time.sleep(0.25)
    
    data_left = ser.inWaiting()
    received_data += ser.read(data_left)
    received = received_data.decode('utf-8', 'ignore')
    received = list(received.split("\n"))
    
    for lines in received:
        print(lines)
        if lines.startswith(euler) and lines.endswith("EE:"):
            euler_buffer =   lines +"\r\n"
            # output_ser.write(euler_buffer.encode())


        if lines.startswith(temp) and lines.endswith("ET:"):
            temp_buffer = lines + "\r\n"
            # output_ser.write(temp_buffer.encode())


        if lines.startswith(alt) and lines.endswith("EA:"):
            alt_buffer = lines + "\r\n"
            print(alt_buffer)
            # output_ser.write(alt_buffer.encode())

        if lines.startswith("SP:") and lines.endswith("EP:"):
            press_buffer = lines + "\r\n"
            # output_ser.write(press_buffer.encode())

