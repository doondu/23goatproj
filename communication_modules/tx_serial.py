import time
import serial
import RPi.GPIO as GPIO
import adafruit_bno055
import board
import busio

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)
timestamp = 0

last_val = 0xFFFF

#Eject
txWarnCheck = '0' * 20

ser = serial.Serial(
    port="/dev/ttyAMA0",
    baudrate=19200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
    )

def list_to_str(lst):
    temp_list = map(str, lst)
    return "".join(temp_list) + ":"
    

def cut_off(lst):
    ret = []

    for elem in lst:
        if  elem is None:
            ret.append(elem)
        else:
            ret.append(round(elem, 1))
    return ret

while 1:
    #Eject
    if ser.inWating():
        ejectline = ser.readline()
        ejectline += ser.read(ser.inWating())
        ejectline = ejectline.decode('utf-8', 'ignore')
        received = list(received.split("\n"))
        if(received.startswith(txWarnCheck) and received.endswith(txWarnCheck)):
            #pwm.ChangeDutyCycle(7.5)
            print("EJECT")
            time.sleep(3)

    cmd = ""
    str_cmd = ""
    st = "-s" + str(timestamp) + "-"
    ser.write(st.encode())
    ser.write("\n".encode())
    
    cmd += st + "\n"

    temp = sensor.temperature
    ser.write("st".encode())
    ser.write("\n".encode())
    
    str_cmd = "ST:" + str(temp)
    ser.write(str_cmd.encode())
    ser.write("ET\n".encode())
    
    cmd += str_cmd + "/n"

    accel = sensor.acceleration
    
    #gyro = sensor.gyro
    #gyro = cut_off(gyro)
    #str_cmd = "G: " + str(gyro)
    #ser.write(str_cmd.encode())
    #ser.write("\n".encode())
    
    euler = sensor.euler
    euler = cut_off(euler)
    str_cmd = "SE:" + str(euler)
    ser.write(str_cmd.encode())
    str_cmd = "EE:\n"
    ser.write(str_cmd.encode())

    cmd += str_cmd + "/n"

    ser.write("ed".encode())
    ser.write("\n".encode())
    
    cmd += "ed\n"

    ed = "-e" + str(timestamp) + "-"
    ser.write(ed.encode())
    ser.write("\n".encode())
    
    cmd += ed + "\n"

    timestamp += 1
    if len(cmd) < 55:
        pad = 55 - len(cmd)
        pad_str = "x" * pad
        cmd += pad_str
        
    print(len(cmd))

