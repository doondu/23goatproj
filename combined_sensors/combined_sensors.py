import time
import board
import adafruit_bmp280
import RPi.GPIO as GPIO
import numpy as np

#For ebimu
import serial
import datetime
#For multiprocessing
import multiprocessing

def ebimu_process(n):
    #Make file with datetime and open
    nowTime = str(datetime.datetime.now())
    fileName = nowTime[:10]+"_ebimu_"+nowTime[11:21]
    try:
        f = open(fileName, 'w')
        #Error log
        log = open("log.txt",'w')
    except:
        print("Failed to open file")

    #Ebimu
    ser = serial.Serial('/dev/ttyUSB0',115200,timeout=0.001) #when connect to usb
    #ser = serial.Serial('/dev/ttyAMA1',115200,timeout=0.001) #when connect to pins (tx4,rx5)
    buf = "" # for ebimu
    while True:
        #Read ebimu values
        while ser.inWaiting():
            data = str(ser.read()).strip()
            buf += data # buffering
            if data[3] == "n": # last data of one line is '\n' so when data[3] is n then do decode 
                buf = buf.replace("'","") # remove (') and (b) because data has (') and (b) like this b'10.55' 
                buf = buf.replace("b","") 
                
                # split each data
                try :
                    roll, pitch, yaw, x, y, z = map(float,buf[1:-4].split(','))
                except Exception as e:
                    print("Error from data processing : ", e)
                    log.write(str(e)+"\n")
                    buf = ""
                    continue
                
                try :
                    datas = [roll,pitch,yaw,x,y,z]
                    writeString = "*"+str(datas)[1:-1]+"\n"
                    f.write(writeString)
                except Exception as e:
                    print("Error from file writing : ", e)
                    log.write(str(e)+"\n") # it also can make error but .. maybe.. i dont think so
                    continue

                print(roll,pitch,yaw,x,y,z)
                buf = ""

if __name__ == '__main__':
    eb_p = multiprocessing.Process(target=ebimu_process, args=(1,))
    eb_p.start()

    #Bmp280
    i2c = board.I2C()  
    bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c)
    bmp280.sea_level_pressure = 1019.5 #Change with location

    #ServoMotor
    GPIO.setmode(GPIO.BCM)
    servo_pin = 18
    GPIO.setwarnings(False)
    GPIO.setup(servo_pin, GPIO.OUT)
    pwm = GPIO.PWM(servo_pin, 50)
    pwm.start(2.5) #Check start num

    #Average algorithm
    def moving_average(data, window):
        moving_avg = []
        for i in range(len(data)):
            if i < window:
                moving_avg.append(data[i])
            else:
                partial_avg = np.mean(data[i - window + 1 : i + 1])
                moving_avg.append(partial_avg)
        return moving_avg
    window = 3
    data = []
    moving_std = []
    alpha = 0.25
    beta = 0.125
    temp_altitude = []
    init_altitude = 0  
    
    #Make file with datetime and open
    now = str(datetime.datetime.now())
    fileN = now[:10]+"_bmp_"+now[11:21]
    fileS = now[:10]+"_servoLog_"+now[11:21]
    try:
       f = open(fileN, 'w')
       s = open(fileS, 'w')
    except:
       print("Failed to open file, bmp")

    #Bmp280
    while True:
        altitude = bmp280.altitude
        #Calibration
        if init_altitude == 0:
            init_altitude = altitude
        
        cali_altitude = altitude - init_altitude
        result = cali_altitude
        data.append(result)

        if len(data) < window:
            data.append(init_altitude)
        
        ma = moving_average(data, window)
        ma = np.array(ma)

        estimated = ma[-1]
        estimated = (1 - alpha) * estimated + alpha * result
        var = np.var(data[-window:])
        var = (1 - beta) * var + beta * abs(result - estimated)
        #Result
        print("Calibration Altitude: ", cali_altitude)
        
        #Checking and moving servo
        if abs(cali_altitude - estimated) <= np.sqrt(var):
            if cali_altitude > 0.5:
                try:
                    # **************************************** Change pwm value
                    pwm.ChangeDutyCycle(7.5)
                    time.sleep(0.5)
                    pwm.ChangeDutyCycle(2.5)
                    time.sleep(0.5)
                    #Write a servo log 
                    s.write(f"{datetime.datetime.now()} Servo open log: {cali_altitude} m\n")
                    # ****************************************
                except KeyboardInterrupt:
                    GPIO.cleanup()
                    pass
        
        #Write bmp280 data
        f.write(f"Calibration altitude: {cali_altitude} m\n")
        