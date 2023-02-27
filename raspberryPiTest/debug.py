import serial

ser = serial.Serial('/dev/tty.raspberrypi', timeout=1, baudrate=115000)

   
while True:
    out = ser.readline().decode()
    if out!='' : print (out)