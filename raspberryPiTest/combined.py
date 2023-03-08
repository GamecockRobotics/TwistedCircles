import socket
import serial
import time

ser = serial.Serial('/dev/ttyACM1', 9600)
if ser.isOpen():
	ser.close()
ser.open()
ser.isOpen()

serverMAC = 'CC:F9:E4:9B:77:A0'

port = 7

s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
s.connect((serverMAC, port))

while True:
	data = ser.readline().decode()
	if data == "quit":
		break
	s.send(bytes(data, 'UTF-8'))
s.close()
ser.close()
