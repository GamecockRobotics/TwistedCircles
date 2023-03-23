import socket
from pydraw import *
import re
x: 432.432  
# Setting up Map
mm = 3660 / 600
x = 1830 / mm
y = 100 / mm
oldX = x
oldY = y
theta = 0
angleDiff = 0
robot = None
screen = None
text1 = None
text2 = None
text3 = None
text4 = None

def startMap():
    goalLoc = 279.4 / mm
    screen = Screen(800, 600)
    line = Line(screen, 600, 0, 600, 600)
    redGoal = Oval(screen, Location(goalLoc, goalLoc), 393.7 / mm, 393.7 / mm, Color('red'))
    blueGoal = Oval(screen, Location(600 - goalLoc - 393.7 / mm, 600 - goalLoc - 393.7 / mm), 393.7 / mm, 393.7 / mm, Color('blue'))
    robot = Image(screen, "turtle.png", Location(x, y), 40, 80, rotation=180)
    screen.update()

def updateMap(data):
    data = data.split("/t")
    x = int(data[0])
    y = int(data[1])
    theta = int(data[2])
    text = data[3]

    if text1:
        text1.remove()
        text2.remove()
        text3.remove()
        text4.remove()
    
    text1 = Text(screen,"x: "+ str(x), 610, 30)
    text2 = Text(screen,"y: "+ str(y), 610, 60)
    text3 = Text(screen,"theta: "+ str(theta), 610, 90)
    text4 = Text(screen, text, 610, 120)

    robot.moveto(Location(x, y))
    newLine = Line(screen, oldX, oldY, x, y)
    robot.rotation(theta)

    x = x + 10
    y = y + 10

    angleDiff = 2

    screen.update()

adapter_addr = 'CC:F9:E4:9B:77:A0'
port = 7  # Normal port for rfcomm?
buf_size = 1024

s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
s.bind((adapter_addr, port))
s.listen(1)



try:
    print('Listening for connection...')
    client, address = s.accept()
    print(f'Connected to {address}')
    startMap()

    while True:
        data = client.recv(buf_size).decode()
        if not re.search("[0-9]+.[0-9]+\\t[0-9]+.[0-9]+\\t[0-9]+.[0-9]+\\t[a-z]*", data):
            continue
        print("got value")
        if data:
            updateMap(data)
        if not data:
            break

except Exception as e:
    print(f'Something went wrong: {e}')
    s.close()