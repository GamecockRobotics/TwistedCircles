from pydraw import *
import time
import keyboard as k

# 416 416

mm = 3660 / 600

goalLoc = 279.4 / mm


x = 1830 / mm
y = 100 / mm
theta = 180

oldX = x
oldY = y
angleDiff = 0

screen = Screen(800, 600)
line = Line(screen, 600, 0, 600, 600)
redGoal = Oval(screen, Location(goalLoc, goalLoc), 393.7 / mm, 393.7 / mm, Color('red'))
blueGoal = Oval(screen, Location(600 - goalLoc - 393.7 / mm, 600 - goalLoc - 393.7 / mm), 393.7 / mm, 393.7 / mm, Color('blue'))
robot = Image(screen, "include/pros/turtle.png", Location(x, y), 40, 80, rotation=theta)
textx = Text(screen, str(x), 610, 30)
texty = Text(screen, str(y), 610, 60)
texttheta = Text(screen, "180", 610, 90)
texttext = Text(screen, "hdajslk", 610, 120)
screen.update()

while True:
    fps = 10
    robot.moveto(Location(x, y))
    newLine = Line(screen, oldX, oldY, x, y, Color("brown"))
    robot.rotation(theta)

    textx.remove()
    texty.remove()
    texttheta.remove()
    texttext.remove()
    textx = Text(screen,"x: "+ str(x), 610, 30)
    texty = Text(screen,"y: "+ str(y), 610, 60)
    texttheta = Text(screen,"theta: "+ str(theta), 610, 90)
    texttext = Text(screen,"djhfalkhfk", 610, 120)

    
    oldX = x
    oldY = y

    if k.is_pressed("d"):
        x += 5
    if k.is_pressed("a"):
        x -= 5
    if k.is_pressed("w"):
        y -= 5
    if k.is_pressed("s"):
        y += 5
    if k.is_pressed("q"):
        theta += 2
    if k.is_pressed("e"):
        theta -= 2



    screen.update()
    screen.sleep(1 / fps)