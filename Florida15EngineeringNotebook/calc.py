import math

x1 = -5.5
y1 = 0

x2 = 5.5
y2 = 0

# rotates (a,b) 1 degree around (c,d)
def rotate(a, b, c, d):
    return (math.sqrt((a - c)**2 + (b-d)**2)*math.sin(math.atan((b-d)/(a-c))+0.0174533)-, math.sqrt((a - c)**2 + (b-d)**2)*math.cos(math.atan((b-d)/(a-c))+0.0174533))

def arotate(a, b, c, d):
    return (math.sqrt((a - c)**2 + (b-d)**2)*math.sin(math.atan((b-d)/(a-c))+0.0174533), math.sqrt((a - c)**2 + (b-d)**2)*math.cos(math.atan((b-d)/(a-c))-0.0174533))


for i in range(0, 90):
    print ("("+str(x1)+","+str(y1)+")")
    x1, y1 = rotate(x1, y1, x2, y2)
    print ("("+str(x2)+","+str(y2)+")")
    x2, y2 = arotate(x2, y2, x1, y1)

