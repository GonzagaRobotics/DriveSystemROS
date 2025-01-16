import rclpy
from rclpy.node import Node

rclpy.init()
node = rclpy.create_node('driveNode')

publisher =node.create_publisher() #declare message type


#Check how speeds are actually calulated assuimg 100 is stopped
#assume less is back or left more is forward or right


FB=100 #forward back speed
LR=100 #left right speed

dir ='stoped' #values stopped FW BW LW RW FR FW BR BL

Direction= dirCalculate(FB, LR)





# testing
x=1

y=2

z=3
print ( x , y , z)

rclpy.spin(node)

#calculates the general direction of the Rover
def dirCalculate(FB, LR):
    Direction='empty'
    FrontDir='F'
    SideDir='L'
    if FB>100: 
        FrontDir='F'
    elif FB==100:
        FrontDir='W'
    else:
        FrontDir='B'
    
    if LR>100: 
        SideDir='F'
    elif LR==100:
        SideDir='W'
    else:
        SideDir='B'

    if FrontDir!= 'W' & SideDir!='W':
        Direction=FrontDir+SideDir
    elif FrontDir== 'W' & SideDir!='W':
        Direction=SideDir+FrontDir
    else:
        Direction='stopped'
    return Direction
