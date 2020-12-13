#Kim, Ji
#using numpy from python package

#the pseudo code for the RRT
#xrand <- random(space)/ goal(1 out of 10 times)
#xnear <- nearest- neighbor(xrand, tree)
#u <- direction ( xnear,  x rand) #unit vector
#xnew <- x near + ut #t = step-size
#if(check-collision(xnew, xnear)) no collision
#    tree.add(xnew,xnear) # include node, edge

import numpy as np
import matplotlib.pyplot as plt
import random
import sys
import math
import time
sys.path.append('PythonAPI')

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')


print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')
else:
    print ('Failed connecting to remote API server')
    sys.exit('Could not connect to Vrep')

# get the handles of arm joints
err_code, armjoint1_handle = sim.simxGetObjectHandle(clientID,"UR5_joint1", sim.simx_opmode_blocking)
err_code, armjoint2_handle = sim.simxGetObjectHandle(clientID,"UR5_joint2", sim.simx_opmode_blocking)
err_code, armjoint3_handle = sim.simxGetObjectHandle(clientID,"UR5_joint3", sim.simx_opmode_blocking)
err_code, armjoint4_handle = sim.simxGetObjectHandle(clientID,"UR5_joint4", sim.simx_opmode_blocking)
err_code, armjoint5_handle = sim.simxGetObjectHandle(clientID,"UR5_joint5", sim.simx_opmode_blocking)
err_code, armjoint6_handle = sim.simxGetObjectHandle(clientID,"UR5_joint6", sim.simx_opmode_blocking)
# get the handles of hand joints
err_code, endeffector_handle = sim.simxGetObjectHandle(clientID,"suctionPad", sim.simx_opmode_blocking)


# set the arm to position control
sim.simxSetObjectIntParameter(clientID, armjoint1_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint1_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint2_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint2_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint3_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint3_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint4_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint4_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint5_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint5_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint6_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint6_handle, 2001, 1, sim.simx_opmode_oneshot)

# get the collision handles
collision_handle_list = []
for i in range(40):
    err_code, collision_handle = sim.simxGetCollisionHandle(clientID, "Collision" + str(i), sim.simx_opmode_blocking)
    sim.simxReadCollision(clientID, collision_handle, sim.simx_opmode_streaming)
    collision_handle_list.append(collision_handle)

# function to control the movement of the arm, the input are the angles of joint1, joint2, joint3, joint4, joint5, joint6. The unit are in degrees
def move_arm(armpose):
    armpose_convert = []
    for i in range(6):
        armpose_convert.append(round(armpose[i]/180 * math.pi,3))
    sim.simxPauseCommunication(clientID,True)
    sim.simxSetJointTargetPosition(clientID, armjoint1_handle, armpose_convert[0], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, armjoint2_handle, armpose_convert[1], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, armjoint3_handle, armpose_convert[2], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, armjoint4_handle, armpose_convert[3], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, armjoint5_handle, armpose_convert[4], sim.simx_opmode_oneshot) 
    sim.simxSetJointTargetPosition(clientID, armjoint6_handle, armpose_convert[5], sim.simx_opmode_oneshot)    
    sim.simxPauseCommunication(clientID,False)
    time.sleep(.5)

# function to check collision
def check_collision():
    collision_reading = np.zeros(40)
    is_collision = 0
    for i in range(40):
        collision_reading[i] = sim.simxReadCollision(clientID, collision_handle_list[i], sim.simx_opmode_buffer)[1]
        if collision_reading[i] == 1:
            is_collision = 1
    if is_collision == 1:
        print('Collision detected!')
        return 1
    else:
        return 0
    
#calculates the distance between two points
def dist(p1, p2):
    distance = math.sqrt(((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) + ((p1[2]-p2[2])**2))
    return distance

#example 1, initial position
move_arm([-20, 20, 70, 0, -90, 0])
position = sim.simxGetObjectPosition(clientID, endeffector_handle, -1, sim.simx_opmode_blocking)[1]
for i in range(3):
    position[i] = round(position[i],3)
xnew = position
#add the initial position to the tree
tree = []
tree.append(position)

#example 2, goal position
move_arm([10, -20, -75, 0, 90, 0])
position = sim.simxGetObjectPosition(clientID, endeffector_handle, -1, sim.simx_opmode_blocking)[1]
for i in range(3):
    position[i] = round(position[i],3)
xgoal = position

random.seed()
count = 1
goal = 0
#loop

while goal==0:
    #create random position in the space
    if count % 10 != 0:
        theta1 = random.randrange(-30,15)
        theta2 = random.randrange(-50,35)
        theta3 = random.randrange(-75,80)
        theta4 = random.choice([0,25,30])
        theta5 = random.choice([-90,90])
        theta6 = random.randrange(-90,90)

        theta6 = 0
        target_arm = [theta1, theta2, theta3, theta4, theta5, theta6]
        move_arm(target_arm)
        position = sim.simxGetObjectPosition(clientID, endeffector_handle, -1, sim.simx_opmode_blocking)[1]

        rPoint = [position[0],position[1],position[2]]
    #every 10 points its going to set the goal as random point
    else:
        rPoint = xgoal

    #finds the nearest node to the random point
    nearest = 1000000000000
    for i in range(len(tree)):
        distance = dist(tree[i],rPoint)
        if distance < nearest:
            nearest = distance
            index = i
    xnear = tree[index]
    ux = rPoint[0] - tree[index][0]
    uy = rPoint[1] - tree[index][1]
    uz = rPoint[2] - tree[index][2]

    #adds unit vector that has the direction to the random number with step size .5
    mag = math.sqrt(ux ** 2 + uy ** 2 + uz ** 2)
    uVector = [ux/mag,uy/mag,uz/mag]
    stepSize = .5

    for i in range(3):
        xnew[i] = xnear[i] + uVector[i] * stepSize
        xnew[i] = round(xnew[i],3)
    
    #check for collision, and if there is none add it to the tree node
    if check_collision() == 0:
        tree.append(xnew)
        print('random point: ' + str(rPoint) + ' xnew: ' + str(xnew))
    count += 1

    #if the distance between the new node and the goal is less than .05 end the loop
    if dist(xnew,xgoal) < .05:
        goal = 1 
    
# close the communication between collision handles
for i in range(40):
    sim.simxReadCollision(clientID, collision_handle_list[i], sim.simx_opmode_discontinue)

print ('Program ended')

