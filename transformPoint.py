#   At the command line:
#   $ python transformPoint.py x y z
#   Where x y and z are the points that Kinect senses
#   Output will be robot coordinates (clipped for safety)

import math
import numpy as np
import sys

# Transform args at commandline into a vector
kinectPoint = np.asarray(sys.argv[1:4], dtype=np.float32)

## Orientation of Kinect relative to robot coordinates (in radians)
## Roll r, Pitch p, and Yaw y

r = -1.92
p = -1.57
y = 0

## Placement of Kinect relative to robot coordinates
xDist = -1.77
yDist = -0.2
zDist = 1.58

###############################
#  Matrix is represented as:  #
#                             #
#           a b c             #
#           d e f             #
#           g h i             #
#                             #
###############################

# Source: https://en.wikibooks.org/wiki/Robotics_Kinematics_and_Dynamics/Description_of_Position_and_Orientation#Forward_Mapping_2

# Forward implementation from RPY angles to rotation matrix
# Assumes the order of rotation is row, pitch, then yaw

a = math.cos(y) * math.cos(p)
b = math.cos(y) * math.sin(p) * math.sin(r) - math.sin(y) * math.cos(r)
c = math.cos(y) * math.sin(p) * math.cos(r) + math.sin(y) * math.sin(r)

d = math.sin(y) * math.cos(p)
e = math.sin(y) * math.sin(p) * math.sin(r) + math.cos(y) * math.cos(r)
f = math.sin(y) * math.sin(p) * math.cos(r) - math.cos(y) * math.sin(r)

g = math.sin(p) * -1
h = math.cos(p) * math.sin(r)
i = math.cos(p) * math.cos(r)

rotationMatrix = [[a,b,c], [d,e,f], [g,h,i]]
invRotationMatrix = np.linalg.inv(rotationMatrix)

#### Now to transform. Lets use a kinect point to find the equivalent robot point

def findRobotPoint(kinectPoint):
	#Step 1: Rotate Point
	rotatedPoint = np.dot(invRotationMatrix, kinectPoint) #might need to use invRotationMatrix instead.
	#Step 2: Translate Point
	translatedPoint = np.array([rotatedPoint[0]+xDist, rotatedPoint[1]+yDist, rotatedPoint[2]+zDist])
	return translatedPoint

def findHumanPoint(robotPoint):
	#Determine a point closer to the origin that the human can reach
	humanPoint = robotPoint * .9

	#Now make sure the point can be found by the robot by clipping the vector
	#First find magnitude of robot point vector (distance from origin)
	robotPointMagnitude = np.linalg.norm(robotPoint)
	#Create a vector, to iteratively clip the human point 
	robotUnitVector = robotPoint / robotPointMagnitude
	clippingVector = robotUnitVector*.001

	#Clip until one of the coordinates is under 0.7
	pointCeiling = .7
	while (humanPoint[0] > pointCeiling \
		or humanPoint[1] > pointCeiling \
		or humanPoint[2] > pointCeiling):
		humanPoint = humanPoint - clippingVector
	return humanPoint

def findGripperQuaternion(point):
	x = point[0]
	y = point[1]
	gripperQuaternion = [0,0,0,0]
	if (x >= 0 and y >= 0):
		gripperQuaternion[0] = -0.5
		gripperQuaternion[1] = -0.5
		gripperQuaternion[2] = 0.5
		gripperQuaternion[3] = 0.5
	elif (x >= 0 and y < 0):
		gripperQuaternion[0] = -0.70710678
		gripperQuaternion[1] = 0
		gripperQuaternion[2] = 0
		gripperQuaternion[3] = 0.70710678
	elif (x < 0 and y >= 0):
		gripperQuaternion[0] = 0
		gripperQuaternion[1] = -0.70710678
		gripperQuaternion[2] = 0.70710678118
		gripperQuaternion[3] = 0
	elif (x < 0 and y < 0):
		gripperQuaternion[0] = -0.5
		gripperQuaternion[1] = 0.5
		gripperQuaternion[2] = -0.5
		gripperQuaternion[3] = 0.5

	return gripperQuaternion


def toJSON(point):
	json = "{'x':" + str(point[0])          + \
			", 'y': " + str(point[1])       + \
			", 'z': " + str(point[2])       + \
			", 'ox': " + str(quaternion[0]) + \
			", 'oy': " + str(quaternion[1]) + \
			", 'oz': " + str(quaternion[2]) + \
			", 'ow': " + str(quaternion[3]) + "}"
	return json



robotPoint = findRobotPoint(kinectPoint)

humanPoint = findHumanPoint(robotPoint)

quaternion = findGripperQuaternion(humanPoint)

print toJSON(humanPoint)




