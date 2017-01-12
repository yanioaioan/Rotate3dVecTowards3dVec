#ROTATE VECTOR TO ANOTHER WITH EULER ANGLES (from vector to Euler intuitively)

import maya.cmds as cmds
import math
import copy 

cmds.file(new=True, force=True)



def cross(a, b):
    c = [a[1]*b[2] - a[2]*b[1],
         a[2]*b[0] - a[0]*b[2],
         a[0]*b[1] - a[1]*b[0]]

    return c

def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))

def findangle(v1, v2):
  return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))

#mutating V
def normalize(V):
    #ax,ay,az = copy.copy(V)
    print V
    
    if V[0]!=0 or V[1]!=0 or V[2]!=0:
    	len = math.sqrt((V[0] * V[0]) + (V[1] * V[1]) + (V[2] * V[2]))
    	V[0] = V[0] / len
    	V[1] = V[1] / len
    	V[2] = V[2] / len
    
#NOT CURRENTLY USED
#not mutating V 
def getNormalized(V):
    ax,ay,az = copy.copy(V)
    len = math.sqrt((V[0] * V[0]) + (V[1] * V[1]) + (V[2] * V[2])) 
    ax = V[0] / len
    ay = V[1] / len
    az = V[2] / len
    
    return [ax,ay,az]
    
def subtract(endVec, startVec):
	retVec=[]
	retVec.append(endVec[0]-startVec[0])
	retVec.append(endVec[1]-startVec[1])
	retVec.append(endVec[2]-startVec[2])
	
	return retVec
	
	
def toEuler( x, y, z, angle):
	s=math.sin(angle)
	c=math.cos(angle)
	t=1-c
	#if axis is not already normalised then uncomment this
	#magnitude = math.sqrt(x*x + y*y + z*z);
	#if (magnitude==0):
	#    print 'magnitude of axis of rotation 0'
	#    exit()
	# x /= magnitude;
	# y /= magnitude;
	# z /= magnitude;
	if x*y*t + z*s > 0.998:# north pole singularity detected
	    heading = 2*math.atan2(x*math.sin(angle/2),math.cos(angle/2))
	    attitude = math.pi/2
	    bank = 0
	    return bank,heading,attitude
	
	if ((x*y*t + z*s) < -0.998):#south pole singularity detected
		heading = -2*math.atan2(x*math.sin(angle/2),math.cos(angle/2));
		attitude = -math.pi/2;
		bank = 0
		return bank,heading,attitude
	
	heading = math.atan2(y * s- x * z * t , 1 - (y*y+ z*z ) * t);
	attitude = math.asin(x * y * t + z * s) ;
	bank = math.atan2(x * s - y * z * t , 1 - (x*x + z*z) * t);
	
	return bank,heading,attitude


def rotateAbout(rotAboutAxis, genericLSysAngleDegrees):
    
    #rotAboutAxis = [1,0,0]#we know what axis we want to rotate about
    myangle=genericLSysAngle * math.pi/180 ##we know our angle and transform it to radians
    '''Quaternion built'''
    qx = rotAboutAxis[0] * math.sin(myangle/2)
    qy = rotAboutAxis[1] * math.sin(myangle/2)
    qz = rotAboutAxis[2] * math.sin(myangle/2)
    qw = math.cos(myangle/2)
    Q=[qx,qy,qz,qw]
    return  Q
  
def EulerAnglesFromAxisQuat(rotAboutAxis, genericLSysAngleDegrees):
    qx,qy,qz,qw = rotateAbout(rotAboutAxis, genericLSysAngleDegrees)
    
    '''Yaw Pitch Roll calculation from quaternion'''
    heading = math.atan2(2*qy*qw-2*qx*qz , 1 - 2*qy*qy - 2*qz*qz)#z
    attitude = math.asin(2*qx*qy+2*qz*qw)#y
    bank = math.atan2(2*qx*qw-2*qy*qz , 1 - 2*qx*qx - 2*qz*qz)# Attention this is euler angle X ..not z
    
    angles=[bank,heading,attitude]
    
    #print "angles=%s"%(angles)
    
    return angles

c1=cmds.polyCube()
cmds.move(1,3,2)

c2=cmds.polyCube()
cmds.move(5,5,0)

t1=cmds.xform(c1[0], query=True, translation=True)
t2=cmds.xform(c2[0], query=True, translation=True)

'''!!!!!'''
#At first you would have to subtract vector one from vector two in order to get vector two relative to vector one. With these values you can calculate Euler angles.
vecDiff=subtract(t1,t2)
'''!!!!!'''

#In order "roll <- pitch <- yaw" calculation can be done as follows:
#To calculate the yaw you calculate the tangent of the two planar axes (x and z) considering the quadrant.
#yaw = atan2(x, z) *180.0/PI;
#Pitch is quite the same but as its plane is rotated along with yaw the 'adjacent' is on two axis. In order to find its length we will have to use the Pythagorean theorem.



padj = math.sqrt(math.pow(vecDiff[0], 2) + math.pow(vecDiff[2], 2))
yaw = math.atan2(vecDiff[0], vecDiff[2]) *(180.0/math.pi)
pitch = math.atan2(padj, vecDiff[1]) * (180.0/math.pi)

'''Use either of the follwing commands'''
#cmds.rotate(pitch,yaw, 0)
cmds.xform(ro=(pitch, yaw, 0), absolute=True)


'''
Notes:

Roll can not be calculated as a vector has no rotation around its own axis. I usually set it to 0.
The length of your vector is lost and can not be converted back.
In Euler the order of your axes matters, mix them up and you will get different results
'''

'''
normalize(t2)
normalize(t1)

t1Dott2=dotproduct(t1, t2)
c1Length=length(t1)
c2Length=length(t2)


angle = math.acos(t1Dott2/ (c1Length*c2Length))
rotAxis=cross(t1, t2)
print angle*(180.0/math.pi)
normalize(rotAxis)
'''

