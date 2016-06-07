'''ALTERNATIVE WAY OF CALCULATING EULER-ANGLES using SPHERICAL COORDINATES'''

'''rotate vector cube face1 to sphere position as sphere moves 
BY..
calculating the vector from cube's face 1 center 2 sphere position vecotr (spherepos - face1 )
then derive euler angles by calculating spherical coordinates (in WORLD SPACE)'''

#NOT BY..
#calculating the cube's face 1 center and axis&angle between the 2 vectors (face1 - spherepos)
#then derive euler angles by forming a quaternion from rotAxis and Angle and rotate accordingly (using RELATIVE rotation)
#NOTE: LINE 187: GRAB the new final rotation after Relatively rotated/concatenated with the previous command (line 184), and set the keyframes based on the absolute rotation further down


import maya.cmds as cmds
import math
import random
import time
import maya.mel as mel

cmds.file(force=True, new=True)

cmds.select(all=True)
cmds.delete()


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

def normalize(V):
    #ax,ay,az = copy.copy(V)
    len = math.sqrt((V[0] * V[0]) + (V[1] * V[1]) + (V[2] * V[2])) 
    V[0] = V[0] / len
    V[1] = V[1] / len
    V[2] = V[2] / len

def rotateAbout(rotAboutAxis, genericLSysAngle):
    
    #rotAboutAxis = [1,0,0]#we know what axis we want to rotate about
    myangle=genericLSysAngle * math.pi/180 ##we know our angle and transform it to radians
    '''Quaternion built'''
    qx = rotAboutAxis[0] * math.sin(myangle/2)
    qy = rotAboutAxis[1] * math.sin(myangle/2)
    qz = rotAboutAxis[2] * math.sin(myangle/2)
    qw = math.cos(myangle/2)
    Q=[qx,qy,qz,qw]
    return  Q
  
def EulerAnglesFromAxisQuat(rotAboutAxis, genericLSysAngle):
    qx,qy,qz,qw = rotateAbout(rotAboutAxis, genericLSysAngle)
    
    '''Yaw Pitch Roll calculation from quaternion'''
    heading = math.atan2(2*qy*qw-2*qx*qz , 1 - 2*qy*qy - 2*qz*qz)#z
    attitude = math.asin(2*qx*qy+2*qz*qw)#y
    bank = math.atan2(2*qx*qw-2*qy*qz , 1 - 2*qx*qx - 2*qz*qz)# Attention this is euler angle X ..not z
    
    angles=[bank,heading,attitude]
    
    #print "angles=%s"%(angles)
    
    return angles
    
#find center of face facenum
def findCenterOfFace(facenum):
	
	facepositionWS=cmds.xform(mycube[0]+".f["+str(facenum)+"]", query=True, translation=True, worldSpace=True)
	#print 'face %d, faceposition=%s'%(facenum,facepositionWS)
	
	#find center of face (average the face's vertex posiitons)
	sumX=0
	sumY=0
	sumZ=0
	
	#number of this face's vertices
	faceVertices=len(facepositionWS)/3
	#each 3d point has 3 coordinates x,y,z
	vertexElements=3
	#print "faceVertices=%d"%(faceVertices)
	
	#iterate over the number of this face's vertices & find the sum for each coordinate direction
	for v in range(0,len(facepositionWS),vertexElements):
	    sumX=sumX + facepositionWS[v]
	    sumY=sumY + facepositionWS[v+1]
	    sumZ=sumZ + facepositionWS[v+2]
	    
	#average position of vertices of each face - the face center
	cubefaceCenter = [sumX/faceVertices, sumY/faceVertices, sumZ/faceVertices]
	return cubefaceCenter

def subtract(spherePos, cubefaceCenter):
	retVec=[]
	retVec.append(spherePos[0]-cubefaceCenter[0])
	retVec.append(spherePos[1]-cubefaceCenter[1])
	retVec.append(spherePos[2]-cubefaceCenter[2])
	
	return retVec
	


#sphere
mysphere=cmds.polySphere()
cmds.xform(mysphere, translation=[1,3,0])
spherePos=cmds.xform(mysphere, translation=True, query=True)


#cube
mycube=cmds.polyCube()
cmds.xform(mycube, translation=[3.0001,0,0])
cubePos=cmds.xform(mycube, translation=True, query=True)


spherePos=cmds.xform(mysphere, translation=True, query=True)
print spherePos

#find center of face 1
cubefaceCenter=findCenterOfFace(1)


#source: http://stackoverflow.com/questions/15101103/euler-angles-between-two-3d-vectors
#Alernative way using Spherical coordinates
#subtract v1 from v2 as in v2-v1
v2_v1=subtract(spherePos, cubefaceCenter)
x=v2_v1[0]
y=v2_v1[1]
z=v2_v1[2]
yaw = math.atan2(float(x), float(z)) *180.0/math.pi;
padj = math.sqrt( math.pow(float(x), 2) + math.pow(float(z), 2)); 
pitch = math.atan2( float(padj), float(y)) *180.0/math.pi;
cmds.xform(mycube, rotation=[pitch, yaw, 0])


'''
#find rot axis
rotAxis=cross(cubefaceCenter,spherePos)
#normalize rot axis
normalize(rotAxis)
print "rotAxis=%s"%rotAxis
#find angle


angle=findangle(cubefaceCenter,spherePos)
print "angle=%s degrees"% (angle*(180/math.pi))
angle=angle*(180/math.pi)

euAngles=EulerAnglesFromAxisQuat(rotAxis, angle)
print euAngles[0]*(180/math.pi)
print euAngles[1]*(180/math.pi)
print euAngles[2]*(180/math.pi)

#convert to degrees
bank=euAngles[0]*(180/math.pi)
heading=euAngles[1]*(180/math.pi)
attitude=euAngles[2]*(180/math.pi)

#now rotate cube on rotaxis by angle
cmds.xform(mycube, rotation=[bank, heading, attitude], r=True)
'''



for i in range(1,121,1):
	
	#move the sphere
	spherePos[0]=spherePos[0]-math.sin( i/10.0 ) / 2.0
	spherePos[2]=spherePos[2]+math.cos( i/10.0 ) / 2.0
	
	cmds.xform(mysphere, translation=spherePos)
	
	spherePos=cmds.xform(mysphere, translation=True, query=True)
	#print spherePos[0]
	
	cmds.select(mysphere)	
	cmds.setKeyframe( v=spherePos[0], at='translateX', t = i)
	cmds.setKeyframe( v=spherePos[1], at='translateY', t = i)
	cmds.setKeyframe( v=spherePos[2], at='translateZ', t = i)
		
		
	#find center of face 1 of cube
	cubefaceCenter=findCenterOfFace(1)
	print cubefaceCenter
	
	
	v2_v1=subtract(spherePos, cubefaceCenter)
	x=v2_v1[0]
	y=v2_v1[1]
	z=v2_v1[2]
	yaw = math.atan2(float(x), float(z)) *180.0/math.pi;
	padj = math.sqrt( math.pow(float(x), 2) + math.pow(float(z), 2)); 
	pitch = math.atan2( float(padj), float(y)) *180.0/math.pi;
	cmds.xform(mycube, rotation=[pitch, yaw, 0])

	
	'''
	#find rot axis
	rotAxis=cross(cubefaceCenter,spherePos)
	#normalize rot axis
	normalize(rotAxis)
	#print "rotAxis=%s"%rotAxis
	
	#find angle		
	angle=findangle(cubefaceCenter,spherePos)		
	angle=angle*(180/math.pi)
	#print "angle=%s degrees"% (angle*(180/math.pi))
	
	euAngles=EulerAnglesFromAxisQuat(rotAxis, angle)
	#print euAngles[0]*(180/math.pi)
	#print euAngles[1]*(180/math.pi)
	#print euAngles[2]*(180/math.pi)
	
	#convert to degrees
	bank=euAngles[0]*(180/math.pi)
	heading=euAngles[1]*(180/math.pi)
	attitude=euAngles[2]*(180/math.pi)
	
	#now rotate cube on rotaxis by angle
	cmds.xform(mycube, rotation=[bank, heading, attitude], r=True)
	'''
	#new world space rotations after relative rotation concatenation
	#GRAB the new final rotation after Relatively rotated/concatenated with the previous command (line 184), and set the keyframes based on the absolute rotation further down
	newWorldrot=cmds.xform(mycube, rotation=True, query=True)	
	
	cmds.select(mycube)	
	cmds.setKeyframe( v=newWorldrot[0], at='rotateX', t = i)
	cmds.setKeyframe( v=newWorldrot[1], at='rotateY', t = i)
	cmds.setKeyframe( v=newWorldrot[2], at='rotateZ', t = i)
	
	
