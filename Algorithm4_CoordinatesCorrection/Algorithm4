from random import random
import numpy as np
from numpy import sin,cos
import matplotlib.pyplot as plt
from Algorithm1 import Algorithm1

#######################################################
#        BRIEF:
#       ** THIS ALGORITHM TAKES AS INPUT THE (X,Y) COORDINATES SENT BY THE ROBOT AND 
#         CORRECTS THOSE COORDINATES TO CREATE A RECTANGLE TO USE WITH ALGORITHM1
#       ** IT IS CURRENTLY USING RANDOM VALUES 

#######################################################

Robot_width = 650
Robot_length = 684
Interpolation_Step, U1, U2 = 2000, 6000, 3000
pi = np.pi

def transform(v1,v2,alpha):
    return [cos(alpha)*np.array(v1)-sin(alpha)*np.array(v2),sin(alpha)*np.array(v1)+cos(alpha)*np.array(v2)]

def distance (x1, x2, y1, y2):
    return (np.sqrt((y2-y1)**2+(x2-x1)**2))


# CREATING THE RANDOM RECTANGLE
alpha = np.pi/2*random()
H     = 10000+1000*random()
L     = 15000+1000*random()

p1=np.array([0,0])
p2=np.array([0,H])
p3=np.array([L,H])
p4=np.array([L,0])

border_X=np.array([p1[0],p2[0],p3[0],p4[0]])+10000*random()
border_Y=np.array([p1[1],p2[1],p3[1],p4[1]])+10000*random()
X,Y=transform(border_X,border_Y,alpha)
q=2
X2=np.array([i+q*200*random()for i in X])
Y2=np.array([i+q*200*random()for i in Y])

# CALCULATING THE CORRECTED RECTANGLES COORDINATES
slope1 = (Y2[1]-Y2[0])/(X2[1]-X2[0])
slope2 = -(1/slope1)
intercept1 = Y2[0]-slope1*X2[0]
intercept2 = Y2[1]-slope2*X2[1]
intercept3 = Y2[2]-slope1*X2[2]
intercept4 = Y2[3]-slope2*X2[3]

xp3 = (intercept3-intercept2)/ (slope2-slope1)
yp3 = slope1 * xp3 + intercept3
xp4 = (intercept4-intercept3)/ (slope1-slope2)
yp4 = slope1 * xp4 + intercept3

x_correction = np.array([X2[0], X2[1], xp3, xp4])
y_correction = np.array([Y2[0], Y2[1], yp3, yp4])

Panel_length = distance(x_correction[0],x_correction[1],y_correction[0],y_correction[1])
Panel_width  = distance(x_correction[1],x_correction[2],y_correction[1],y_correction[2])


#translate_x = Trajectory[0]

# ROTATING THE TRAJECTORY:
Trajectory = Algorithm1([0,0],
                        Robot_width,Robot_length,
                        Panel_width, Panel_length,
                        Interpolation_Step, U1, U2 )

#xp1  = Trajectory[0][0]
#yp1  = Trajectory[1][0]
#
#origin_x = cos(alpha)*(Robot_width/2) - sin(alpha)*(Robot_length/2) + Trajectory[0][0]
#origin_y = sin(alpha)*(Robot_length/2) + cos(alpha)*(Robot_width/2) + Trajectory[1][0]
#print(origin_x,origin_y)
#plt.scatter(origin_x,origin_y,color='orange')
#plt.scatter(xp1,yp1,color='purple')

#rotated_trajectory = np.empty((2,len(Trajectory))) 
X,Y= transform(Trajectory[0],Trajectory[1],alpha)

X = X + x_correction[0]
Y = Y + y_correction[0]
#for i in range(len(Trajectory[0])):
    #X.append(cos(alpha)*Trajectory[0][i] - sin(alpha)*Trajectory[1][i] + Trajectory[0][i])
    #Y.append(sin(alpha)*Trajectory[1][i] + cos(alpha)*Trajectory[0][i] + Trajectory[1][i])

#print(rotated_trajectory)

plt.plot(X,Y)
plt.scatter(X,Y,color='green',s=20)
plt.scatter(0,0 , color='green')
plt.scatter(X2,Y2,color='blue')
plt.scatter(x_correction,y_correction,color='pink')
plt.plot(np.append(X2,X2[0]),np.append(Y2,Y2[0]),color='blue')
plt.plot(np.append(x_correction,x_correction[0]),np.append(y_correction,y_correction[0]),color='pink')

plt.axis('equal')
plt.grid('both')
plt.show()
