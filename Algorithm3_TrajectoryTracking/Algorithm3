import numpy as np
import matplotlib.pyplot as plt
import sympy as sp

##################################
#   BRIEF: 

#   **THIS ALGORITHM PLOTS THE SURFACE SWEPT BY THE ROBOT 
#   (USES THE SINUS FUNCTION WHICH SHOULD BE REPLACED BY THE PLOT OF THE ACTUAL ROBOT TRAJECTORY)

##################################

# d=0.25
width = 0.25
xfunc=np.linspace(-np.pi,np.pi,1000)
# yfunc=0.5*xfunc*np.abs(np.sin(xfunc))
yfunc=np.sin(xfunc)

#x, y, xc, yc, d, a  = sp.symbols('x y xc yc d a')
#eq1 = np.square(x-xc) + np.square(y-yc)-np.square(d)
#eq2 = ((y-yc)/(x-xc)) + (1/a)
#
#solution = sp.solve((eq1, eq2), (x, y))
#print(solution)
d=0.5
X1=[]
Y1=[]
X2=[]
Y2=[]
for i in range(len(xfunc)-1):
    a=(yfunc[i+1]-yfunc[i])/(xfunc[i+1]-xfunc[i])
    xc=xfunc[i]
    yc=yfunc[i]
    x1=-a*d/np.sqrt(a**2 + 1) + xc
    y1=d/np.sqrt(a**2 + 1) + yc
    x2=a*d/np.sqrt(a**2 + 1) + xc
    y2=-d/np.sqrt(a**2 + 1) + yc
    
    X1.append(x1)
    X2.append(x2)
    Y1.append(y1)
    Y2.append(y2)


    
    # plt.plot([xfunc[i]-d,xfunc[i]+d],[a*(xfunc[i]-d-xfunc[i])+yfunc[i],a*(xfunc[i]+d-xfunc[i])+yfunc[i]])
    # plt.plot([xfunc[i]-d,xfunc[i]+d],[-1/a*(xfunc[i]-d-xfunc[i])+yfunc[i],-1/a*(xfunc[i]+d-xfunc[i])+yfunc[i]],color='green') #perpendiculire
    
    #plt.scatter(solution[0][0], solution[0][1], )
    #plt.scatter(solution[1][0], solution[1][1])
X2=X2[::-1]
Y2=Y2[::-1]
plt.plot(X1+X2+[X1[0]],Y1+Y2+[Y1[0]])
plt.fill(X1+X2,Y1+Y2,color=(0,1,1,0.5))

plt.grid('both')
plt.axis('equal')
plt.show()
