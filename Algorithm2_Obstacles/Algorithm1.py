#!/usr/bin/env python3
#Author: Ilef Mghirbi

import numpy as np 
from matplotlib import pyplot as plt

########################################################
#                  BRIEF:
#      THIS ALGORITHM CREATES P-SOLAR'S TRAJECTORY ON A RECTANGULAR PANEL FREE OF OBSTACLES
#      IT IS TO BE IMPORTED AND USED IN OTHER ALGORITHMS THAT USE THE TRAJECTORY ALGORITHM
#      Units are in [mm]

#           row/ligne | column  
# X == ARRAY  [0]     |  [i] 
# Y == ARRAY  [1]     |  [i]

###################     PARAMETERS INITIALIZATION   #########################

Robot_Width = 650 
Robot_Length = 684
Interpolation_Step = 2000
U1 = 6000
U2 = 3000
###################   ARRAYS   #################################################

#RE-INITIALZED LATER WITH ACCURATE VALUES 
Start_Points_Array = np.zeros((2,10))
End_Points_Array = np.zeros((2,10))
Trajectory_Points_Array = np.zeros((2,100)) 
Interm_X = []
Interm_Y = []
StartCommuting_point=np.zeros((2,10))
EndCommuting_point=np.zeros((2,10))

###################     FUNCTIONS    #########################################
def Algorithm1( start , Robot_Width, Robot_Length, Panel_Width, Panel_Length, Interpolation_Step, U1,U2): #borders is the 2-D array [P0[x0,y0],P1[x1,y1],P2[x2,y2],P3[x3,y3]]

    global Y_start
    global Y_end   
    global sections
    global Nbr_sections
    global Nbr_Checkpoints
    global Nbr_Targets
    global Start_Points_Array 
    global End_Points_Array 
    global Trajectory_Points_Array 
    global StartCommuting_point
    global EndCommuting_point

    Robot_Width = 650
    #hardcoded solar panel dimensions 35000*100000 (35m*40m):
    
    
    Y_start = int(Robot_Length/2)
    Y_end   = int(Panel_Length - (Robot_Length/2))

    Checkpoints = divmod(Y_end - Y_start , Interpolation_Step )
    Nbr_Checkpoints = int(Checkpoints[0])
    if (Checkpoints[1]==0):
        Nbr_Targets = Nbr_Checkpoints + 3
    else: 
        Nbr_Targets = Nbr_Checkpoints + 4

    Robot_Width_Float = np.ceil(Panel_Width/Robot_Width)
    Robot_Width = int(Panel_Width/Robot_Width_Float)
    Nbr_sections = int(Panel_Width/Robot_Width)

    #print('Nbr_sections= ', Nbr_sections)
    Start_Points_Array = np.zeros((2,Nbr_sections))
    End_Points_Array = np.zeros((2,Nbr_sections))
    Trajectory_Points_Array = np.zeros((2,Nbr_sections*Nbr_Targets)) #TODO: modify columns number: DONE
    StartCommuting_point=np.zeros((2,Nbr_sections))
    EndCommuting_point=np.zeros((2,Nbr_sections))

    Y_start = start[1] + int(Robot_Length/2)
    Y_end   = start[1] + int(Panel_Length - (Robot_Length/2))
    
    #create an array of start points (Xi, Ymax) BOTTOM POINTS
    for i in range (Nbr_sections):
        Start_Points_Array[0][i]= start [0] + (Robot_Width*i) + (Robot_Width/2)
        Start_Points_Array[1][i]= Y_start

    #create an array of end points (Xi, Ymax)   UPPER POINTS
    for i in range (Nbr_sections):
        End_Points_Array[0][i]= start [0] + (Robot_Width*i) + (Robot_Width/2)
        End_Points_Array[1][i]= Y_end

    for i in range (Nbr_sections):
        Parity = divmod(i,2)
        if (Parity[1]==0): #even 
            #start
            Trajectory_Points_Array[0][i*Nbr_Targets]= Start_Points_Array[0][i] 
            Trajectory_Points_Array[1][i*Nbr_Targets]= Start_Points_Array[1][i] 
            #checkpints
            Interm_X, Interm_Y = IntermPoints_Generator_even(Start_Points_Array[0][i], Nbr_Checkpoints,
                                                        Interpolation_Step, Y_start)
            
            Trajectory_Points_Array[0][(i*Nbr_Targets)+1: (i*Nbr_Targets)+Nbr_Checkpoints+1]= Interm_X
            Trajectory_Points_Array[1][(i*Nbr_Targets)+1: (i*Nbr_Targets)+Nbr_Checkpoints+1]= Interm_Y
            #end 
            Trajectory_Points_Array[0][(i*Nbr_Targets)+Nbr_Checkpoints+1]= End_Points_Array[0][i]
            Trajectory_Points_Array[1][(i*Nbr_Targets)+Nbr_Checkpoints+1]= End_Points_Array[1][i]
            
            #commuting points
            StartCommuting_point[0][i]= End_Points_Array[0][i]
            StartCommuting_point[1][i]= End_Points_Array[1][i] - U1 #6 meters backward
            EndCommuting_point[0][i]  = End_Points_Array[0][i] + Robot_Width
            EndCommuting_point[1][i]  = End_Points_Array[1][i] - U2 
            Trajectory_Points_Array[0][(i*Nbr_Targets)+Nbr_Checkpoints+2] = StartCommuting_point[0][i]
            Trajectory_Points_Array[0][(i*Nbr_Targets)+Nbr_Checkpoints+3] = EndCommuting_point[0][i]
            Trajectory_Points_Array[1][(i*Nbr_Targets)+Nbr_Checkpoints+2] = StartCommuting_point[1][i]
            Trajectory_Points_Array[1][(i*Nbr_Targets)+Nbr_Checkpoints+3] = EndCommuting_point[1][i]

        else:  #odd
            #start
            Trajectory_Points_Array[0][i*Nbr_Targets]= End_Points_Array[0][i] #TODO  i is wrong as the step is not the same
            Trajectory_Points_Array[1][i*Nbr_Targets]= End_Points_Array[1][i] #solution: multiply i by the number of interm points: DONE
            #checkpints
            Interm_X, Interm_Y = IntermPoints_Generator_odd(Start_Points_Array[0][i], Nbr_Checkpoints,
                                                        Interpolation_Step,Y_end)
            Trajectory_Points_Array[0][(i*Nbr_Targets)+1: (i*Nbr_Targets)+Nbr_Checkpoints+1]= Interm_X
            Trajectory_Points_Array[1][(i*Nbr_Targets)+1: (i*Nbr_Targets)+Nbr_Checkpoints+1]= Interm_Y
            #end 
            Trajectory_Points_Array[0][(i*Nbr_Targets)+Nbr_Checkpoints+1]= Start_Points_Array[0][i]
            Trajectory_Points_Array[1][(i*Nbr_Targets)+Nbr_Checkpoints+1]= Start_Points_Array[1][i]
            
            #commuting points
            StartCommuting_point[0][i]= Start_Points_Array[0][i]
            StartCommuting_point[1][i]= Start_Points_Array[1][i] + U1 #6 meters forward
            EndCommuting_point[0][i]  = Start_Points_Array[0][i] + Robot_Width
            EndCommuting_point[1][i]  = Start_Points_Array[1][i] + U2
            Trajectory_Points_Array[0][(i*Nbr_Targets)+Nbr_Checkpoints+2] = StartCommuting_point[0][i]
            Trajectory_Points_Array[0][(i*Nbr_Targets)+Nbr_Checkpoints+3] = EndCommuting_point[0][i]
            Trajectory_Points_Array[1][(i*Nbr_Targets)+Nbr_Checkpoints+2] = StartCommuting_point[1][i]
            Trajectory_Points_Array[1][(i*Nbr_Targets)+Nbr_Checkpoints+3] = EndCommuting_point[1][i]

    Trajectory_Points_Array = np.delete(Trajectory_Points_Array, [Nbr_sections*Nbr_Targets -1, Nbr_sections*Nbr_Targets -2], axis=1)
    return Trajectory_Points_Array 
    
def IntermPoints_Generator_even(X,Nbr_Checkpoints, Interpolation_Step,Y_start ): 
    Interm_Points_Array = [[0 for x in range(Nbr_Checkpoints)] for y in range(2)]    
    for i in range (Nbr_Checkpoints):
        Interm_Points_Array[0][i] = X
        Interm_Points_Array[1][i] = Interpolation_Step * (i+1) + Y_start
    return  Interm_Points_Array[0],  Interm_Points_Array[1]

def IntermPoints_Generator_odd(X,Nbr_Checkpoints, Interpolation_Step, Y_end): #nbr of checkpoints 
    Interm_Points_Array = [[0 for x in range(Nbr_Checkpoints)] for y in range(2)]    
    for i in  range (Nbr_Checkpoints):
        Interm_Points_Array[0][i] = X
        Interm_Points_Array[1][i] = Y_end  - (Interpolation_Step * (i+1))
    return  Interm_Points_Array[0],  Interm_Points_Array[1] 

def main():

    global Start_Points_Array 
    global End_Points_Array 
    global Trajectory_Points_Array 
    global StartCommuting_point
    global EndCommuting_point
    global Panel_Length
    global Panel_Width
    global Y_start
    global Y_end   
    global sections
    global Nbr_sections
    global Nbr_Checkpoints
    global Nbr_Targets

    Robot_Width = 650
    #hardcoded solar panel dimensions 35000*100000 (35m*40m):
    borders = np.array([[Robot_Width/2,Robot_Width/2, 35000-Robot_Width/2, 35000-Robot_Width/2],
                        [Robot_Length/2,40000-Robot_Length/2,40000-Robot_Length/2,Robot_Length/2 ]])
    Panel_Length = int(borders[1][1]-borders[1][0] + 2*(Robot_Length/2)) #y1 - y0  TOTAL
    Panel_Width  = int(borders[0][2]-borders[0][1] + 2*(Robot_Width/2)) #x2 - x1  TOTAL
    
    Y_start = int(Robot_Length/2)
    Y_end   = int(Panel_Length - (Robot_Length/2))

    Checkpoints = divmod(borders[1][1]-borders[1][0], Interpolation_Step )
    Nbr_Checkpoints = int(Checkpoints[0])
    if (Checkpoints[1]==0):
        Nbr_Targets = Nbr_Checkpoints + 3
    else: 
        Nbr_Targets = Nbr_Checkpoints + 4

    Robot_Width_Float = np.ceil(Panel_Width/Robot_Width)
    Robot_Width = int(Panel_Width/Robot_Width_Float)
    Nbr_sections = int(Panel_Width/Robot_Width)

    print('Nbr_sections= ', Nbr_sections)
    Start_Points_Array = np.zeros((2,Nbr_sections))
    End_Points_Array = np.zeros((2,Nbr_sections))
    Trajectory_Points_Array = np.zeros((2,Nbr_sections*Nbr_Targets)) #TODO: modify columns number: DONE
    StartCommuting_point=np.zeros((2,Nbr_sections))
    EndCommuting_point=np.zeros((2,Nbr_sections))

    Trajectory = Algorithm1( [0,0] ,Robot_Width, Robot_Length, Panel_Width, Panel_Length,
                      Interpolation_Step, U1, U2)
    print (Trajectory)
   
    plt.rcParams["figure.autolayout"] = True
    plt.plot(Trajectory[0], Trajectory[1])
    plt.scatter(Trajectory[0], Trajectory[1],color='green',s=20)
    plt.plot([0,0,Panel_Width,Panel_Width,0],[0,Panel_Length,Panel_Length,0,0],linestyle='--',color=(1,0,0),linewidth=2)
    #plt.axis('equal')
    plt.grid('both')
    plt.show()

if __name__ == '__main__':
    
    main()



