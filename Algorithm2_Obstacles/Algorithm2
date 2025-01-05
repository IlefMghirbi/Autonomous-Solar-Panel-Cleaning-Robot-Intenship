#!/usr/bin/env python3
#Author: Ilef Mghirbi

##############################################

#         BRIEF: ** THIS ALGORITHM CREATES THE TRAJECTORY FOR THE ROBOT P-SOLAR
#                IN THE PRESENCE OF ALIGNED OBSTACLES HAVING THE SAME WIDTH 
#                (OBSTACLES EXAMPLE IS IN THE FILE obstacles.json OR obstacles_NOvlp.json )

#                ** THIS ALGORITHM USES ALGORITHM1.PY 
##############################################

import numpy as np 
from matplotlib import pyplot as plt
import json
from Algorithm1 import Algorithm1
###################     PARAMETERS INITIALIZATION   #########################
obstacles_list = []
cells_list = []
Robot_width = 650
Robot_length = 684
Panel_length = 40000
Panel_width = 35000
Interpolation_Step, U1, U2 = 2000, 6000, 3000
###################   CLASSES    ############################################
class Obstacle ():
    def __init__(self, id, x_coordinates, y_coordinates ):
        self.id = id
        self.x_coordinates = x_coordinates
        self.y_coordinates = y_coordinates
    def __eq__(self, other):
        return self.x_coordinates == other.x_coordinates
    
class Cell ():
    def __init__(self, x_coordinates, y_coordinates ):
        self.x_coordinates = x_coordinates
        self.y_coordinates = y_coordinates

def occurence (i,array):
    global occurence 
    for j in range (i+1,len(array)-1):
        if (array[i].x_coordinates[0] == array[j].x_coordinates[0]):
            
            occurence = Obstacle (array[i].id, array[i].x_coordinates, array[i].y_coordinates)
            occurence.append(array[j])
            
            for i in range(len(occurence)):
                for j in range(len(occurence)-1):
                    if occurence[j].y_coordinates[0] > occurence[j+1].y_coordinates[0] :
                        occurence[j], occurence[j+1] = occurence[j+1], occurence[j]

    return occurence

# DETERMINES IF 2 RECTANGLES OVERLAP: ARGUMENTS ARE OF ARRAY TYPES: [x,y]
# FALSE == NO OVERLAPPING // TRUE == CELL AND OBSTACLE ARE OVERLAPPING
# BOTTOM CORNER IS THE RIGHT BOTTOM RCORNER OF A RECTANGLE / TOP CORNER IS THE LEFT TOP CORNER
def overlap (TopCorner_Cell, TopCorner_Obs, BotCorner_Cell, BotCorner_Obs):

    if (TopCorner_Obs[0] < BotCorner_Cell[0] and  BotCorner_Obs[1]> BotCorner_Cell[1] and TopCorner_Obs[1]<TopCorner_Cell[1] ):
   
        return True
        
    else: return False

def Check_overlap (obstacles_list, cell, offset):
    for k in range ( offset + 1 , len(obstacles_list)):
        if (overlap([cell.x_coordinates[1],cell.y_coordinates[1]],
                    [ obstacles_list[k].x_coordinates[1], obstacles_list[k].y_coordinates[1]],
                    [cell.x_coordinates[3],cell.y_coordinates[3]],
                    [ obstacles_list[k].x_coordinates[3], obstacles_list[k].y_coordinates[3]]) == True):
            print("OBSTACLE: ", obstacles_list[k].x_coordinates[0], obstacles_list[k].y_coordinates[0] ,
                  "OVERLAPS CELL : ", cell.y_coordinates[0],cell.y_coordinates[1] )
        #except: print("FINISHED OBSTACLES")

def division (obstacles_list,panel_length, panel_width):

    global cells_list
    global obstacle_x # ARRAY OF ALL OBSTACLES WITHOUT RECURRENCE
    global obstacle_x_occurence # NUMBER OF OCCURENCES OF X

    Minimum_CellLength = 6684

    # hedhi kifelli zeyda ema khalliha
    compteur = []
    inc = 0
    for i in range (len(obstacle_x_occurence)):
        inc += obstacle_x_occurence[i]
        compteur.append(inc)

    start_x = 0
    start_y = 0
    offset = 0

    for j in range (len(obstacle_x_occurence)):
        if (obstacles_list[offset].x_coordinates[0] != start_x):
            if ((panel_length - start_y) > Minimum_CellLength):
                adjacent_cell = Cell ([start_x, start_x, obstacles_list[offset].x_coordinates[0],obstacles_list[offset].x_coordinates[0]],
                                       [start_y,panel_length,panel_length, start_y] ) 
                cells_list.append(adjacent_cell)
                Check_overlap (obstacles_list, adjacent_cell, offset)
        
        if (obstacles_list[offset].y_coordinates[0] != start_y):
            if ((obstacles_list[offset].y_coordinates[0] - start_y) > Minimum_CellLength):
                below_cell = Cell([obstacles_list[offset].x_coordinates[0],obstacles_list[offset].x_coordinates[0],
                                   obstacles_list[offset].x_coordinates[2],obstacles_list[offset].x_coordinates[2]], 
                                   [start_y,obstacles_list[offset].y_coordinates[0], obstacles_list[offset].y_coordinates[0],  start_y])
                cells_list.append(below_cell)
                Check_overlap (obstacles_list, below_cell, offset)

        if (obstacles_list[offset+ obstacle_x_occurence[j]-1].y_coordinates[1] != panel_length):
            if ((panel_length - (obstacles_list[offset+ obstacle_x_occurence[j]-1].y_coordinates[1])) > Minimum_CellLength):
                above_cell = Cell([obstacles_list[offset+ obstacle_x_occurence[j]-1].x_coordinates[0],obstacles_list[offset+ obstacle_x_occurence[j]-1].x_coordinates[0],
                                   obstacles_list[offset+ obstacle_x_occurence[j]-1].x_coordinates[2],obstacles_list[offset+ obstacle_x_occurence[j]-1].x_coordinates[2]],
                                  [obstacles_list[offset+ obstacle_x_occurence[j]-1].y_coordinates[1],panel_length,
                                   panel_length,obstacles_list[offset+ obstacle_x_occurence[j]-1].y_coordinates[1]])
                cells_list.append(above_cell)
                Check_overlap (obstacles_list, above_cell, offset)

        if (obstacle_x_occurence[j]>1):   
            for i in range(offset, offset + obstacle_x_occurence[j]-1):
                if ((obstacles_list[i+1].y_coordinates[0]- obstacles_list[i].y_coordinates[1]) > Minimum_CellLength):
                    inter_Cell = Cell ([obstacles_list[i].x_coordinates[1],obstacles_list[i].x_coordinates[1],
                                        obstacles_list[i].x_coordinates[2],obstacles_list[i].x_coordinates[2]],
                                       [obstacles_list[i].y_coordinates[1], obstacles_list[i+1].y_coordinates[0],
                                        obstacles_list[i+1].y_coordinates[0], obstacles_list[i].y_coordinates[1]])
                    cells_list.append(inter_Cell)
                    Check_overlap (obstacles_list, inter_Cell, offset)
                    
        start_x = obstacles_list[offset].x_coordinates[2]
        # last instruction in one j loop
        offset += obstacle_x_occurence[j] #Increments by the current x's number of occurences.

    if (start_x < panel_width):
        final_cell = Cell ([start_x, start_x, panel_width, panel_width],
                    [start_y,panel_length, panel_length, start_y])
        cells_list.append (final_cell)

    return cells_list
    
def main():
    global nbr_obstacles 
    global sorted_x
    global cells_list
    global obstacle_x
    global obstacle_x_occurence

    #  EXTRACTING THE OBSTACLES COORDINATES 
    with open("/home/ilef/PSolar_ws/src/scripts/src/obstacles_NOvLp.json") as file:
    #with open("/home/ilef/PSolar_ws/src/scripts/src/obstacles.json") as file:
        data = json.load(file)
        # print(data["Obstacles"][0]["x"][0])
        Nbr_obstacles = len(data["Obstacles"])
        #print([i["x"] for i in data["Obstacles"]])
        for i in data["Obstacles"]:
            obstacle = Obstacle(i["id"],i["x"],i["y"])
            obstacles_list.append(obstacle)
            plt.plot(i["x"]+[i["x"][0]],i["y"]+[i["y"][0]])
            plt.scatter(i["x"],i["y"])
        plt.scatter([0,0,Panel_width,Panel_width],[0,Panel_length,Panel_length,0])
        plt.plot([0,0,Panel_width,Panel_width,0],[0,Panel_length,Panel_length,0,0])
        plt.axis('equal')
    
    obstacle_x = []
    obstacle_x_occurence = []

    #  SORTING OBSTACLES BASED ON ASCENDANT X 
    for i in range(len(obstacles_list)):
        for j in range(len(obstacles_list)-1):
            if obstacles_list[j].x_coordinates[0] > obstacles_list[j+1].x_coordinates[0] :
                obstacles_list[j], obstacles_list[j+1] = obstacles_list[j+1], obstacles_list[j]

    compteur = 1  
    for i in range(len(obstacles_list)):
        new_occur = Obstacle (obstacles_list[i].id, obstacles_list[i].x_coordinates, obstacles_list[i].y_coordinates)
        if (i==0):
            obstacle_x= [new_occur]
            obstacle_x_occurence= [compteur]
        elif (new_occur == obstacle_x[len(obstacle_x)-1]):
            compteur += 1 
            obstacle_x_occurence[len(obstacle_x)-1]= compteur
        else: # NEW X
            obstacle_x.append(new_occur)
            compteur = 1
            obstacle_x_occurence.append(compteur)

    offset = 0
    for i in range (len(obstacle_x_occurence)):
        for j in range(offset, offset + obstacle_x_occurence[i]):
            for k in range (offset, offset + obstacle_x_occurence[i]-1):
                if obstacles_list[k].y_coordinates[0] > obstacles_list[k+1].y_coordinates[0] :
                        obstacles_list[k], obstacles_list[k+1] = obstacles_list[k+1], obstacles_list[k]
        offset += obstacle_x_occurence[i]
    
    #for i in range (len(obstacles_list)):
    #    print(obstacles_list[i].y_coordinates)

    cells_list = division (obstacles_list,Panel_length, Panel_width)

    trajectory_x = []
    trajectory_y = []
    #  APPLYING ALGORITHM1 ON EACH CELL 
    for i in range (len(cells_list)):
        plt.scatter(cells_list[i].x_coordinates,cells_list[i].y_coordinates,color='green',s=20)
        plt.plot(cells_list[i].x_coordinates, cells_list[i].y_coordinates)
        
        cell_width = cells_list[i].x_coordinates[3] - cells_list[i].x_coordinates[0]
        cell_length = cells_list[i].y_coordinates[1] - cells_list[i].y_coordinates [0]
        #print(cell_length, cell_width)
        Trajectory = Algorithm1( [cells_list[i].x_coordinates[0], cells_list[i].y_coordinates[0]],Robot_width, Robot_length, cell_width, cell_length,
                        Interpolation_Step, U1, U2)
        plt.plot(Trajectory[0], Trajectory[1])
        plt.scatter(Trajectory[0], Trajectory[1],color='green',s=20)

        trajectory_x = Trajectory[0].tolist()
        trajectory_y = Trajectory[1].tolist()
        coordinates = list(zip(trajectory_x, trajectory_y))
        file_path = "/home/ilef/PSolar_ws/src/scripts/src/cells_trajectory_targets.json"
        try: 
            with open(file_path, 'r') as file:
                existing_data = json.load(file)
        except:
            existing_data = []

        existing_data.extend(coordinates)

        # Write the updated data back to the JSON file
        with open(file_path, 'w') as file:
            json.dump(existing_data, file, indent=4)
            
    plt.show()

if __name__ == '__main__':
    
    main()
