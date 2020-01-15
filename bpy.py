import bpy
import math
import os
import array
import numpy as np
import datetime

# angle value
angle = [0.0]*6

# location and attitude
r = [0.0]*6

# transformation matrix
T = [np.identity(4)]*6
U = [np.identity(4)]*6
D = [np.identity(4)]*6

arms = [bpy.data.objects['Arm1'],bpy.data.objects['Arm2'],bpy.data.objects['Arm3'],bpy.data.objects['Arm4'],bpy.data.objects['Arm5'],bpy.data.objects['Arm6']]

def cal_tm(angle):
    # forward kinematics
    # Tij : Transformation matrix from i to j
    # Dij : partial Derivative of Tij with respect to theta_j
    T01 = np.array([[np.cos(angle[0]), -np.sin(angle[0]), 0 , 0],[np.sin(angle[0]), np.cos(angle[0]) , 0, 0],[0,0,1,1.462], [0,0,0,1]])
    D01 = np.array([[-np.sin(angle[0]), -np.cos(angle[0]), 0 , 0],[np.cos(angle[0]), -np.sin(angle[0]) , 0, 0],[0,0,0,0], [0,0,0,0]])

    T12 = np.array([[np.cos(angle[1]), -np.sin(angle[1]), 0 , 0],[0,0,1,-1.46],[-np.sin(angle[1]), -np.cos(angle[1]),0,0], [0,0,0,1]])
    D12 = np.array([[-np.sin(angle[1]), -np.cos(angle[1]), 0 , 0],[0,0,0,0],[-np.cos(angle[1]), np.sin(angle[1]),0,0], [0,0,0,0]])

    T23 = np.array([[np.cos(angle[2]), -np.sin(angle[2]), 0, 3.29], [np.sin(angle[2]), np.cos(angle[2]), 0, 0], [0, 0, 1, 1.297], [0, 0, 0, 1]])
    D23 = np.array([[-np.sin(angle[2]), -np.cos(angle[2]), 0, 0], [np.cos(angle[2]), -np.sin(angle[2]), 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])

    T34 = np.array([[np.cos(angle[3]), -np.sin(angle[3]), 0, 3.115], [np.sin(angle[3]), np.cos(angle[3]), 0, 0], [0, 0, 1, -1.06], [0, 0, 0, 1]])
    D34 = np.array([[-np.sin(angle[3]), -np.cos(angle[3]), 0, 0], [np.cos(angle[3]), -np.sin(angle[3]), 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])

    T45 = np.array([[np.cos(angle[4]), -np.sin(angle[4]), 0, 0], [0, 0, 1, -1.06], [np.sin(angle[4]), np.cos(angle[4]), 0, 0], [0, 0, 0, 1]])
    D45 = np.array([[-np.sin(angle[4]), -np.cos(angle[4]), 0, 0], [0, 0, 0, 0], [np.cos(angle[4]), -np.sin(angle[4]), 0, 0], [0, 0, 0, 0]])

    T56 = np.array([[np.cos(angle[5]), -np.sin(angle[5]), 0, 0], [0, 0, 1, -2.9815], [-np.sin(angle[5]), -np.cos(angle[5]), 0, 0], [0, 0, 0, 1]])
    D56 = np.array([[-np.sin(angle[5]), -np.cos(angle[5]), 0, 0], [0, 0, 0, 0], [-np.cos(angle[5]), np.sin(angle[5]), 0, 0], [0, 0, 0, 0]])

    T = [T01,T12,T23,T34,T45,T56]
    D = [D01,D12,D23,D34,D45,D56]

#def cal_1to6_tm():
    U[0] = T[0]
    U[1] = np.dot(T[0],T[1])
    U[2] = np.dot(U[1],T[2])
    U[3] = np.dot(U[2],T[3])
    U[4] = np.dot(U[3],T[4])
    U[5] = np.dot(U[4],T[5])

#def cal_location_n_attitude():
    r = [U[5][0][3],U[5][1][3],U[5][2][3], math.atan2(math.sqrt(math.pow(U[5][0][2],2)+math.pow(U[5][1][2],2)), U[5][2][2]), math.atan2(U[5][1][2], U[5][0][2]), math.atan2(U[5][2][1], -U[5][2][0])]
    return r

# rendering a frame
def render():
    bpy.ops.render.render()
    bpy.data.images['Render Result'].save_render(filepath = '/Users/nemototatsuaki/NTUproject/NTU2019Summer/image.png')
    
# initialize angle
def init_angle(obj):
    for i in range(3):
        obj.rotation_euler[i] = 0.0 

def init_all_angle():
    # initialize all angles
    for i in range(6):
        init_angle(arms[i])
        
def get_position(row):
    # get position data of the raw you want
    with open('/Users/nemototatsuaki/NTUproject/NTU2019Summer/output.txt', mode='rt', encoding='utf-8') as f:
        positions = f.readlines()
    pos_str = positions[row].split()
    pos = [float(n) for n in pos_str]
    return pos
    
# input angle data from "angle.txt"
def realtime():
    angles = []
    with open('/Users/nemototatsuaki/NTUproject/NTU2019Summer/angle.txt', mode='rt', encoding='utf-8') as f:
        angles = f.readlines()
    for i in range(len(angles)):
        temp_str = angles[i].split()
        #del temp_str[:2]
        temp = [float(n) for n in temp_str]
        for j in range(6):
            temp[j] = temp[j]*math.pi/180
        for j in range(6):
            if(j == 0 or j==4):            
                arms[j].rotation_euler[2] = temp[j]
            else:
                arms[j].rotation_euler[1] = temp[j]
            arms[j].keyframe_insert(data_path="rotation_euler", frame = i)
        now = datetime.datetime.now()
        angle[0] = temp[0]
        angle[1] = temp[1] - math.pi/2
        angle[2] = temp[2]
        angle[3] = temp[3] + math.pi/2
        angle[4] = temp[4]
        angle[5] = temp[5]
        r = cal_tm(angle)
        #cal_1to6_tm()
        #cal_location_n_attitude()
        with open('/Users/nemototatsuaki/NTUproject/NTU2019Summer/output.txt', mode='a') as f:
            f.write(str(r[0])+" "+str(r[1])+" "+str(r[2])+" "+str(r[3])+" "+str(r[4])+" "+str(r[5])+"\n")

init_all_angle()
realtime()
# render()




