import numpy as np

# angle value
angle = [0.0]*6

# location and attitude
r = [0.0]*6

# transformation matrix
T = [np.identity(4)]*6
U = [np.identity(4)]*6
D = [np.identity(4)]*6



def cal_tm():
    # 順運動学 fk
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

    T56 = np.array([[np.cos(angle[5]), -np.sin(angle[5]), 0, 0], [0, 0, 1, -1.32], [-np.sin(angle[5]), -np.cos(angle[5]), 0, 0], [0, 0, 0, 1]])
    D56 = np.array([[-np.sin(angle[5]), -np.cos(angle[5]), 0, 0], [0, 0, 0, 0], [-np.cos(angle[5]), np.sin(angle[5]), 0, 0], [0, 0, 0, 0]])

    T = [T01,T12,T23,T34,T45,T56]
    D = [D01,D12,D23,D34,D45,D56]

    return 0

def cal_1to6_matrix():
    U[0] = T[0]
    U[1] = np.dot(T[0],T[1])
    U[2] = np.dot(U[1],T[2])
    U[3] = np.dot(U[2],T[3])
    U[4] = np.dot(U[3],T[4])
    U[5] = np.dot(U[4],T[5])
    return 0

def cal_location_n_attitude():
    r = [U[5][0][3],U[5][1][3],U[5][2][3], 0.0, 0.0, 0.0]


# angle_lで偏微分し、angle[]を代入
def dif_matrix_T(l):
    C = np.identity(3)
    for i in range(1,l):
        C = np.dot(T[i-1],T[i])
    C = np.dot(C, D[l-1])
    for j in range(l+1,6):
        C = np.dot(T[j-1],T[j])
    return C

print(r)

jacobian = np.identity(6)
for i in range(3):
    for j in range(6):
        pass

"""
print("T01")
print(T01)
print("T02")
print(T02)
print("T03")
print(T03)
print("T04")
print(T04)
print("T05")
print(T05)
print("T06")
print(T06)
"""