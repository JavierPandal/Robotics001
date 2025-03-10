import numpy as np
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
from spatialmath import SE3
from codigoVerTrayect import plot_robot_trajectory

# Definir el robot KUKA KR 4 R600 usando parámetros DH
robot = rtb.DHRobot(
    [
        rtb.RevoluteDH(d=0.0985, a=0, alpha=np.deg2rad(-90), offset=0, qlim=[np.deg2rad(-175), np.deg2rad(175)]),
        rtb.RevoluteDH(d=0.1405, a=0.408, alpha=np.deg2rad(-180), offset=np.deg2rad(-90), qlim=[np.deg2rad(-175), np.deg2rad(175)]),
        rtb.RevoluteDH(d=0.1215, a=0.376, alpha=np.deg2rad(180), offset=0, qlim=[np.deg2rad(-175), np.deg2rad(175)]),
        rtb.RevoluteDH(d=0.1025, a=0, alpha=np.deg2rad(90), offset=np.deg2rad(-90), qlim=[np.deg2rad(-175), np.deg2rad(175)]),
        rtb.RevoluteDH(d=0.1025, a=0, alpha=np.deg2rad(-90), offset=0, qlim=[np.deg2rad(-175), np.deg2rad(175)]),
        rtb.RevoluteDH(d=0.094, a=0, alpha=0, offset=0, qlim=[np.deg2rad(-175), np.deg2rad(175)])
    ],
    name="Aubo i5"
)
#Ponemos el TCP alineado con el brazo
robot.tool = SE3.OA([0, 1, 0], [0,0,1]) #Right, elbow up
robot.configurations_str('lu') #Valores en 0
robot.qz = [0, 0, 0, 0, 0, 0] #Valores en 0

#Puntos en el espacio
# Definir las coordenadas de los puntos
T = np.array([
    [-0.1, 0.1, 0],            #A
    [-0.1, 0.1, 0 +0.2],       #B
    [-0.1, -0.1, 0 + 0.2],     #C
    [-0.1, -0.1, 0],           #D
    [-0.1, 0.1, 0],            #A
    [0.1, 0.1, 0],            #E
    [0.1, 0.1, 0],            #F
    [0.1, 0.1, 0 +0.2],       #G
    [0.1, -0.1, 0 + 0.2],     #H
    [0.1, -0.1, 0],           #I
    [0.1, 0.1, 0],            #J
    [0.1, -0.1, 0],           #K
    [-0.1, -0.1, 0],            #L
    [-0.1, -0.1, 0 + 0.2],      #M
    [0.1, -0.1, 0 + 0.2],       #N
    [0.1, 0.1, 0 + 0.2],        #O  
    [0.1, 0.1, 0 + 0.2],        #P   
    [-0.1, 0.1, 0 + 0.2],       #Q
    [-0.1, 0.1, 0],            #A
])
qn = [0,np.deg2rad(-98), np.deg2rad(65), 0, np.deg2rad(-75), 0] #Valores en 0
T_t = SE3.Trans(-0.3,0,0.5) * SE3.Trans(T)
qc= robot.ikine_LM(SE3(T_t),q0=robot.qz,tol=0.0025, joint_limits=True)
qc_array = np.array(qc.q)   

#Dibujo de linea
p_lim=[-1, 1, -1, 1, -0.15, 1.5]
plot_robot_trajectory(
    robot=robot,
    q_trajectory=qc_array,
    limits=p_lim,
    eeframe=True,
    jointaxes=False,
    shadow=True,
    drawing_mode='continuous',  # o 'segments' si prefieres
    traj_color='r',             # Color de la trayectoria completa
    drawing_color='b',          # Color del trazo principal
    dt=1,
    block=True
)

via = np.empty((0, 3))
for punto in T:
    xyz = np.array(punto)
     #print(xyz)
    via = np.vstack((via, xyz)) #Agregamos los puntos al array
 
xyz_traj = rtb.mstraj(via, qdmax = (0.5, 0.5, 0.5), dt = 0.02, tacc=0.2).q 
 
fig = plt.figure()
ax= fig.add_subplot(111, projection='3d')
plt.plot(xyz_traj[:,0], xyz_traj[:,1], xyz_traj[:,2])
ax.scatter(xyz_traj[0,0], xyz_traj[0,1], xyz_traj[0,2], color='red', marker = '*') #Inicio
ax.scatter(xyz_traj[-1,0], xyz_traj[-1,1], xyz_traj[-1,2], color='green', marker = 'o') #Final  
plt.show()

T_tool = SE3.Trans(-0.3,0,0.5) * SE3.Trans(xyz_traj) #Right, elbow up

sol = robot.ikine_LM(T_tool,q0=robot.qz)
print(sol.success)

#Dibijo de linea
plot_robot_trajectory(
    robot=robot,
    q_trajectory=sol.q,
    limits=p_lim,
    eeframe=True,
    jointaxes=False,
    shadow=True,
    drawing_mode='continuous',  # o 'segments' si prefieres
    traj_color='r',             # Color de la trayectoria completa
    drawing_color='b',          # Color del trazo principal
    dt=0.05,
    block=True
)