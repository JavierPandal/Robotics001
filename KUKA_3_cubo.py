import numpy as np
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
from spatialmath import SE3
from codigoVerTrayect import plot_robot_trajectory

# Definir el robot KUKA KR 4 R600 usando parámetros DH
#toolbox
robot = rtb.DHRobot(
    [
        rtb.RevoluteDH(d=0.45, a=0.15, alpha=np.deg2rad(-90), offset=0, qlim=[np.deg2rad(-170), np.deg2rad(170)]),
        rtb.RevoluteDH(d=0, a=0.83, alpha=np.deg2rad(0), offset=np.deg2rad(-90), qlim=[np.deg2rad(-185), np.deg2rad(65)]),
        rtb.RevoluteDH(d=0, a=0, alpha=np.deg2rad(-90), offset=0, qlim=[np.deg2rad(-137), np.deg2rad(163)]),
        rtb.RevoluteDH(d=0.66, a=0, alpha=np.deg2rad(90), offset=0, qlim=[np.deg2rad(-185), np.deg2rad(185)]),
        rtb.RevoluteDH(d=0, a=0, alpha=np.deg2rad(-90), offset=0, qlim=[np.deg2rad(-120), np.deg2rad(120)]),
        rtb.RevoluteDH(d=0.08, a=0, alpha=np.deg2rad(0), offset=0, qlim=[np.deg2rad(-350), np.deg2rad(350)])
    ],
    name="Kuka KR8 R1620", base=SE3(0, 0, 0)
)
#Ponemos el TCP alineado con el brazo
robot.tool = SE3.OA([0, 1, 0], [0,0,1]) #Right, elbow up
robot.configurations_str('lu') #Valores en 0
robot.qz = [0, 0, 0, 0, 0, 0] #Valores en 0
robot.teach(robot.qz)
#Verificar que el robot quedo igual en DH y en Toolbox
print(robot)
#Graficamos

#Puntos en el espacio x, y, z
T = np.array([
    [-0.08, 0.08, 0],            #A
    [-0.08, 0.08, 0 +0.16],       #B
    [-0.08, -0.08, 0 + 0.16],     #C
    [-0.08, -0.08, 0],           #D
    [-0.08, 0.08, 0],            #A
    [0.08, 0.08, 0],            #E
    [0.08, 0.08, 0],            #F
    [0.08, 0.08, 0 +0.16],       #G
    [0.08, -0.08, 0 + 0.16],     #H
    [0.08, -0.08, 0],           #I
    [0.08, 0.08, 0],            #J
    [0.08, -0.08, 0],           #K
    [-0.08, -0.08, 0],            #L
    [-0.08, -0.08, 0 + 0.16],      #M
    [0.08, -0.08, 0 + 0.16],       #N
    [0.08, 0.08, 0 + 0.16],        #O  
    [0.08, 0.08, 0 + 0.16],        #P   
    [-0.08, 0.08, 0 + 0.16],       #Q
    [-0.08, 0.08, 0],            #A
])

qn = [0,np.deg2rad(-98), np.deg2rad(65), 0, np.deg2rad(-75), 0] #Valores en 0
T_t = SE3.Trans(-0.2,0,1.0) * SE3.Trans(T)
qc= robot.ikine_LM(SE3(T_t),q0=robot.qz,tol=0.0025, joint_limits=True)
qc_array = np.array(qc.q)   

#Dibujo de linea
p_lim=[-1, 1, -1, 1, -0.25, 1.5]
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

T_tool = SE3.Trans(-0.2,0,1.1) * SE3.Trans(xyz_traj) #Right, elbow up

sol = robot.ikine_LM(T_tool,q0=robot.qz)
print(sol.success)

# Dibujo de linea
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