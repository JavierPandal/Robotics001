import numpy as np
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
from spatialmath import SE3
from codigoVerTrayect import plot_robot_trajectory

# Definir el robot KUKA KR 4 R600 usando parámetros DH
aubo = rtb.DHRobot(
    [
        rtb.RevoluteDH(d=0.45, a=0.15, alpha=np.deg2rad(-90), offset=0, qlim=[np.deg2rad(-170), np.deg2rad(170)]),
        rtb.RevoluteDH(d=0, a=0.83, alpha=np.deg2rad(0), offset=np.deg2rad(-90), qlim=[np.deg2rad(-185), np.deg2rad(65)]),
        rtb.RevoluteDH(d=0, a=0, alpha=np.deg2rad(-90), offset=0, qlim=[np.deg2rad(-137), np.deg2rad(163)]),
        rtb.RevoluteDH(d=0.66, a=0, alpha=np.deg2rad(90), offset=0, qlim=[np.deg2rad(-185), np.deg2rad(185)]),
        rtb.RevoluteDH(d=0, a=0, alpha=np.deg2rad(-90), offset=0, qlim=[np.deg2rad(-120), np.deg2rad(120)]),
        rtb.RevoluteDH(d=0.08, a=0, alpha=np.deg2rad(0), offset=0, qlim=[np.deg2rad(-350), np.deg2rad(350)])
    ],
    name="Aubo i5"
)

# Ponemos el TCP alineado con el brazo
aubo.tool = SE3.OA([0, 1, 0], [0, 0, 1])  # Right, elbow up
aubo.qz = [0, 0, 0, 0, 0, 0]  # Posición de referencia en 0


# Matriz de posiciones articulares (en radianes)
T = np.array([
    #[Joint 1 (Brazo mas largo), Joint 2, Joint 3, Joint 4, Joint 5, Joint 6 (TCP rotativo)],           
  [np.deg2rad(-170), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0)],        # A 
    [np.deg2rad(150), np.deg2rad(-195), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0)],       # B 
    [np.deg2rad(170), np.deg2rad(-175), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0)],    # C 
    [np.deg2rad(0), np.deg2rad(65), np.deg2rad(160), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0)],    # D 
    [np.deg2rad(170), np.deg2rad(0), np.deg2rad(-135), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0)],    # E 
    [np.deg2rad(0), np.deg2rad(0), np.deg2rad(-135), np.deg2rad(185), np.deg2rad(0), np.deg2rad(0)],    # F 
    [np.deg2rad(170), np.deg2rad(0), np.deg2rad(0), np.deg2rad(-185), np.deg2rad(0), np.deg2rad(0)],    # G 
    [np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(-185), np.deg2rad(120), np.deg2rad(0)],    # H 
    [np.deg2rad(-170), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(-120), np.deg2rad(0)],      # I
])

# Generar la trayectoria articular con jtraj
q_traj = []
for i in range(len(T) - 1):
    traj = rtb.jtraj(T[i], T[i + 1], 10)
    q_traj.append(traj.q)
q_traj = np.vstack(q_traj)  # Convertimos la lista en un array

# Dibujar la trayectoria
p_lim = [-1, 1, -1, 1, -0.15, 1.5]
plot_robot_trajectory(
    robot=aubo,
    q_trajectory=q_traj,
    limits=p_lim,
    eeframe=True,
    jointaxes=False,
    shadow=True,
    drawing_mode='continuous',
    traj_color='r',
    drawing_color='b',
    dt=0.15,
    block=True
)