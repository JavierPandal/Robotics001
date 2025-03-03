import roboticstoolbox as rtb
import numpy as np

robot = rtb.models.DH.Puma560()

q= [0, np.deg2rad(30), -np.deg2rad(160), 0, 0, 0]

robot.teach(q)