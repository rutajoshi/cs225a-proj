import numpy as np
import matplotlib.pyplot as plt

l1 = 0.5
l2 = 0.4

t = range(0, 100)
positions = []
vertical_offset = 0.1

T_world_leg = np.array([1.25, 0.0, 0.15])
R_world_leg = np.eye(3)
T_world_robot = np.array([0.5, -0.4, 0.05])
R_world_robot = np.array([[0, -1, 0],
                          [1, 0, 0],
                          [0, 0, 1]])

for count in t:
    q1 = (45.0*np.pi/180)*np.sin(6*np.pi*count/100.0 - np.pi/2.0) - (15.0*np.pi/180)
    q2 = (-30.0*np.pi/180)*np.sin(6*np.pi*count/100.0 - np.pi/2.0) - (90.0*np.pi/180)

    a = -l1 * np.sin(q1) - l2 * np.sin(q1 + q2)
    b = 0.0
    c = l1 * np.cos(q1) + l2 * np.cos(q1 + q2) + vertical_offset
    X = np.array([a, b, c])

    # Transform X
    X -= T_world_leg
    X = np.matmul(R_world_leg, X)
    X += T_world_robot
    X = np.matmul(R_world_robot, X)

    print(X)
    print("\n")

    positions.append(X)

# print(positions)
positions = np.array(positions)

plt.title("positions after transformations")
plt.xlabel("time (seconds)")
plt.ylabel("axis value")
plt.plot(t, positions[:, 0])
plt.plot(t, positions[:, 1])
plt.plot(t, positions[:, 2])
plt.legend(['x', 'y', 'z'], loc='upper right')
plt.show()
