import numpy as np
# using sympy because it can correctly work with values of trigonometric functions
# from box, when numpy gives us only closer number, so here used the same function like in numpy
import sympy as sp
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D


def x_rot(q):
    return np.array([[1, 0, 0, 0],
                     [0, sp.N(sp.cos(q)), sp.N(sp.sin(q)), 0],
                     [0, -sp.N(sp.sin(q)), sp.N(sp.cos(q)), 0],
                     [0, 0, 0, 1]])


def y_rot(q):
    return np.array([[sp.N(sp.cos(q)), 0, sp.N(sp.sin(q)), 0],
                     [0, 1, 0, 0],
                     [-sp.N(sp.sin(q)), 0, sp.N(sp.cos(q)), 0],
                     [0, 0, 0, 1]])


def z_rot(q):
    return np.array([[sp.N(sp.cos(q)), sp.N(sp.sin(q)), 0, 0],
                     [-sp.N(sp.sin(q)), sp.N(sp.cos(q)), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])


def x_trans(l):
    t = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    t[0][3] = l
    return t


def y_trans(l):
    t = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    t[1][3] = l
    return t


def z_trans(l):
    t = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    t[2][3] = l
    return t


def forward_kinematics(q):
    t = z_trans(q[1][0]).dot(z_rot(q[0][0])).dot(y_trans(q[1][1])).dot(x_rot(q[0][1])).dot(z_trans(q[1][2])).dot(
        x_rot(q[0][2])).dot(z_trans(q[1][3])).dot(
        y_trans(q[1][4])).dot(y_rot(q[0][3])).dot(x_rot(q[0][4])).dot(y_trans(q[1][5])).dot(y_rot(q[0][5]))
    return t


def inverse_kinematics(t, v):
    a1 = sp.atan2(t[0][3], -t[1][3]) + sp.atan2(sp.sqrt(t[0][3] ** 2 + t[1][3] ** 2), v[1][5])
    a2 = sp.atan2(sp.cos(a1) * t[0][3] + sp.sin(a1) * t[1][3], -t[2][3] + v[1][0])
    a3 = sp.atan2(sp.sin(a2) * (a1 * t[0][3] + a1 * t[1][3]), sp.cos(a2) * (t[2][3] - v[1][0]))
    a4 = sp.atan2((t[0][2] * sp.sin(a1) - t[1][2] * sp.cos(a1)) / sp.sin(a3), - (
            t[2][2] * sp.sin(a2) + t[0][2] * sp.cos(a1) * sp.cos(a2) + t[1][2] * sp.cos(a2) * sp.sin(a1)) / sp.sin(
        a3))
    a5 = sp.atan2(sp.sqrt(
        1 - (t[2][2] * sp.cos(a2) - t[0][2] * sp.cos(a1) * sp.sin(a2) - t[1][2] * sp.sin(a1) * sp.sin(a2)) ** 2),
        t[2][2] * sp.cos(a2) - t[0][2] * sp.cos(a1) * sp.sin(a2) - t[1][1] * sp.sin(a1) * sp.sin(a2))
    a6 = sp.atan2(sp.sqrt(t[0][3] ** 2 + t[1][3] ** 2 + t[2][3] ** 2), t[2][3])
    return a1, a2, a3, a4, a5, a6


def calculate_rotation_matrix(alpha, beta, theta):
    return x_rot(alpha).dot(y_rot(beta)).dot(x_rot(theta))


def rot2eul(rotation_matrix):
    # print(rotation_matrix)
    a = sp.atan2(rotation_matrix[1][0], rotation_matrix[2][0])
    b = sp.atan2(sp.sqrt(rotation_matrix[1][0] ** 2 + rotation_matrix[2][0] ** 2), rotation_matrix[0][0])
    c = sp.atan2(rotation_matrix[0][1], rotation_matrix[0][2])
    return a, b, c


if __name__ == '__main__':
    l1 = 670
    l2 = 312
    l3 = 1075
    l4 = 225
    l5 = 1280
    l6 = 215
    # ---------
    q1 = 0
    q2 = 0
    q3 = 0
    q4 = 0
    q5 = 0
    q6 = 0
    # configuration vector for zero position
    vector = [[q1, q2, q3, q4, q5, q6],
              [l1, l2, l3, l4, l5, l6]]
    # --------------------------------------
    # print euler angels in radians from rotation matrix
    rot_matrix = calculate_rotation_matrix(sp.pi / 2, sp.pi / 2, -sp.pi / 2)
    print(rot2eul(rot_matrix))
    # print homogeneous transformation matrix
    T = forward_kinematics(vector)
    print(T)
    print("Angle: {0}".format(sum((vector[0][:]))))
    # print inverse kinematics
    print(inverse_kinematics(T, vector))

    """ Visualization on develop process
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    ax.plot3D(T[0:3][3])
    plt.show()"""
