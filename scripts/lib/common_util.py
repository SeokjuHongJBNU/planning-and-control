import cv2
import numpy as np
import math


def RotationMatrix(yaw, pitch, roll):
     
    R_x = np.array([[1,         0,              0,                0],
                    [0,         math.cos(roll), -math.sin(roll) , 0],
                    [0,         math.sin(roll), math.cos(roll)  , 0],
                    [0,         0,              0,               1],
                    ])
                     
    R_y = np.array([[math.cos(pitch),    0,      math.sin(pitch) , 0],
                    [0,                  1,      0               , 0],
                    [-math.sin(pitch),   0,      math.cos(pitch) , 0],
                    [0,         0,              0,               1],
                    ])
                 
    R_z = np.array([[math.cos(yaw),    -math.sin(yaw),    0,    0],
                    [math.sin(yaw),    math.cos(yaw),     0,    0],
                    [0,                0,                 1,    0],
                    [0,         0,              0,               1],
                    ])
                     
    R = np.matmul(R_x, np.matmul(R_y, R_z))
 
    return R


def TranslationMatrix(x, y, z):
     
    M = np.array([[1,         0,              0,               x],
                  [0,         1,              0,               y],
                  [0,         0,              1,               z],
                  [0,         0,              0,               1],
                  ])
    
    return M

def RTmatrix_2D(theta,tx,ty,scale):
     
    Rt = np.array([[math.cos(theta), -math.sin(theta),tx],
                  [math.sin(theta), math.cos(theta), ty],
                  [0,               0,               1]
                  ])
                     
    return Rt * scale

def Rmatrix_2D(theta):
     
    R = np.array([[np.cos(np.deg2rad(theta)), -np.sin(np.deg2rad(theta)),0],
                  [np.sin(np.deg2rad(theta)), np.cos(np.deg2rad(theta)), 0],
                  [0,               0,               1]
                  ])
                     
    return R

def Tmatrix_2D(tx,ty):
     
    t = np.array([[1, 0,tx],
                  [0, 1, ty],
                  [0, 0, 1]
                  ])
                     
    return t
