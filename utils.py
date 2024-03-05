import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
# from numpy.linalg import pinv
from geopy import distance
from tqdm import tqdm
import math as mt

# x is north and y is east
def coords_to_Rnlative_meters(lat0, lng0, lat1, lng1, ellipsoid="WGS-84"):
    ya = distance.geodesic((lat0, lng0), (lat1, lng0), ellipsoid=ellipsoid).meters
    xa = distance.geodesic((lat1, lng0), (lat1, lng1), ellipsoid=ellipsoid).meters
    if lat1<lat0:
        ya=-ya
    if lng1<lng0:
        xa=-xa
    return xa, ya


def meter_to_coord(p_0,l_pos):
    l_pos2=np.zeros_like(l_pos)
    l_pos2[0] = p_0
    for i in range(1,len(l_pos)):
        l_pos2[i,0]=l_pos2[0,0]+(180/np.pi)*(l_pos[i,0]/6378137)/np.cos(l_pos2[0,1])
        l_pos2[i,1]=l_pos2[0,1]+(180/np.pi)*(l_pos[i,1]/6378137)
        l_pos2[i,2]=l_pos2[0,2]+l_pos[i,2]
    return l_pos2

def coord_to_meter(lat,long,p_0=None):
    pos_x=np.zeros_like(lat)
    pos_y=np.zeros_like(long)
    if p_0 is None:
        p_0 = np.array([lat[0],long[0]])

    for i in tqdm(range(len(lat))):
        pos_x[i],pos_y[i] = coords_to_Rnlative_meters(p_0[0], p_0[1], lat[i], long[i], ellipsoid="WGS-84")

    return pos_x,pos_y


def gauss_newton(f,J,x0,maxite=1000,tol=1):

    x = x0.copy()
    fx = f(x)
    ev = np.ones(len(x))*100

    for i in tqdm(range(maxite)):
        if ev[ev > tol].size == 0:
            break

        fx = f(x)
        Jx = J(x)
        #ev = -pinv(Jx)@fx
        ev = -np.linalg.inv(Jx.T@Jx)@(Jx.T)*fx
        x += ev


    return x


def moving_average(x, w):
    x[w-1:,0] =  np.convolve(x[:,0], np.ones(w), "valid") / w
    x[w-1:,1] =  np.convolve(x[:,1], np.ones(w), "valid") / w
    x[w-1:,2] =  np.convolve(x[:,2], np.ones(w), "valid") / w
    return x

def compute_RmRn(Lat):
    R = 6378137.0
    f =  1 / 298.257223563
    e = mt.sqrt(f * (2 - f))
    # e = 0.08181919
    Rm = R*(1-e**2)/(1-e**2*mt.sin(Lat)**2)**(3/2)
    Rn = R/mt.sqrt(1-e**2*(mt.sin(Lat)**2))
    return Rm,Rn


def get_wen(Lat,v,h):

    Rm,Rn = compute_RmRn(Lat)
    wen = np.zeros(3)

    wen[0] = -v[1]/(Rm+h)
    wen[1] = v[0]/(Rn+h)
    wen[2] = v[0]*mt.tan(Lat)/(Rn+h)

    return wen

def get_g(Lat,h):
    g0 = 9.780327
    g = g0*(1+5.2790414e-3*(mt.sin(Lat)**2)+2.32718e-5*(mt.sin(Lat)**4)+ 1.262e-7*(mt.sin(Lat)**6) + 7e-10*(mt.sin(Lat)**8))-3.085e-6*h
    return np.array([0,0,-g])

def get_localgravity(Lat,h):
    wie = get_wie(Lat)
    e = 0.08181919 
    Rm,Rn = compute_RmRn(Lat)
    r = np.array([0,0,(1-e**2)*Rn+h])
    g = get_g(Lat,h)
    return g-np.cross(wie,np.cross(wie,r))


def get_wie(Lat):

    w = 7.29211505e-5
    wie = np.zeros(3)

    wie[0] = 0
    wie[1] = w*mt.cos(Lat)
    wie[2] = w*mt.sin(Lat)
    
    return wie


def get_rotmatrix(angles,degrees=False):
    if degrees:
        angles=angles*np.pi/180

    x,y,z = angles[0].copy(),angles[1].copy(),angles[2].copy()
    Rx = np.array([
        [1,0,0],[0,mt.cos(x),-mt.sin(x)],[0,mt.sin(x),mt.cos(x)]
    ])
    Ry = np.array([
        [mt.cos(y),0,mt.sin(y)],[0,1,0],[-mt.sin(y),0,mt.cos(y)]
    ])
    Rz = np.array([
        [mt.cos(z),-mt.sin(z),0],[mt.sin(z),mt.cos(z),0],[0,0,1]
        ])
    return Rz@(Ry@Rx)