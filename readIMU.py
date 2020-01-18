from scipy import signal
import numpy as np
import matplotlib.pyplot as plt
from quaternion import *
from filter import AHRS

import matplotlib
import math
import csv

import logging
import sys
import time
import serial

ser = serial.Serial('/dev/cu.usbmodem14101', 115200, timeout=1)
def GetPos():

    t = 0;
    first_data = True

    while True:
        acc_t, quat_t = read_data()

        if not first_data:
            Acc = np.concatenate((Acc, acc_t), axis = 0)
            Quat = np.concatenate((Quat, quat_t), axis = 0)
        else:
            Acc = acc_t
            Quat = quat_t
            first_data = False

        t = t + 1
        time.sleep(1./100)

        if t > 300:
            print('end getting data')
            break

    plt.rcParams['lines.linewidth'] = 0.5
    samplePeriod = 1./100

    accX = Acc[:, 0]
    accY = Acc[:, 1]
    accZ = Acc[:, 2]

    t = len(accX)
    runtime = np.array([i * samplePeriod for i in range(t)])
    acc_mag = np.sqrt(accX**2 + accY**2 + accZ**2)

    #butterworth
    filtCutOff = 0.001
    b, a = signal.butter(1, (2*filtCutOff) / (1 / samplePeriod), 'high')
    acc_magFilt = signal.filtfilt(b, a, acc_mag)
    acc_magFilt = np.absolute(acc_magFilt)

    filtCutOff = 5.
    b, a = signal.butter(1, (2* filtCutOff) / (1 / samplePeriod), 'low')
    acc_magFilt = signal.filtfilt(b, a, acc_magFilt)

    accX_filt = signal.filtfilt(b, a, accX)
    accY_filt = signal.filtfilt(b, a, accY)
    accZ_filt = signal.filtfilt(b, a, accZ)

    accX_drift = np.mean(accX)
    accY_drift = np.mean(accY)
    accZ_drift = np.mean(accZ)

    accX_fix_drift = accX_filt - accX_drift
    accY_fix_drift = accY_filt - accY_drift
    accZ_fix_drift = accZ_filt - accZ_drift


    stationary = [a < 0.07 for a in acc_magFilt]

    l = len(accX)
    acc_fix_drift = np.concatenate((np.transpose([accX_fix_drift]), np.transpose([accY_fix_drift]), np.transpose([accZ_fix_drift])), axis=1)
    for i in range(l):
        acc_d_t = quaternRot(acc_fix_drift[i,:], quaternInv(Quat[i,:]))
        if i == 0:
            acc_d = acc_d_t
        else:
            acc_d = np.concatenate((acc_d, acc_d_t), axis = 0)

    vel = np.zeros(acc_d.shape)
    t_total = len(vel) - 2
    for a in range(t_total):
        i = a + 1
        vel[[i], :] = vel[[i-1], :] + acc_d[[i], :] * samplePeriod
        if stationary[i]:
            vel[[i], :] = [[0, 0, 0]]

    velDrift = np.zeros(vel.shape)
    stationary_temp = [0] + np.diff(1 * np.array(stationary))
    stationaryStart = [i for i,x in enumerate(stationary_temp) if x == -1]
    stationaryEnd = [i for i,x in enumerate(stationary_temp) if x == 1]

    if len(stationaryEnd) > len(stationaryStart):
        if len(stationaryEnd) > 1:
            stationaryEnd = stationaryEnd[1:]
        else:
            stationaryEnd = []
    if len(stationaryStart) > len(stationaryEnd):
        stationaryEnd = stationaryEnd.append(t_total)

    if stationaryStart == []:
        stationaryStart = [10]
    if stationaryEnd == [] or stationaryEnd == None:
        stationaryEnd = [t_total]


    for i in range(len(stationaryEnd)):
        driftRate = vel[stationaryEnd[i] - 1,:] / (stationaryEnd[i] - stationaryStart[i])
        enum = range(0, (stationaryEnd[i] - stationaryStart[i]))
        drift = np.concatenate((np.transpose([enum]) * driftRate[0],
                                np.transpose([enum]) * driftRate[1],
                                np.transpose([enum]) * driftRate[2]), axis=1)
        velDrift[stationaryStart[i]: stationaryEnd[i], :] = drift

    vel = vel-velDrift

    pos = np.zeros(vel.shape)
    for a in range(t_total):
        i = a + 1
        pos[[i], :] = pos[[i], :] + vel[[i], :] * samplePeriod
    l = len(pos[:, 0])
    pos = pos[0:l-1,:]


    np.savetxt('position_data.txt', pos, delimiter=',')
    return pos

def read_data():
    ser.write(b".")
    line = ser.readline()
    data = line.split(b", ")
    if len(data) == 7:
        ax = float(data[0])
        ay = float(data[1])
        az = float(data[2])
        qw = float(data[3])
        qx = float(data[4])
        qy = float(data[5])
        qz = float(data[6])
    acc_t = np.array([[ax, ay, az]])
    quat_t = np.array([[qw, qx, qy, qz]])
    return acc_t, quat_t