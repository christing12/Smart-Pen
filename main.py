

import numpy as np
import serial
import serial.tools.list_ports
import pygame
import sys
import matplotlib.pyplot as plt

class DataCollector:
    def __init__(self, portName, baudrate):
        self.portName = portName
        self.baudrate = baudrate
        print('Connecting to port... ')
        self.InitCommunication()

    def InitCommunication(self):
        self.ser = serial.Serial(self.portName, self.baudrate)
        print('Initialized connection... ')

    def GetLine(self):
        return self.ser.readline()

    def GetData(self):
        line = self.ser.readline()
        rawData = line.split(b", ")
        gx = 0
        gy = 0
        gz = 0
        if len(rawData) == 3:
            gx = float(rawData[0])
            gy = float(rawData[1])
            gz = float(rawData[2])
        gyro_vector = np.array([gx, gy, gz])
        return gyro_vector

def FindArduino():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if 'usbmodem' in p.device and 'Generic CDC' in p.description:
            print("FOUND ARDUINO");
            return p

def PlotData(data, timestamps, title = 'Acceleration Data', fileName = 'acceleration.png'):
    plt.plot(timestamps, data)
    plt.xlabel('time')
    plt.ylabel('acceleration')
    plt.title(title)
    plt.grid(True)
    plt.savefig(fileName)
    plt.show()

def main():
    port = FindArduino()
    baudrate = 115200
    dataCollector = None
    if not port:
        print('Error with finding Arduino port')
        return

    dataCollector = DataCollector(port.device, baudrate)

    while 1:
        line = dataCollector.GetLine()
        #print(line)
        if b"DMP ready!" in line:
            print("DONE")
            break

    numDataPoints = 500
    accelX = np.zeros(numDataPoints)
    accelY = np.zeros(numDataPoints)
    accelZ = np.zeros(numDataPoints)


    timestamps = np.zeros(numDataPoints)
    t = 0
    while t < numDataPoints:
        data = dataCollector.GetData()
        t += 1

    print("ACTUALLY READING DATA NOW")
    t = 0

    while t < numDataPoints:
        data = dataCollector.GetData()
        accelX[t] = data[0]
        accelY[t] = data[1]
        accelZ[t] = data[2]

        timestamps[t] = t
        t += 1

    dataFile = np.savetxt('data.txt', accelX, delimiter=", ")
    PlotData(accelX, timestamps, "X Acceleration", "accelx.png")
    PlotData(accelY, timestamps, "Y Acceleration", "accelY.png")
    PlotData(accelZ, timestamps, "Z Acceleration", "accelZ.png")

    # size = (1280, 1024)
    # white = (255, 255, 255)
    # black = (0, 0, 0)
    # orange = (255, 165, 0)
    # pygame.init()
    # screen = pygame.display.set_mode(size)
    # screen.fill(black)
    # while 1:
    #     for event in pygame.event.get():
    #         if event.type == pygame.QUIT:
    #             pygame.quit()
    #             sys.exit()
    #
    #     pygame.draw.line(screen, orange, (60, 80), (130, 100))
    #     pygame.display.flip()




if __name__ == '__main__': main()