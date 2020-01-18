import pygame
from pygame.locals import *
import os
import numpy
import sys

from readIMU import GetPos
import serial

white = (255,255,255)
black = (0,0,0)
orange = (255,165,0)
size = (1280, 1024)

def test():
    pygame.display.init()
    screen = pygame.display.set_mode(size)
    screen.fill(orange)
    while True:
        for event in pygame.event.get():
            screen.fill(orange)

        pygame.display.update()

def main():
    ser = serial.Serial('/dev/cu.usbmodem14101', 115200, timeout=1)

    while 1:
        line = ser.readline()
        if b"DMP Ready!" in line:
            break
    pygame.init()
    screen = pygame.display.set_mode(size)
    screen.fill(black)
    while 1:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        points = GetPos()
        points = numpy.array(points)
        points = points[:, [0, 1]]

        ranges = numpy.ptp(points, axis=0)
        x_scale = 230/ranges[0]
        y_scale = 180/ranges[1]

        scale = min(x_scale, y_scale)
        points[:,0] = points[:, 0] * scale
        points[:,1] = points[:, 1] * scale
        means = numpy.mean(points, axis = 0)

        points[:, 0] = points[:, 0] + (160-means[0])
        points[:, 1] = points[:, 1] + (120-means[1])
        ranges = numpy.ptp(points, axis=0)
        screen.fill((0, 0, 0))
        pygame.draw.lines(screen, (255, 255, 255), False, points, 2)
        pygame.display.flip()

if __name__ == '__main__': main()
