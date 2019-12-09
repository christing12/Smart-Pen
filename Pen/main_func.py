import pygame
from pygame.locals import *
import os
import numpy

from readIMU import GetPos
import serial

white = (255,255,255)
black = (0,0,0)
orange = (255,165,0)

def test():
    pygame.init()
    screen = pygame.display.set_mode((480, 340))
    while 1:
        pygame.event.get()
        screen.fill(white)
        pygame.display.flip()
def main():
    ser = serial.Serial('/dev/cu.usbmodem14401', 115200, timeout=1)
    pygame.init()
    screen = pygame.display.set_mode((320,240))
    while 1:
        line = ser.readline()
        if b"DMP Ready!" in line:
            break
    while 1:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            pygame.quit()
            break
        screen.fill(white)
        GetPos()

        # points = GetPos()
        # points = numpy.array(points)
        # points = points[:, [0, 1]]
        #
        # ranges = numpy.ptp(points, axis=0)
        # x_scale = 230/ranges[0]
        # y_scale = 180/ranges[1]
        #
        # scale = min(x_scale, y_scale)
        # points[:,0] = points[:, 0] * scale
        # points[:,1] = points[:, 1] * scale
        # means = numpy.mean(points, axis = 0)
        #
        # points[:, 0] = points[:, 0] + (160-means[0])
        # points[:, 1] = points[:, 1] + (120-means[1])
        # ranges = numpy.ptp(points, axis=0)
        # print(ranges)
        #
        # screen.fill((0, 0, 0))
        # pygame.draw.lines(screen, (255, 255, 255), False, points, 2)
        # pygame.display.flip()

if __name__ == '__main__': main()
