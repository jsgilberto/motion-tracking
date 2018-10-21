import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from sensor import Sensor
from madgwick import Madgwick
import numpy as np
import math

vertices = (
    (1, -1, -1),
    (1, 1, -1),
    (-1, 1, -1),
    (-1, -1, -1),
    (1, -1, 1),
    (1, 1, 1),
    (-1, -1, 1),
    (-1, 1, 1)
    )

edges = (
    (0,1),
    (0,3),
    (0,4),
    (2,1),
    (2,3),
    (2,7),
    (6,3),
    (6,4),
    (6,7),
    (5,1),
    (5,4),
    (5,7)
    )

def Cube():
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()

def main():
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

    gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)
    glTranslatef(0.0,0.0, -5)

    mpu9250 = Sensor('/dev/ttyACM0', 115200)
    m = Madgwick()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
        # sensor data
        # mpu9250.update_values()
        
        # using average values
        mpu9250.update_values_with_moving_average(15)
        accel = mpu9250._accel_avg
        gyro = mpu9250._gyro_avg
        mag = mpu9250._mag_avg
        # madgwick algorithm
        m.update(accel, gyro * np.pi / 180, mag)
        q = m._quaternion_avg
        q = np.squeeze(np.asarray(q))

        glMatrixMode(GL_MODELVIEW)
        glLoadMatrixf(Madgwick.quat_to_rotmatrix(q))
        glScalef(0.2, 0.2, 0.2)
        #glRotatef(q[0], q[1], q[2], q[3])
        #glRotatef(1, 3, 1, 1)
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        Cube()
        pygame.display.flip()
        pygame.time.wait(10)


main()