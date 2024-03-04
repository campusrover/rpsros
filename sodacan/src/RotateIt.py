"""
RotateIt is a graphical tool to help visualize 3d rotations on a cube from mark-world.com

rotatecube came from PyTeapot module for drawing rotating cube using OpenGL as per
quaternion or yaw, pitch, roll angles received over serial port.
started with: https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation/blob/master/pyteapot.py

Prerequisites:
- Python 3.10 (may work on earlier python)
- Requires pygame.  pip install pygame   as well as PyOpenGL  pip install PyOpenGL

Some modifications from mark-world.com (Mark Johnston)
Modified for keyboard increment entry to play with rotation in rpy mode
r and R inc/dec roll, p and P inc/dec pitch and y and Y inc/dec yaw.

z zeros all rotations.  i and I  inc/dec the increment amount.
I also show increment amount on the screen now for reference.

Holding the roll, pitch or yaw will auto-rotate but don't hold long as there is some queuing

Note that rotations happen in the global space and not relative to the objects own reference

Note that the modifications are only for rpy rotations and quaternion logic has not tracked the changes

Axis Description: 
OpenGL uses the Right-Hand Coordinates system where axis are as follows:
x-axis is pointing right, y-axis is pointing up, and z-axis is pointing out of the screen
That is as if looking down on the robot where forward is to the right
The program was for NorthEastDown (x into page, Y to the right, Z down.
The RobotMod: is to use OpenGL for roll, pitch, yaw instead of how program was running.
The quaternion mode has not changed so the two modes will disagree
The ROS standard is defined here, midpage:  https://www.ros.org/reps/rep-0103.html

Support Notes:
- The program is for keyboard input but some legacy inputs for serial are present but not supported
- Quaternion mode has not tracked the RPY changes for keyboard visualizations

"""

version = "20240303"

import pygame
import math
import time
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *

useSerial = False # set true for using serial for data transmission, false for wifi
useQuat = False   # set true for using quaternions, false for using y,p,r angles

if(useSerial):
    import serial
    ser = serial.Serial('/dev/ttyUSB0', 38400)
else:
    import socket

    UDP_IP = "0.0.0.0"
    UDP_PORT = 5005
    sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    sock.bind((UDP_IP, UDP_PORT))

def main():
    dispSizeX = 1280
    dispSizeY = 960
    video_flags = OPENGL | DOUBLEBUF
    pygame.init()
    screen = pygame.display.set_mode((dispSizeX, dispSizeY), video_flags)
    pygame.display.set_caption("RotateIt ROS 3D orientation visualization")
    resizewin(dispSizeX, dispSizeY)
    init()
    frames = 0
    ticks = pygame.time.get_ticks()

    # Set initial rotations. Normally 0 but can force specific values here
    rollNext = 0
    pitchNext = 0
    yawNext = 0
    angleInc = 5
    keyDown = 0 
    keyMods = 0
    shift = 0
    doNewRotate = False
    currentKey = K_a

    drawDelay = 0.1     # this effect auto-repeat
    while 1:
        event = pygame.event.poll()

        doNewRotate = False
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break
        if event.type == KEYUP:
            print("Key up")
            keyDown = 0
            keyMods = 0
        elif keyDown != 0:
            keyDown += 1
            # print("Key still down")
            if keyDown > 5:    # a crude delay for auto-repeat
                doNewRotate = True

        if (event.type == KEYDOWN):
            keyDown += 1       # This is for a crude auto-repeat for holding a key
            print("Key down")
            keyMods = pygame.key.get_mods()
            shift = keyMods & KMOD_SHIFT
            doNewRotate = True
            currentKey = event.key

        if doNewRotate:
            if currentKey == K_z:
              rollNext = 0
              pitchNext = 0
              yawNext = 0
            if currentKey == K_i:
              if shift == 0:
                angleInc += 1
              else:
                angleInc -= 1

            if currentKey == K_r:
              if shift == 0:
                rollNext += angleInc 
              else:
                rollNext -= angleInc
            if currentKey == K_p:
              if shift == 0:
                pitchNext += angleInc
              else:
                pitchNext -= angleInc
            if currentKey == K_y:
              if shift == 0:
                yawNext += angleInc
              else:
                yawNext -= angleInc
        if(useQuat):
            [w, nx, ny, nz] = read_data()
        else:
            # [yaw, pitch, roll] = read_data()
            [yaw, pitch, roll] = [yawNext, pitchNext, rollNext]
        if(useQuat):
            drawScreen(angleInc, w, nx, ny, nz)
        else:
            drawScreen(angleInc, 1, yaw, pitch, roll)
        pygame.display.flip()
        frames += 1

        time.sleep(drawDelay)

    print("fps: %d" % ((frames*1000)/(pygame.time.get_ticks()-ticks)))
    if(useSerial):
        ser.close()


def resizewin(width, height):
    """
    For resizing window
    """
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


def cleanSerialBegin():
    if(useQuat):
        try:
            line = ser.readline().decode('UTF-8').replace('\n', '')
            w = float(line.split('w')[1])
            nx = float(line.split('a')[1])
            ny = float(line.split('b')[1])
            nz = float(line.split('c')[1])
        except Exception:
            pass
    else:
        try:
            line = ser.readline().decode('UTF-8').replace('\n', '')
            yaw = float(line.split('y')[1])
            pitch = float(line.split('p')[1])
            roll = float(line.split('r')[1])
        except Exception:
            pass


def read_data():
    if(useSerial):
        ser.reset_input_buffer()
        cleanSerialBegin()
        line = ser.readline().decode('UTF-8').replace('\n', '')
        print(line)
    else:
        # Waiting for data from udp port 5005
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        line = data.decode('UTF-8').replace('\n', '')
        print(line)
                
    if(useQuat):
        w = float(line.split('w')[1])
        nx = float(line.split('a')[1])
        ny = float(line.split('b')[1])
        nz = float(line.split('c')[1])
        return [w, nx, ny, nz]
    else:
        yaw = float(line.split('y')[1])
        pitch = float(line.split('p')[1])
        roll = float(line.split('r')[1])
        return [yaw, pitch, roll]

# Define some 6-sided color schemes that can be used to paint the 6 faces of cubes or rods
cubeMultiColor = [ [0.0, 1.0, 0.0], [1.0, 0.5, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 1.0] ]
cubeRedColor = [ [1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0] ]
cubeGreenColor = [ [0.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0] ]
cubeBlueColor = [ [0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0] ]

# Draw a cube which is simply a special case of a rod
def drawCube(colors, size):
    drawRecRod(colors, size, size, size)

# Draw a rectangular solid shape which can also be a cube. 6 colors for sides are supplies
def drawRecRod(colors, szx, szy, szz):
    glBegin(GL_QUADS)
    glColor3f(colors[0][0], colors[0][1], colors[0][2])
    glVertex3f(szx, szy, -szz)
    glVertex3f(-szx, szy, -szz)
    glVertex3f(-szx, szy, szz)
    glVertex3f(szx, szy, szz)

    glColor3f(colors[1][0], colors[1][1], colors[1][2])
    glVertex3f(szx, -szy, szz)
    glVertex3f(-szx, -szy, szz)
    glVertex3f(-szx, -szy, -szz)
    glVertex3f(szx, -szy, -szz)

    glColor3f(colors[2][0], colors[2][1], colors[2][2])
    glVertex3f(szx, szy, szz)
    glVertex3f(-szx, szy, szz)
    glVertex3f(-szx, -szy, szz)
    glVertex3f(szx, -szy, szz)

    glColor3f(colors[3][0], colors[3][1], colors[3][2])
    glVertex3f(szx, -szy, -szz)
    glVertex3f(-szx, -szy, -szz)
    glVertex3f(-szx, szy, -szz)
    glVertex3f(szx, szy, -szz)

    glColor3f(colors[4][0], colors[4][1], colors[4][2])
    glVertex3f(-szx, szy, szz)
    glVertex3f(-szx, szy, -szz)
    glVertex3f(-szx, -szy, -szz)
    glVertex3f(-szx, -szy, szz)

    glColor3f(colors[5][0], colors[5][1], colors[5][2])
    glVertex3f(szx, szy, -szz)
    glVertex3f(szx, szy, szz)
    glVertex3f(szx, -szy, szz)
    glVertex3f(szx, -szy, -szz)
    glEnd()

# Draw the full view complete with text lines and of course the graphics
def drawScreen(inc, w, nx, ny, nz):
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    drawText((-2.6, 1.8, 2), "RotateIt!  A Tool to visualize Euler angle rotations for ROS robots Version: "+version, 20)
    drawText((-2.6, 1.6, 2), "Axis and Controls:  X to right is red and Roll is changed by r/R", 16)
    drawText((-2.6, 1.5, 2), "Y is up and Pitch changed by p/P.  Z is Out of screen and Yaw is changed by y/Y", 16)
    drawText((-2.6, 1.4, 2), "Angle increment in degrees: %f.  go back to Zero rotation with z" %(inc), 16)
    drawText((-2.6, -2, 2), "Press Escape to exit.", 16)

    if(useQuat):
        [yaw, pitch , roll] = quat_to_ypr([w, nx, ny, nz])
        drawText((-2.6, -1.8, 2), "Roll: %f, Pitch: %f, Yaw: %f" %(roll, pitch, yaw), 16)
        glRotatef(2 * math.acos(w) * 180.00/math.pi, -1 * nx, nz, ny)
    else:
        yaw = nx
        pitch = ny
        roll = nz
        yawR = (yaw / 180) * 3.14159
        pitchR = (pitch / 180) * 3.14159
        rollR = (roll / 180) * 3.14159
        drawText((-2.6, -1.8, 2), "Roll: %f (%f rad), Pitch: %f (%f rad), Yaw: %f (%f rad)" %\
            (roll, rollR,  pitch, pitchR, yaw, yawR), 16)

        # RobotMod: OpenGL uses x to right, y up and z out of the screen
        glRotatef(roll, 1.00, 0.00, 0.00)   # Experimental
        glRotatef(pitch, 0.00, 1.00, 0.00)  # Experimental
        glRotatef(yaw, 0.00, 0.00, 1.00)    # Experimental

    # Draw a small cube with different color sides
    drawCube(cubeMultiColor, 0.2)

    # Draw a larger multi-color cross with red, green, blue for original X, Y, Z 
    drawRecRod(cubeRedColor, 1.0, 0.05, 0.05)
    drawRecRod(cubeGreenColor, 0.05, 1.0, 0.05)
    drawRecRod(cubeBlueColor, 0.05, 0.05, 1.0)


def drawText(position, textString, size):
    font = pygame.font.SysFont("Courier", size, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

def quat_to_ypr(q):
    yaw   = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
    pitch = -math.asin(2.0 * (q[1] * q[3] - q[0] * q[2]))
    roll  = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
    pitch *= 180.0 / math.pi
    yaw   *= 180.0 / math.pi
    yaw   -= -0.13  # Declination at Chandrapur, Maharashtra is - 0 degress 13 min
    roll  *= 180.0 / math.pi
    return [yaw, pitch, roll]


if __name__ == '__main__':
    main()
