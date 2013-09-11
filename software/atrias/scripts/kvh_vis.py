#!/usr/bin/env python

import sys
from math import sqrt, pi, acos

# OpenGL
try:
    from OpenGL.GL import *
    from OpenGL.GLUT import*
    from OpenGL.GLU import *
    print "[Vis] OpenGL successfully imported!"
except:
    print "[Vis] PyOpenGL not installed properly. Exiting..."
    exit(1)


# =============================================================================
# Telemetry data
# =============================================================================

# Initial DCM values.
dcm = [[1.0, 0.0, 0.0],
       [0.0, 1.0, 0.0],
       [0.0, 0.0, 1.0]]


# =============================================================================
# OpenGL elements
# =============================================================================

# Number of the glut window.
window = 0

def drawScene():
    global quadratic

    # Define axes to draw.
    axes = dcm

    # Initial vertex values for a box drawn around the DCM.
    dcmBox = [[-1,-1,1], [-1,1,1], [1,1,1], [1,-1,1], [-1,-1,-1], [-1,1,-1], [1,1,-1], [1,-1,-1]]

    # Clear screen and depth buffer.
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    # Reset view
    glLoadIdentity()

    # Move object into screen so it's not in my face (i.e,. invisible).
    glTranslatef(0.0, 0.0, -3.0)

    # Syntax: glRotatef(angle, x, y, z)
    glRotatef(-90.0, 1.0, 0.0, 0.0)
    #glRotatef(-20.0, 0.0, 1.0, 0.0)

    # =========================================================================
    # DCM visualization
    # =========================================================================

    # Calculate vertex locations for a box. Refer to the declaration of dcmBox to
    # see the order of the vertices.
    for i in range(3):
        dcmBox[0][i] = (-axes[0][i] -axes[1][i] +axes[2][i]/2) * 0.4
        dcmBox[1][i] = (-axes[0][i] +axes[1][i] +axes[2][i]/2) * 0.4
        dcmBox[2][i] = ( axes[0][i] +axes[1][i] +axes[2][i]/2) * 0.4
        dcmBox[3][i] = ( axes[0][i] -axes[1][i] +axes[2][i]/2) * 0.4
        dcmBox[4][i] = (-axes[0][i] -axes[1][i] -axes[2][i]/2) * 0.4
        dcmBox[5][i] = (-axes[0][i] +axes[1][i] -axes[2][i]/2) * 0.4
        dcmBox[6][i] = ( axes[0][i] +axes[1][i] -axes[2][i]/2) * 0.4
        dcmBox[7][i] = ( axes[0][i] -axes[1][i] -axes[2][i]/2) * 0.4

    # Draw the axes of whichever DCM we're using.
    glBegin(GL_LINES)
    glColor3f(1,0,0)
    glVertex3f(0.0, 0.0, 0.0); glVertex3fv(axes[0])
    glColor3f(0,1,0)
    glVertex3f(0.0, 0.0, 0.0); glVertex3fv(axes[1])
    glColor3f(0,0,1)
    glVertex3f(0.0, 0.0, 0.0); glVertex3fv(axes[2])
    glEnd()

    # Draw a box around the DCM to help visualize.
    glBegin(GL_LINES)
    glColor3f(1,1,1)
    glVertex3fv(dcmBox[0]); glVertex3fv(dcmBox[1])
    glVertex3fv(dcmBox[0]); glVertex3fv(dcmBox[3])
    glVertex3fv(dcmBox[0]); glVertex3fv(dcmBox[4])
    glVertex3fv(dcmBox[2]); glVertex3fv(dcmBox[1])
    glVertex3fv(dcmBox[2]); glVertex3fv(dcmBox[3])
    glVertex3fv(dcmBox[2]); glVertex3fv(dcmBox[6])
    glVertex3fv(dcmBox[5]); glVertex3fv(dcmBox[1])
    glVertex3fv(dcmBox[5]); glVertex3fv(dcmBox[4])
    glVertex3fv(dcmBox[5]); glVertex3fv(dcmBox[6])
    glVertex3fv(dcmBox[7]); glVertex3fv(dcmBox[3])
    glVertex3fv(dcmBox[7]); glVertex3fv(dcmBox[4])
    glVertex3fv(dcmBox[7]); glVertex3fv(dcmBox[6])
    glEnd()

    # Draw static axes.
    glBegin(GL_LINES)
    glColor3f(0.2, 0.0, 0.0)
    glVertex3f(0.0, 0.0, 0.0); glVertex3f(1.5, 0.0, 0.0)
    glColor3f(0.0, 0.2, 0.0)
    glVertex3f(0.0, 0.0, 0.0); glVertex3f(0.0, 1.5, 0.0)
    glColor3f(0.0, 0.0, 0.2)
    glVertex3f(0.0, 0.0, 0.0); glVertex3f(0.0, 0.0, 1.5)
    glColor3f(0.2, 0.2, 0.2)
    glEnd()

    # Draw a static box.
    #glutWireCube(1.2)

    # Since this is double buffered, swap the buffers to display what just got drawn.
    glutSwapBuffers()


def resizeScene(width, height):
    # Protect against divide by zero when window size is small.
    if height == 0:
        height = 1

    # Reset current viewport and perspective transformation.
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, float(width)/float(height), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)


def initGL(width, height):
    global quadratic

    quadratic = quadratic = gluNewQuadric()

    glClearColor(0.0, 0.0, 0.0, 0.0)   # Clear background color to black.
    glClearDepth(1.0)                  # Enable clearing of the depth buffer.
    glDepthFunc(GL_LESS)               # Type of depth test.
    glEnable(GL_DEPTH_TEST)            # Enable depth testing.
    glShadeModel(GL_SMOOTH)            # Enable smooth color shading.

    glMatrixMode(GL_PROJECTION)   # Specify which matrix is the current matrix.
    glLoadIdentity()              # Reset the projection matrix.

    gluPerspective(45.0, float(width)/float(height), 0.1, 100.0)

    glMatrixMode(GL_MODELVIEW)


def setupVisualizer():
    global window
    glutInit(sys.argv)

    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
    glutInitWindowSize(640, 480)
    glutInitWindowPosition(0, 0)

    # Initialize window so we can close it later.
    window = glutCreateWindow("IMU visualization")

    # Register the drawing function with glut.
    glutDisplayFunc(drawScene)

    # When doing nothing, redraw scene.
    glutIdleFunc(drawScene)

    # Register the function called when window is resized.
    glutReshapeFunc(resizeScene)

    # Register the function called when key is pressed.
    #glutKeyboardFunc(keyPressed)

    # Initialize window.
    initGL(640, 480)


def updateDCM(newDCM):
    dcm[0][0] = newDCM[0][0]
    dcm[0][1] = newDCM[0][1]
    dcm[0][2] = newDCM[0][2]
    dcm[1][0] = newDCM[1][0]
    dcm[1][1] = newDCM[1][1]
    dcm[1][2] = newDCM[1][2]
    dcm[2][0] = newDCM[2][0]
    dcm[2][1] = newDCM[2][1]
    dcm[2][2] = newDCM[2][2]


def updateVisualizer():
    # Start event processing engine.
    drawScene()
    glutMainLoopEvent()


# vim: expandtab

