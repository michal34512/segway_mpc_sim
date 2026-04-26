#!/usr/bin/env python3

import sys
import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from math import cos, sin, pi, sqrt
from iballancingbot import IBalancingBot
from mpc import MPC  # Importujemy naszego nowego potwora sterującego!

print("<-------------------------------------------------------------->")
print("<--                    IBBOT simulator                       -->")
print("<-- (Now powered by Advanced Model Predictive Control!)      -->")
print("<-------------------------------------------------------------->\n")

print(" Page down : activate simulation")
print(" F1 : move backward")
print(" F2 : move forward")
print(" F3 : turn")
print(" F4 : stop motion")
print(" F5 : ON/OFF MPC correction")
print(" F6 : reset simulation")
print(" F7 : add pertubation (push the robot)")
print(" F8 : add pertubation (push the robot)")
print(" F9 : ON/OFF follow the robot")

# -- Programme variables --
ref_time, FPS = 0, 50
dt_ctrl = 1.0 / FPS  # Czas trwania jednej pętli sterowania (20ms)

CameraPosX, CameraPosY, CameraPosZ = 0.0, 3.0, 10.0
ViewUpX, ViewUpY, ViewUpZ = 0.0, 1.0, 0.0
CenterX, CenterY, CenterZ = 0.0, 0.0, 0.0
follow_robot = False

Theta, dtheta = 0.0, 2*pi/100.0
Radius = sqrt(CameraPosX**2 + CameraPosZ**2)

quadric = gluNewQuadric()
gluQuadricNormals(quadric, GLU_SMOOTH)

myBot = IBalancingBot()

# Zmienna na nasz nowy kontroler MPC
myMPC = None

posx = posz = 0
F = [0, 0]  # Nasze nowe wejścia F to teraz docelowe PRĘDKOŚCI KÓŁ (rad/s)
use_mpc = False

speed = 0
current_speed = 0
turn = 0
current_turn = 0


def initMPC():
    global myMPC, myBot, dt_ctrl

    # Horyzont 40 kroków przy 50Hz daje predykcję na 0.8 sekundy
    myMPC = MPC(N=40, dt=dt_ctrl)

    # Wagi: [X, X_dot, Phi, Phi_dot, Psi, Psi_dot]
    # Używamy tylko wag prędkości (X i Psi) oraz potężnej wagi dla kąta pochylenia
    myMPC.setWeights([0.0, 2.0, 100.0, 5.0, 0.0, 2.0], R_diag=[0.1, 0.1])
    print("\tNieliniowy MIMO MPC (ERK) Initialized!")


def correction():
    global myBot, F, myMPC, use_mpc
    global speed, turn

    # if use_mpc and myMPC is not None:

    #     # Testowanie naszej "ósemki" przy użyciu pełnego MIMO
    #     time_s = glutGet(GLUT_ELAPSED_TIME) / 1000.0
    #     target_speed = 0.4
    #     target_turn = 1.2 * sin(0.5 * time_s)

    #     # Nowy wektor stanu (6-wymiarowy)
    #     # Przekazujemy błędy prędkości, aby MPC sprowadził je do "zera"
    #     state = np.array([
    #         0.0,
    #         myBot.xp - target_speed,
    #         myBot.phi,
    #         myBot.phip,
    #         0.0,
    #         myBot.psip - target_turn
    #     ])

    #     # MIMO MPC wypluwa teraz 2 wartości - optymalne przyśpieszenie lewego i prawego koła
    #     u_opt = myMPC.update(state)
    #     a_L = u_opt[0]
    #     a_R = u_opt[1]

    #     # 1. Obecne prędkości liniowe kół (obliczane z bazy robota)
    #     v_L = myBot.xp - (myBot.L * myBot.psip) / 2.0
    #     v_R = myBot.xp + (myBot.L * myBot.psip) / 2.0

    #     # 2. Całkujemy wyznaczone przez MPC przyśpieszenia, by dostać nowe prędkości liniowe kół
    #     v_L_target = v_L + a_L * dt_ctrl
    #     v_R_target = v_R + a_R * dt_ctrl

    #     # 3. Dodajemy poprawkę kinematyczną na toczenie opadającego robota (phip)
    #     omega_L = v_L_target / myBot.R - myBot.phip
    #     omega_R = v_R_target / myBot.R - myBot.phip

    #     F = [omega_L, omega_R]
    # else:
    #     F = [0, 0]
    if use_mpc and myMPC is not None:

        # Testowanie naszej "ósemki" przy użyciu pełnego MIMO
        time_s = glutGet(GLUT_ELAPSED_TIME) / 1000.0

        # Nowy wektor stanu (6-wymiarowy)
        # Przekazujemy błędy prędkości, aby MPC sprowadził je do "zera"
        state = np.array([
            0.0,
            myBot.xp,
            myBot.phi,
            myBot.phip,
            0.0,
            myBot.psip
        ])

        # MIMO MPC wypluwa teraz 2 wartości - optymalne przyśpieszenie lewego i prawego koła
        u_opt = myMPC.update(state)
        a_L = u_opt[0]
        a_R = u_opt[1]

        # 1. Obecne prędkości liniowe kół (obliczane z bazy robota)
        v_L = myBot.xp - (myBot.L * myBot.psip) / 2.0
        v_R = myBot.xp + (myBot.L * myBot.psip) / 2.0

        # 2. Całkujemy wyznaczone przez MPC przyśpieszenia, by dostać nowe prędkości liniowe kół
        v_L_target = v_L + a_L * dt_ctrl
        v_R_target = v_R + a_R * dt_ctrl

        # 3. Dodajemy poprawkę kinematyczną na toczenie opadającego robota (phip)
        omega_L = v_L_target / myBot.R - myBot.phip
        omega_R = v_R_target / myBot.R - myBot.phip

        F = [omega_L, omega_R]
    else:
        F = [0, 0]


def animation():
    global ref_time, FPS, F, posx, posz, myBot, dt_ctrl
    delta_t = 0.001

    if glutGet(GLUT_ELAPSED_TIME)-ref_time > (dt_ctrl)*1000:
        # Obliczamy, ile drobnych kroków fizyki (delta_t) przypada na jedną klatkę sterowania (dt_ctrl)
        steps = int(dt_ctrl / delta_t)
        dst = 0
        for i in range(0, steps):
            myBot.dynamics(delta_t, F)
            dst = (myBot.xp * delta_t)
            posx += dst*cos(myBot.psi)
            posz += (-dst*sin(myBot.psi))

        # Obliczamy nowe sterowanie z użyciem MPC
        correction()
        glutPostRedisplay()
        ref_time = glutGet(GLUT_ELAPSED_TIME)

# ... (Reszta funkcji rysujących: drawGround, drawIBot, drawWheels, drawWheel, drawBase, drawPendulum - pozostają bez zmian)


def drawGround():
    nb_rows, nb_cols = 20, 20
    for r in range(0, nb_rows):
        for c in range(0, nb_cols):
            if (r % 2 and not c % 2 or not r % 2 and c % 2):
                glColor3d(0.3, 0.3, 0.3)
            else:
                glColor3d(0.1, 0.1, 0.1)
            glBegin(GL_QUADS)
            glVertex3f(c-nb_cols/2, 0, r-nb_rows/2)
            glVertex3f(c-nb_cols/2+1, 0, r-nb_rows/2)
            glVertex3f(c-nb_cols/2+1, 0, r-nb_rows/2+1)
            glVertex3f(c-nb_cols/2, 0, r-nb_rows/2+1)
            glEnd()


def drawIBot(ibot):
    global myBot
    drawWheels(ibot.d_dstw, ibot.d_widthw, ibot.d_rw)
    drawBase(ibot.d_dstw, ibot.d_widthp)
    drawPendulum(ibot.d_heightp, ibot.d_widthp, ibot.d_centerp)


def drawWheels(dst_wheels, width_wheel, radius_wheel):
    global quadric
    glPushMatrix()
    glTranslatef(0, 0, -dst_wheels/2)
    drawWheel(radius_wheel/2, width_wheel)
    glPopMatrix()
    glPushMatrix()
    glTranslatef(0, 0, dst_wheels/2)
    drawWheel(radius_wheel/2, width_wheel)
    glPopMatrix()


def drawWheel(size, width):
    global quadric
    glPushMatrix()
    glColor3d(0.4, 0.4, 0.4)
    glTranslatef(0, 0, -width/2)
    gluCylinder(quadric, size, size, width, 32, 16)
    glPopMatrix()
    glPushMatrix()
    glColor3d(0.6, 0.6, 0.6)
    glTranslatef(0, 0, -width/2)
    gluDisk(quadric, 0, size, 32, 32)
    glPopMatrix()
    glPushMatrix()
    glColor3d(0.6, 0.6, 0.6)
    glTranslatef(0, 0, width/2)
    gluDisk(quadric, 0, size, 32, 32)
    glPopMatrix()


def drawBase(dst_wheels, radius_pipe):
    global quadric
    glPushMatrix()
    glColor3d(0.4, 0.4, 1)
    glTranslatef(0, 0, -dst_wheels/2)
    gluCylinder(quadric, radius_pipe/2, radius_pipe/2, dst_wheels, 32, 16)
    glPopMatrix()


def drawPendulum(height_pendulum, radius_pipe, center_pendulum):
    global quadric
    glPushMatrix()
    glColor3d(1, 0.4, 0.4)
    glRotatef(90, -1, 0, 0)
    gluCylinder(quadric, radius_pipe/2, radius_pipe/2, height_pendulum, 32, 16)
    glPopMatrix()
    glPushMatrix()
    glColor3d(0.4, 1, 0.4)
    glTranslatef(0, center_pendulum, 0)
    glutSolidSphere(radius_pipe, 32, 32)
    glPopMatrix()


def Displayfct():
    scaleCoeff = 10
    glClearColor(0, 0, 0, 0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    glFrustum(-1, 1, -1, 1, 1, 80.)

    global CameraPosX, CameraPosY, CameraPosZ, CenterX, CenterY, CenterZ, ViewUpX, ViewUpY, ViewUpZ
    global myBot, posx, posz, follow_robot

    if (follow_robot):
        CenterX = -posx*scaleCoeff
        CenterZ = -posz*scaleCoeff

    gluLookAt(CameraPosX, CameraPosY, CameraPosZ, CenterX,
              CenterY, CenterZ, ViewUpX, ViewUpY, ViewUpZ)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    glPushMatrix()
    glScalef(scaleCoeff, scaleCoeff, scaleCoeff)
    glTranslatef(0, -myBot.d_rw/2, 0)
    drawGround()
    glPopMatrix()

    glPushMatrix()
    glScalef(scaleCoeff, scaleCoeff, scaleCoeff)
    glTranslatef(-posx, 0, -posz)
    glRotatef(myBot.psi*180/pi, 0, 1, 0)
    glRotatef(myBot.phi*180/pi, 0, 0, 1)
    drawIBot(myBot)
    glPopMatrix()

    glutSwapBuffers()


def ReshapeFunc(w, h):
    glViewport(0, 0, w, h)


def rotate_camera(angle):
    global CameraPosX, CameraPosZ, Radius, Theta
    Theta += angle
    CameraPosZ = Radius*cos(Theta)
    CameraPosX = Radius*sin(Theta)
    return 0


def zoom_camera(factor):
    global CameraPosX, CameraPosY, CameraPosZ, Radius
    CameraPosX, CameraPosY, CameraPosZ = factor * \
        CameraPosX, factor*CameraPosY, factor*CameraPosZ
    Radius = sqrt(CameraPosX**2 + CameraPosZ**2)


def SpecialFunc(skey, x, y):
    global CameraPosY, Theta, dtheta
    global myBot, speed, use_mpc, turn
    global posx, posz, follow_robot

    if glutGetModifiers() == GLUT_ACTIVE_SHIFT:
        if skey == GLUT_KEY_UP:
            CameraPosY += 0.3
        if skey == GLUT_KEY_DOWN:
            CameraPosY -= 0.3
    else:
        if skey == GLUT_KEY_LEFT:
            rotate_camera(-dtheta)
        elif skey == GLUT_KEY_RIGHT:
            rotate_camera(dtheta)
        elif skey == GLUT_KEY_UP:
            zoom_camera(0.9)
        elif skey == GLUT_KEY_DOWN:
            zoom_camera(1.1)
        elif skey == GLUT_KEY_PAGE_DOWN:
            print("\tSimulation ON")
            glutIdleFunc(animation)
        elif skey == GLUT_KEY_PAGE_UP:
            print("\tSimulation PAUSE")
            glutIdleFunc(None)
        elif skey == GLUT_KEY_F1:
            speed = 0.5   # Przyspieszamy sterowanie dla testu MPC
        elif skey == GLUT_KEY_F2:
            speed = -0.5
        elif skey == GLUT_KEY_F3:
            turn = 1.0
        elif skey == GLUT_KEY_F4:
            speed = 0
            turn = 0
        elif skey == GLUT_KEY_F5:
            use_mpc = not (use_mpc)
            print("\tUsing MPC : " + str(use_mpc))
            if (use_mpc):
                initMPC()
        elif skey == GLUT_KEY_F6:
            posx = posz = 0
            myBot.initRobot()
            if (use_mpc):
                initMPC()
        elif skey == GLUT_KEY_F7:
            myBot.phi += (10*pi/180)
        elif skey == GLUT_KEY_F8:
            myBot.phi += (-10*pi/180)
        elif skey == GLUT_KEY_F9:
            follow_robot = not (follow_robot)
            print("\tFollowing robot : " + str(follow_robot))
    glutPostRedisplay()
    return 0


glutInit(sys.argv)
glutInitWindowPosition(100, 100)
glutInitWindowSize(600, 600)
glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
glutCreateWindow(b"IBBot simulator (MPC)")

glClearColor(0.0, 0.0, 0.0, 0.0)
glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
glEnable(GL_DEPTH_TEST)

glutDisplayFunc(Displayfct)
glutReshapeFunc(ReshapeFunc)
glutSpecialFunc(SpecialFunc)

glutMainLoop()
