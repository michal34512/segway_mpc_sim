#!/usr/bin/env python3

""" ibalancingbot.py details a class to simulate a self balancing robot
    behavior.It uses a runge-kutta approach to deal with the system dynamic.

    Copyright (C) 2017 
    Remy GUYONNEAU, ISTIA/LARIS, University of Angers (France)
    remy.guyonneau@univ-angers.fr

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>."""

from math import cos
from math import sin
from math import pi
from math import atan


class IBalancingBot:
    """ A class to simulate the behavior of a self balancing robot
        The mathematical model is from the article : 
            'A comparison of Controllers for Balancing Two Wheeled Inverted 
            Pendulum Robot', Amir A. Bature, Salinda Buyamin, Mohammed N. Ahmad,
            Mustapha Muhammad.
    """

    def __init__(self):
        """ The constructor of the class """

        # variables for the model
        self.Mb = None  # kg      mass of main body (pendulum)
        self.Mw = None  # kg      mass of wheels
        self.d = None   # m       center of mass from base
        self.R = None   # m       Radius of wheel
        self.L = None   # m       Distance between the wheels
        self.Ix = None  # kg.m^2  Moment of inertia of body x-axis
        self.Iz = None  # kg.m^2  Moment of inertia of body z-axis
        self.Ia = None  # kg.m^2  Moment of inertia of wheel according to center
        self.g = None   # m/s^2   Acceleration due to gravity

        # variables for dynamic evaluation
        self.phi = None    # angle of the pendulum
        self.phip = None   # angular speed of the pendulum
        self.phipp = None  # angular acceleration of the pendulum
        self.x = None      # x position of the robot
        self.xp = None     # linear x speed of the robot
        self.xpp = None    # linear x acceleration of the robot
        self.psi = None    # rotation of the robot
        self.psip = None   # rotation angular speed of the robot
        self.psipp = None  # rotation angular acceleration of the robot

        # variables for the drawing
        self.d_rw = None       # wheel radius
        self.d_dstw = None     # distance between the wheels
        self.d_widthw = None   # width of a wheel
        self.d_heightp = None  # height of the pendulum
        self.d_centerp = None  # distance between the wheel and the pendulum center
        self.d_widthp = None   # width of the robot pipes

        self.initRobot()  # to initialize all the parameters of the system

    def initRobot(self):
        """ Function to initialize all the parameters of the system
            The value presented here are the value from the paper where the 
            model come from.
        """
        # variables for the model
        self.Mb = 13.3    # kg      mass of main body (pendulum)
        self.Mw = 1.89    # kg      mass of wheels
        self.d = 0.13     # m       center of mass from base
        self.R = 0.130    # m       Radius of wheel
        self.L = 0.325    # m       Distance between the wheels
        self.Ix = 0.1935  # kg.m^2  Moment of inertia of body x-axis
        self.Iz = 0.3379  # kg.m^2  Moment of inertia of body z-axis
        self.Iy = 0.3379  # kg.m^2  Moment of inertia of body y-axis : NOT A VALUE FROM THE PAPER
        self.Iwx = 0.1229  # kg.m^2  Moment of inertia of wheel according to center
        self.Iwz = 0.1000  # kg.m^2  Moment of inertia of wheel according to the Z-axis
        self.g = 9.81     # m/s^2   Acceleration due to gravity

        self.phi = 2*pi/180  # the pendulum is initially unstable
        self.phip = 0

        self.x = 0
        self.xp = 0

        self.psi = 0
        self.psip = 0

        # variables for the drawing
        self.d_rw = self.R
        self.d_dstw = self.L
        self.d_widthw = 0.01
        self.d_heightp = self.d
        self.d_centerp = self.d
        self.d_widthp = 0.01

        # to deal with the fact that the pendulum can not be lower than the ground
        # we compute the maximal possible angle for the pendulum
        self.phimax = pi/2 + atan(self.R/(2*self.d))
        self.phimin = -pi/2 - atan(self.R/(2*self.d))

    def f(self, phi, phip, x, xp, psi, psip, deltat, F):
        """ Function to evaluate the new state of the system according to:
            - the mathematical model (phipp=, xpp=, psipp=)
            - the current state (phi, phip, x, xp, psi, psip)
            - the time step deltat
            - the command F=[tau1, tau2] (constant over deltat)
            It returns phip, phipp, xp, xpp, psip, psipp
        """

        tau1 = F[0]
        tau2 = F[1]

        # to ease the reading
        Mb = self.Mb
        Mw = self.Mw
        d = self.d
        R = self.R
        L = self.L
        Ix = self.Ix
        Iz = self.Iz
        Iy = self.Iy
        Iwx = self.Iwx
        Iwz = self.Iwz
        g = self.g

        R2 = R**2
        d2 = d**2

        # # computation of phipp
        # den_phipp = 4*Iwx*Iy + 4*Iwx*Mb*d**2 + 2*Iy*Mb*R**2 + 4*Iy*Mw*R**2 - 2 * \
        #     Mb**2*R**2*d**2*cos(phi)**2 + 2*Mb**2*R**2*d**2 + 4*Mb*Mw*R**2*d**2
        # phipp = (2*Iwx*Ix*psip**2*sin(2*phi) - 2*Iwx*Iz*psip**2*sin(2*phi) + 4*Iwx*Mb*d*g*sin(phi) - 4*Iwx*tau1 - 4*Iwx*tau2 + Ix*Mb*R**2*psip**2*sin(2*phi) + 2*Ix*Mw*R**2*psip**2*sin(2*phi) - Iz*Mb*R**2*psip**2*sin(2*phi) - 2*Iz*Mw*R**2*psip**2 *
        #          sin(2*phi) - 2*Mb**2*R**2*d**2*phip**2*sin(phi)*cos(phi) + 2*Mb**2*R**2*d*g*sin(phi) + 4*Mb*Mw*R**2*d*g*sin(phi) - 2*Mb*R**2*tau1 - 2*Mb*R**2*tau2 - 2*Mb*R*d*tau1*cos(phi) - 2*Mb*R*d*tau2*cos(phi) - 4*Mw*R**2*tau1 - 4*Mw*R**2*tau2) / den_phipp

        # # computation of xpp
        # den_xpp = 4*Iwx*Iy + 4*Iwx*Mb*d**2 + 2*Iy*Mb*R**2 + 4*Iy*Mw*R**2 - 2 * \
        #     Mb**2*R**2*d**2*cos(phi)**2 + 2*Mb**2*R**2*d**2 + 4*Mb*Mw*R**2*d**2
        # xpp = (-2*Iwx*Ix*R*psip**2*sin(2*phi) + 2*Iwx*Iz*R*psip**2*sin(2*phi) - 4*Iwx*Mb*R*d*g*sin(phi) + 4*Iwx*R*tau1 + 4*Iwx*R*tau2 - Ix*Mb*R**3*psip**2*sin(2*phi) - Ix*Mb*R**2*d*psip**2*sin(2*phi)*cos(phi) - 2*Ix*Mw*R**3*psip**2*sin(2*phi) + 2*Iy*Mb*R**2*d*phip**2*sin(phi) + 2*Iy*R*tau1 + 2*Iy*R*tau2 + Iz*Mb*R**3*psip**2*sin(2*phi) + Iz*Mb*R**2*d*psip**2*sin(2*phi)*cos(phi) + 2*Iz *
        #        Mw*R**3*psip**2*sin(2*phi) + 2*Mb**2*R**3*d**2*phip**2*sin(phi)*cos(phi) - 2*Mb**2*R**3*d*g*sin(phi) + 2*Mb**2*R**2*d**3*phip**2*sin(phi) - 2*Mb**2*R**2*d**2*g*sin(phi)*cos(phi) - 4*Mb*Mw*R**3*d*g*sin(phi) + 2*Mb*R**3*tau1 + 2*Mb*R**3*tau2 + 4*Mb*R**2*d*tau1*cos(phi) + 4*Mb*R**2*d*tau2*cos(phi) + 2*Mb*R*d**2*tau1 + 2*Mb*R*d**2*tau2 + 4*Mw*R**3*tau1 + 4*Mw*R**3*tau2) / den_xpp

        # # computation of psipp
        # den_psipp = 4*Iwx*L**2*R + 8*Iwz*R**3 + 8*Ix*R**3 * \
        #     sin(phi)**2 + 8*Iz*R**3*cos(phi)**2 + 4*L**2*Mw*R**3
        # psipp = (Iwx*L**2*psip**2*tau1 - Iwx*L**2*psip**2*tau2 + 4*Iwx*R**2*phip**2*tau1 - 4*Iwx*R**2*phip**2*tau2 + 8*Iwx*R*phip*tau1*xp - 8*Iwx*R*phip*tau2*xp + 4*Iwx*tau1*xp**2 - 4*Iwx*tau2*xp**2 + 2*Iwz*R**2*psip**2*tau1 - 2*Iwz*R**2*psip**2*tau2 - 8*Ix*R**3*phip*psip*sin(2*phi) + 2*Ix*R**2*psip**2*tau1*sin(phi)**2 - 2*Ix*R**2*psip**2*tau2*sin(phi)**2 + 2*Iy*R**2*phip**2*tau1 - 2*Iy*R**2*phip**2*tau2 + 8*Iz*R**3*phip*psip*sin(2*phi) + 2*Iz*R**2*psip**2*tau1*cos(phi)**2 - 2*Iz*R**2*psip**2*tau2*cos(phi)**2 + L**2*Mw*R**2*psip**2*tau1 - L**2*Mw*R**2*psip**2*tau2 +
        #          2*Mb*R**4*phip**2*tau1 - 2*Mb*R**4*phip**2*tau2 + 4*Mb*R**3*d*phip**2*tau1*cos(phi) - 4*Mb*R**3*d*phip**2*tau2*cos(phi) + 4*Mb*R**3*phip*tau1*xp - 4*Mb*R**3*phip*tau2*xp + 2*Mb*R**2*d**2*phip**2*tau1 - 2*Mb*R**2*d**2*phip**2*tau2 - 4*Mb*R**2*d*g*tau1*cos(phi) + 4*Mb*R**2*d*g*tau2*cos(phi) + 4*Mb*R**2*d*phip*tau1*xp*cos(phi) - 4*Mb*R**2*d*phip*tau2*xp*cos(phi) + 2*Mb*R**2*tau1*xp**2 - 2*Mb*R**2*tau2*xp**2 + 4*Mw*R**4*phip**2*tau1 - 4*Mw*R**4*phip**2*tau2 + 8*Mw*R**3*phip*tau1*xp - 8*Mw*R**3*phip*tau2*xp + 4*Mw*R**2*tau1*xp**2 - 4*Mw*R**2*tau2*xp**2) / den_psipp

        omega_L_target = F[0]
        omega_R_target = F[1]
        xp_target = R * ((omega_R_target + omega_L_target) / 2.0 + phip)
        psip_target = R * (omega_R_target - omega_L_target) / L

        xpp_in = (xp_target - xp) / deltat
        psipp_in = (psip_target - psip) / deltat

        den_phipp = 4*Iwx*R + 2*Iy*R + 2*Mb*R**3 + 4 * \
            Mb*R**2*d*cos(phi) + 2*Mb*R*d**2 + 4*Mw*R**3
        phipp = (-4*Iwx*xpp_in + Ix*R*psip**2*sin(2*phi) - Iz*R*psip**2*sin(2*phi) + 2*Mb*R**2*d*phip**2*sin(phi) -
                 2*Mb*R**2*xpp_in + 2*Mb*R*d*g*sin(phi) - 2*Mb*R*d*xpp_in*cos(phi) - 4*Mw*R**2*xpp_in) / den_phipp

        xpp = xpp_in
        psipp = psipp_in

        return phip*deltat, phipp*deltat, xp*deltat, xpp*deltat, psip*deltat, psipp*deltat

    def runge_kutta(self, deltat, F):
        """ Function that call the f() function defined above, and uses a 
            runge-kutta approach to deal with the differential equations
        """
        k1phip, k1phipp, k1xp, k1xpp, k1psip, k1psipp = \
            self.f(self.phi, self.phip, self.x, self.xp,
                   self.psi, self.psip, deltat, F)
        k2phip, k2phipp, k2xp, k2xpp, k2psip, k2psipp = \
            self.f(self.phi + k1phip/2, self.phip + k1phipp/2, self.x + k1xp/2,
                   self.xp + k1xpp/2, self.psi + k1psip/2, self.psip + k1psipp/2, deltat, F)
        k3phip, k3phipp, k3xp, k3xpp, k3psip, k3psipp = \
            self.f(self.phi + k2phip/2, self.phip + k2phipp/2, self.x + k2xp/2,
                   self.xp + k2xpp/2, self.psi + k2psip/2, self.psip + k2psipp/2, deltat, F)
        k4phip, k4phipp, k4xp, k4xpp, k4psip, k4psipp = \
            self.f(self.phi + k3phip, self.phip + k3phipp, self.x + k3xp,
                   self.xp + k3xpp, self.psi + k3psip, self.psip + k3psipp, deltat, F)

        self.phi = self.phi + 1/6.0*(k1phip+2*k2phip+2*k3phip+k4phip)
        self.phip = self.phip + 1/6.0*(k1phipp+2*k2phipp+2*k3phipp+k4phipp)

        self.xp = self.xp + 1/6.0*(k1xpp+2*k2xpp+2*k3xpp+k4xpp)
        self.x = self.x + 1/6.0*(k1xp+2*k2xp+2*k3xp+k4xp)

        self.psip = self.psip + 1/6.0*(k1psipp+2*k2psipp+2*k3psipp+k4psipp)
        self.psi = self.psi + 1/6.0*(k1psip+2*k2psip+2*k3psip+k4psip)

        # to deal with the fact that the pendulum can not be lower than the ground.
        # we also put the acceleration to 0 to avoid that the robot moves
        #   when the pendulum is down
        if (self.phi > self.phimax):
            self.phi = self.phimax
            self.phip = 0
            self.xp = 0
        if (self.phi < self.phimin):
            self.phi = self.phimin
            self.phip = 0
            self.xp = 0

    def dynamics(self, deltat, F):
        """ Function to provide an interface between the model and the display
        """
        self.runge_kutta(deltat, F)
