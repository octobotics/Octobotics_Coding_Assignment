#!/usr/bin/env python
# /* ------------- COPYRIGHT NOTICE ---------------
#
# Copyright (C) 2022 Octobotics Tech Pvt. Ltd. All Rights Reserved.
# Do not remove this copyright notice.
# Do not use, reuse, copy, merge, publish, sub-license, sell, distribute or modify this code - except without explicit,
# written permission from Octobotics Tech Pvt. Ltd.
# Contact connect@octobotics.tech for full license information.
# Author: Yogesh Phalak
#
# ------------- COPYRIGHT NOTICE ---------------*/

__copyright__ = "Copyright (C) 2022 Octobotics Tech Pvt. Ltd. All Rights Reserved. " \
                "Do not remove this copyright notice. " \
                "Do not use, reuse, copy, merge, publish, sub-license, sell, distribute " \
                "or modify this code - except without explicit, written permission from Octobotics Tech Pvt. Ltd."
__license__ = "Contact connect@octobotics.tech for full license information."
__author__ = "Yogesh Phalak"

'''
This node simulates the inverted pendulum.
'''

import rospy
import rospkg
import pygame as pg
import sys
from math import sin, cos, pi
from inverted_pendulum_sim.msg import CurrentState
from inverted_pendulum_sim.msg import ControlForce
from inverted_pendulum_sim.srv import SetParams, SetParamsResponse, SetParamsRequest

'''
pygame initialization and parameter settings
'''
pg.init()
SCREENSIZE = (800, 700)
GRIDSIZE = 5
CAPTIONS = "Inverted Pendulum Sim"

BLACK = (0, 0, 0)
BACKALPHA = 255

'''
Color palette
'''
WHITE = (255, 255, 255)
GREEN = (51, 102, 0)
RED = (128, 0, 0)
ORANGE = (255, 165, 0)
BLUE = (25, 25, 112)
VIOLET = (138, 43, 226)
PINK = (255, 20, 147)
GRIDCOLOR = (204, 204, 255)
WHEELCOLOR = (153, 51, 255)
CARTCOLOR = (255, 255, 153)
PENDULUMCOLOR = (102, 51, 0)
BLOBCOLOR = (102, 0, 102)

DISPSURFACE = pg.display.set_mode(SCREENSIZE, pg.SRCALPHA, 32)
pg.display.set_caption(CAPTIONS)

FPS = 100
DRAW_RATE = 2
CLOCK = pg.time.Clock()


class InvertedPendulum:
    """
    This class simulates the inverted pendulum.
    """

    def __init__(self):
        """
        @brief: Constructor - Initializes the inverted pendulum simulation.
        """
        self.x = 0
        self.y = SCREENSIZE[1] / 2
        self.x_dot = 0
        self.x_d_dot = 0
        self.theta = -pi / 3
        self.theta_dot = 0
        self.theta_d_dot = 0
        self.length = 200
        self.cart_m = 10
        self.pendulum_m = 10
        self.dt = 5.0 / FPS
        self.g = -9.8

        self.force = 0

        self.cart_w = 129.4424
        self.cart_h = 80.0
        self.wheel_r = 10.0

        self.rospack = rospkg.RosPack()

        self.current_state_pub = None
        self.ctrl_force_sub = None
        self.set_params_srv = None

        self.init_pubs_subs()

        self.draw_counter = 0

        '''
        starting main loop
        '''
        self.main_loop()

    def init_pubs_subs(self):
        """
        @brief: Initializes the publishers, subscribers and services.
        :return: none
        """
        self.current_state_pub = rospy.Publisher('/inverted_pendulum/current_state', CurrentState, queue_size=10)
        self.ctrl_force_sub = rospy.Subscriber('/inverted_pendulum/control_force', ControlForce,
                                               self.ctrl_force_callback)
        self.set_params_srv = rospy.Service('/inverted_pendulum/set_params', SetParams, self.set_params_srv_callback)

    def set_params_srv_callback(self, req):
        """
        @brief: Callback for the service to set the parameters and initial conditions.
        :param req: type SetParamsRequest
        :return res: type SetParamsResponse
        """
        if req.pendulum_mass <= 0:
            rospy.logwarn(
                "[InvertedPendulum]: Pendulum mass cannot be less than or equal to zero, using default value 10kg")
            self.pendulum_m = 10
        else:
            self.pendulum_m = req.pendulum_mass

        if req.pendulum_length <= 0:
            rospy.logwarn(
                "[InvertedPendulum]: Pendulum length cannot be less than or equal to zero, using default value 200m")
            self.length = 200
        else:
            self.length = req.pendulum_length

        if req.cart_mass <= 0:
            rospy.logwarn(
                "[InvertedPendulum]: Cart mass cannot be less than or equal to zero, using default value 10kg")
            self.cart_m = 10
        else:
            self.cart_m = req.cart_mass

        self.theta = req.theta_0
        self.theta_dot = req.theta_dot_0
        self.theta_d_dot = req.theta_dot_dot_0
        self.x = req.cart_x_0
        self.x_dot = req.cart_x_dot_0
        self.x_d_dot = req.cart_x_dot_dot_0

        rospy.loginfo("[InvertedPendulum]: Parameters set successfully")

        res = SetParamsResponse()
        res.success = True
        res.message = "Parameters set successfully"
        return res

    def ctrl_force_callback(self, msg):
        """
        @brief: Callback for the subscriber to get the control force.
        :param msg:
        :return: none
        """
        self.force = msg.force

    def background(self):
        """
        @brief: Draws the background of the simulation. (Watermark of Octobotics logo)
        :return: none
        """
        img = pg.image.load(self.rospack.get_path('inverted_pendulum_sim') + '/media/octobotics_c.jpeg')
        DISPSURFACE.blit(img, (0, 0))

        for i in range(0, SCREENSIZE[0], GRIDSIZE):
            pg.draw.line(DISPSURFACE, GRIDCOLOR, (i, 0), (i, SCREENSIZE[1]))
        for i in range(0, SCREENSIZE[1], GRIDSIZE):
            pg.draw.line(DISPSURFACE, GRIDCOLOR, (0, i), (SCREENSIZE[0], i))

        pg.draw.line(DISPSURFACE, GREEN, (0, SCREENSIZE[1] / 2 + self.cart_h / 2 + 2 * self.wheel_r),
                     (SCREENSIZE[0], SCREENSIZE[1] / 2 + + self.cart_h / 2 + 2 * self.wheel_r), 5)
        pg.draw.line(DISPSURFACE, BLACK, (SCREENSIZE[0] / 2, 0), (SCREENSIZE[0] / 2, SCREENSIZE[1]), 2)

    def cart(self):
        """
        @brief: Draws the cart.
        :return: none
        """
        x = self.x + SCREENSIZE[0] / 2 - self.cart_w / 2
        y = self.y - self.cart_h / 2

        pg.draw.rect(DISPSURFACE, CARTCOLOR, (x, y, self.cart_w, self.cart_h))
        pg.draw.rect(DISPSURFACE, BLACK, (x, y, self.cart_w, self.cart_h), 3)

        pg.draw.circle(DISPSURFACE, BLACK, (x, y + self.cart_h / 2 + 4 * self.wheel_r), 2 * self.wheel_r, 7)
        pg.draw.circle(DISPSURFACE, WHEELCOLOR, (x, y + self.cart_h / 2 + 4 * self.wheel_r), 2 * (self.wheel_r - 3.5))
        pg.draw.circle(DISPSURFACE, BLACK, (x + self.cart_w, y + self.cart_h / 2 + 4 * self.wheel_r), 2 * self.wheel_r,
                       7)
        pg.draw.circle(DISPSURFACE, WHEELCOLOR, (x + self.cart_w, y + self.cart_h / 2 + 4 * self.wheel_r),
                       2 * (self.wheel_r - 3.5))

    def pendulum(self):
        """
        @brief: Draws the pendulum.
        :return: none
        """
        x = self.x + SCREENSIZE[0] / 2

        pg.draw.line(DISPSURFACE, PENDULUMCOLOR, (x, self.y),
                     (x + self.length * sin(self.theta), self.y + self.length * cos(self.theta)), 10)

        pg.draw.circle(DISPSURFACE, BLOBCOLOR,
                       (x + self.length * sin(self.theta), self.y + self.length * cos(self.theta)), 25)
        pg.draw.circle(DISPSURFACE, BLACK,
                       (x + self.length * sin(self.theta), self.y + self.length * cos(self.theta)), 25, 4)

        # draw a circle centered at x, y
        pg.draw.circle(DISPSURFACE, RED, (x, self.y), 10)

    def update(self):
        """
        @brief: Updates the simulation.
        :return: none
        """
        self.x_d_dot = (self.force + self.pendulum_m * self.length * (self.theta_dot ** 2) * sin(
            self.theta) - self.theta_d_dot * cos(self.theta)) / (self.pendulum_m + self.cart_m)

        self.theta_d_dot = (self.g * sin(self.theta) - self.x_d_dot * cos(self.theta)) / self.length

        self.x_dot = self.x_dot + self.x_d_dot * self.dt
        self.x = self.x + self.x_dot * self.dt

        self.theta_dot = self.theta_dot + self.theta_d_dot * self.dt
        self.theta = self.theta + self.theta_dot * self.dt

    def main_loop(self):
        """
        @brief: Main loop of the simulation.
        :return: none
        """
        while not rospy.is_shutdown():
            CLOCK.tick(FPS)

            for event in pg.event.get():
                if event.type == pg.QUIT:
                    pg.quit()
                    sys.exit()
            self.update()

            if self.draw_counter == 0:
                self.background()
                self.draw()
                pg.display.update()

            self.draw_counter = (self.draw_counter + 1) % DRAW_RATE

            state = CurrentState()
            state.curr_x = self.x
            state.curr_x_dot = self.x_dot
            state.curr_x_dot_dot = self.x_d_dot
            state.curr_theta = self.theta
            state.curr_theta_dot = self.theta_dot
            state.curr_theta_dot_dot = self.theta_d_dot

            self.current_state_pub.publish(state)

    def draw(self):
        """
        @brief: Draws the simulation.
        :return:
        """
        self.cart()
        self.pendulum()


if __name__ == '__main__':
    """
    @brief: Main function.
    :return: none
    """
    rospy.init_node("InvertedPendulum", anonymous=True)
    rospy.loginfo("[InvertedPendulum]: Node Started")
    InvertedPendulum()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[StreamViewer]: Shutting down node")
