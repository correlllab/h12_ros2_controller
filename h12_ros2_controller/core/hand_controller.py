import time
import numpy as np
import tkinter as tk

from h12_ros2_controller.core.channel_interface import HandSubscriber, HandPublisher

class HandController:
    def __init__(self):
        # hand topics subscriber and publisher
        self.hand_subscriber = HandSubscriber()
        self.hand_publisher = HandPublisher()
        self.dt =  self.hand_publisher.dt

    @property
    def q_right(self):
        '''
        Get the right hand angles.
        0 for close, 1 for open
        #index 0:pinky
        #index 1:ring
        #index 2:middle
        #index 3:index
        #index 4:thumb
        #index 5:thumb angle
        '''
        return self.hand_subscriber.q_right

    @property
    def q_left(self):
        '''
        Get the left hand angles.
        0 for close, 1 for open
        #index 0:pinky
        #index 1:ring
        #index 2:middle
        #index 3:index
        #index 4:thumb
        #index 5:thumb angle
        '''
        return self.hand_subscriber.q_left

    def ctrl_right(self, right_arr):
        '''
        Control the right hand with right angles.
        0 for close, 1 for open
        #index 0:pinky
        #index 1:ring
        #index 2:middle
        #index 3:index
        #index 4:thumb
        #index 5:thumb angle
        '''
        assert(len(right_arr) == 6), 'Right angles must be of length 6.'
        self.hand_publisher.q[:6] = right_arr

    def ctrl_left(self, left_arr):
        '''
        Control the left hand with left angles.
        0 for close, 1 for open
        #index 0:pinky
        #index 1:ring
        #index 2:middle
        #index 3:index
        #index 4:thumb
        #index 5:thumb angle
        '''
        assert(len(left_arr) == 6), 'Left angles must be of length 6.'
        self.hand_publisher.q[6:] = left_arr

    def ctrl(self, right_arr, left_arr):
        '''
        Control the hand with right and left angles.
        0 for close, 1 for open
        #index 0:pinky
        #index 1:ring
        #index 2:middle
        #index 3:index
        #index 4:thumb
        #index 5:thumb angle
        '''
        assert(len(right_arr) == 6 and len(left_arr) == 6), 'Right and left angles must be of length 6.'
        self.ctrl_right(right_arr)
        self.ctrl_left(left_arr)
