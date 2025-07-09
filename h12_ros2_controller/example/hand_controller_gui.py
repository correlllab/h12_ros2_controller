import time
import numpy as np
import tkinter as tk

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.hand_controller import HandController

def main():
    ChannelFactoryInitialize()
    hand_controller = HandController()

    root = tk.Tk()
    root.title('Hand Controller')
    root.geometry('600x400')

    # pack sliders side by side
    left_frame = tk.Frame(root)
    right_frame = tk.Frame(root)
    left_frame.pack(side=tk.LEFT, padx=10, pady=10)
    right_frame.pack(side=tk.RIGHT, padx=10, pady=10)

    # left hand sliders
    slider_l0 = tk.Scale(left_frame, label="Left Pinky",
                         from_=0.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_l1 = tk.Scale(left_frame, label="Left Ring",
                         from_=0.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_l2 = tk.Scale(left_frame, label="Left Middle",
                         from_=0.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_l3 = tk.Scale(left_frame, label="Left Index",
                         from_=0.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_l4 = tk.Scale(left_frame, label="Left Thumb",
                         from_=0.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_l5 = tk.Scale(left_frame, label="Left Angle",
                         from_=0.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_l0.pack(in_=left_frame, pady=5)
    slider_l1.pack(in_=left_frame, pady=5)
    slider_l2.pack(in_=left_frame, pady=5)
    slider_l3.pack(in_=left_frame, pady=5)
    slider_l4.pack(in_=left_frame, pady=5)
    slider_l5.pack(in_=left_frame, pady=5)

    # right hand sliders
    slider_r0 = tk.Scale(right_frame, label="Right Pinky",
                         from_=0.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_r1 = tk.Scale(right_frame, label="Right Ring",
                         from_=0.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_r2 = tk.Scale(right_frame, label="Right Middle",
                         from_=0.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_r3 = tk.Scale(right_frame, label="Right Index",
                         from_=0.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_r4 = tk.Scale(right_frame, label="Right Thumb",
                         from_=0.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_r5 = tk.Scale(right_frame, label="Right Angle",
                         from_=0.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_r0.pack(in_=right_frame, pady=5)
    slider_r1.pack(in_=right_frame, pady=5)
    slider_r2.pack(in_=right_frame, pady=5)
    slider_r3.pack(in_=right_frame, pady=5)
    slider_r4.pack(in_=right_frame, pady=5)
    slider_r5.pack(in_=right_frame, pady=5)

    root.update()

    while True:
        start_time = time.time()
        root.update()
        # get right and left states
        left_arr = np.array([slider_l0.get(), slider_l1.get(), slider_l2.get(),
                             slider_l3.get(), slider_l4.get(), slider_l5.get()])
        right_arr = np.array([slider_r0.get(), slider_r1.get(), slider_r2.get(),
                              slider_r3.get(), slider_r4.get(), slider_r5.get()])
        # control the hand
        hand_controller.ctrl(right_arr, left_arr)

        time.sleep(max(0, hand_controller.dt - (time.time() - start_time)))

if __name__ == '__main__':
    main()
