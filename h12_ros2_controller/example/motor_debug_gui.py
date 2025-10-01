import time
import numpy as np
import tkinter as tk

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.core.channel_interface import CommandPublisher, setup_gains_mj, setup_gains_real
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS

class MotorDebugGUI:
    '''GUI interface for motor debugging'''

    def __init__(self, init_q, robot_model, command_publisher, control_idx=14):
        self.robot_model = robot_model
        self.command_publisher = command_publisher
        self.control_idx = control_idx

        self._setup_window()
        self._setup_controls(init_q)
        self._setup_display()
        self._setup_buttons()

    def _setup_window(self):
        '''Create main window'''
        self.root = tk.Tk()
        self.root.title(f'Motor Debug - Joint {self.control_idx} ({BODY_JOINTS[self.control_idx]})')
        self.root.geometry('600x300')

    def _setup_controls(self, init_q):
        '''Setup motor index and position controls'''
        # motor index selection
        index_frame = tk.Frame(self.root)
        index_frame.pack(pady=10)
        tk.Label(index_frame, text='Motor Index:').pack(side=tk.LEFT)

        self.index_var = tk.StringVar(value=str(self.control_idx))
        self.index_entry = tk.Entry(index_frame, textvariable=self.index_var, width=5)
        self.index_entry.pack(side=tk.LEFT, padx=5)

        # position slider
        slider_frame = tk.Frame(self.root)
        slider_frame.pack(pady=10)

        self.slider = tk.Scale(slider_frame,
                               label=f'Joint {self.control_idx} {BODY_JOINTS[self.control_idx]}',
                               from_=-3.0, to=3.0,
                               resolution=0.01, orient=tk.HORIZONTAL, length=500)
        self.slider.pack()
        self.slider.set(init_q[self.control_idx])

        # store reference to index frame for button setup
        self.index_frame = index_frame

    def _setup_display(self):
        '''Setup information display labels'''
        info_frame = tk.Frame(self.root)
        info_frame.pack(pady=10)

        self.position_label = tk.Label(info_frame, text='Current Position: 0.000')
        self.position_label.pack()

        self.error_label = tk.Label(info_frame, text='Position Error: 0.000')
        self.error_label.pack()

        self.velocity_label = tk.Label(info_frame, text='Current Velocity: 0.000')
        self.velocity_label.pack()

    def _setup_buttons(self):
        '''Setup control buttons'''
        # update button
        update_button = tk.Button(self.index_frame, text='Update', command=self.update_motor_index)
        update_button.pack(side=tk.LEFT, padx=5)

        # emergency stop button
        estop_button = tk.Button(self.root, text='EMERGENCY STOP',
                                command=self.emergency_stop,
                                bg='red', fg='white',
                                font=('Arial', 12, 'bold'))
        estop_button.pack(pady=10)

    def update_motor_index(self):
        '''Callback to update the controlled motor index'''
        try:
            new_idx = int(self.index_var.get())
            if 0 <= new_idx < 27:
                self.control_idx = new_idx
                self.root.title(f'Motor Debug - Joint {new_idx} {BODY_JOINTS[new_idx]}')
                self.slider.config(label=f'Joint {new_idx} {BODY_JOINTS[new_idx]}')
                self.slider.set(self.robot_model.state['q'][self.control_idx])
        except ValueError:
            pass

    def emergency_stop(self):
        '''Callback for emergency stop'''
        self.command_publisher.estop()
        print('EMERGENCY STOP ACTIVATED!')

    def update_display(self, position_error):
        '''Update the information display'''
        self.position_label.config(text=f'Current Position: {self.robot_model.state["q"][self.control_idx]:.3f}')
        self.error_label.config(text=f'Position Error: {position_error:.3f}')
        self.velocity_label.config(text=f'Current Velocity: {self.robot_model.state["dq"][self.control_idx]:.3f}')

    def get_target_position(self):
        '''Get the target position from slider'''
        return self.slider.get()

    def update(self):
        '''Update the GUI (call from main loop)'''
        self.root.update()

    def shutdown(self):
        '''Shutdown GUI'''
        try:
            self.root.destroy()
        except:
            pass



def main_loop(gui, robot_model, command_publisher):
    '''main control loop for motor debugging'''
    try:
        while True:
            gui.update()

            # sync robot state
            robot_model.update_kinematics()

            # update target position for the controlled motor
            target_position = gui.get_target_position()
            command_publisher.q[gui.control_idx] = target_position

            # update information display
            position_error = target_position - robot_model.state['q'][gui.control_idx]
            gui.update_display(position_error)

            time.sleep(0.01)

    except KeyboardInterrupt:
        print('Keyboard interrupt received.')
    except tk.TclError:
        print('GUI window closed.')
    except Exception as e:
        print(f'Exception occurred: {e}')

def main():
    ChannelFactoryInitialize()

    # initialize robot model and command publisher
    robot_model = RobotModel('./assets/h1_2/h1_2.urdf')
    robot_model.init_subscriber()
    command_publisher = CommandPublisher()

    # wait for initial state sync
    time.sleep(1.0)
    robot_model.update_kinematics()

    # setup motor gains
    setup_gains_mj(command_publisher)
    # setup_gains_real(command_publisher)

    # enable all motors at initial positions
    motor_ids = list(range(27))
    init_q = robot_model.state['q']
    command_publisher.enable_motor(motor_ids, init_q)
    command_publisher.start_publisher()

    # create GUI interface
    gui = MotorDebugGUI(init_q, robot_model, command_publisher)

    # print startup messages
    print('Motor debug interface started.')
    print('Use the slider to control the selected motor.')
    print('Use the index entry to change which motor to control (0-26).')
    print('Press the Emergency Stop button to stop all motors.')
    print('Close the window or press Ctrl+C to shutdown.')

    # run the main control loop
    main_loop(gui, robot_model, command_publisher)

    print('Shutting down...')
    command_publisher.shutdown()
    robot_model.shutdown()
    gui.shutdown()

if __name__ == '__main__':
    main()
