import tkinter as tk
import time
import random
import sys
import matplotlib.pyplot as plt
import numpy as np
from PlotTools import PlotTools
# from kinematics import Kinematics
# sys.path.append('../../arm_kinematics')
from ArmKinematics import ArmKinematics
import math

class SliderFrame:
    def __init__(self, kinematics, container, arduino_msg, plot_tools, motor_name, grid_column_pos, grid_row_pos,
                 home_motor_val, min_motor_val, max_motor_val,
                servo_direction, pwm2angel):
        self.send_msg_arduino = arduino_msg == None
        # self.gripper_locked = False
        # self.enable_collision = False
        self.plot_tools = plot_tools
        self.kinematics = kinematics
        self.current_value = tk.IntVar()
        self.current_value.set(0)
        self.last_value = self.current_value.get()
        self.container = container
        self.arduino_msg = arduino_msg
        self.grid_column_pos = grid_column_pos
        self.grid_row_pos = grid_row_pos
        self.min_motor_val = min_motor_val
        self.max_motor_val = max_motor_val
        self.home_motor_val = home_motor_val
        self.motor_name = motor_name
        self.servo_direction = servo_direction
        self.pwm2angel = pwm2angel

        self.min_motor_angel = self.pwm_2_angel(min_motor_val)
        self.max_motor_angel = self.pwm_2_angel(max_motor_val)
        self.home_motor_angel = self.pwm_2_angel(home_motor_val)

        self.container.columnconfigure(grid_column_pos, weight=1)
        self.container.rowconfigure(grid_row_pos, weight=1)
        self.frame = tk.Frame(self.container)
        self.frame.grid(column=self.grid_column_pos, row=self.grid_row_pos)

        self.create_slider_params()

    def create_slider_params(self):
        self.joint_slider_label = tk.Label(self.frame, text=self.motor_name)
        self.joint_slider_label.grid(column=0, row=0)
        if(self.servo_direction==1):
            self.joint_slider = tk.Scale(self.frame, from_=self.min_motor_angel, to=self.max_motor_angel, orient='vertical',
                                     command=self.slider_changed,
                                     variable=self.current_value, length=200, takefocus=False)
        else:
            self.joint_slider = tk.Scale(self.frame, from_=-self.max_motor_angel,
                                         to=-self.min_motor_angel, orient='vertical',
                                         command=self.slider_changed,
                                         variable=self.current_value, length=200, takefocus=False)

        self.joint_slider.grid(column=0, row=1, columnspan=2)

    def pwm_2_angel(self, val):
        return (val-self.home_motor_val)*self.pwm2angel#* self.servo_direction

    def angel_2_pwm(self, val):
        val = val * self.servo_direction
        return self.home_motor_val+val/self.pwm2angel

    def set_value(self, value):
        self.joint_slider.set(value)

    def set_home_value(self):
        self.joint_slider.set(0)
        # self.send_msg(self.grid_column_pos, 0)

    def set_rand_value(self):
        min_val = round(self.min_motor_angel)
        max_val = round(self.max_motor_angel)
        value = random.randint(min(min_val, max_val), max(min_val, max_val))
        self.joint_slider.set(value)

    def set_min_value(self):
        self.joint_slider.set(self.min_motor_angel)
        # self.send_msg(self.grid_column_pos, self.min_motor_val)

    def set_max_value(self):
        self.joint_slider.set(self.max_motor_angel)
        # self.send_msg(self.grid_column_pos, self.max_motor_val)

    def slider_changed(self, event):
        # print('slider change')
        # print('value: ', self.current_value.get())
        # print('ind:', self.grid_column_pos)
        # print('theta len:', len(self.kinematics.theta))
        theta = self.kinematics.set_theta_by_motor_ind(self.grid_column_pos, self.current_value.get())
        pos, is_collision, H, T = self.kinematics.run_forward_kinematics()
        self.plot_tools.show_kinematics(theta, H, T)
        if(is_collision):
            print("--- COLLISION---")
        self.send_kinematic_angel()

    # def slider_changed(self, event):
    #     # print('slider change')
    #     # print('value: ', self.current_value.get())
    #     # print('ind:', self.grid_column_pos)
    #     # print('theta len:', len(self.kinematics.theta))
    #     if (self.grid_column_pos < len(self.kinematics.theta)):
    #         is_collision = False
    #         if self.enable_collision:
    #             is_collision = self.is_collision(self.grid_column_pos, self.current_value.get())
    #         theta, b_collision  = self.set_arm_pos(is_collision, self.grid_column_pos, self.current_value.get())
    #         # if not b_collision:
    #         #     self.send_msg(self.grid_column_pos, theta[self.grid_column_pos])
    #         # else:
    #         #     self.slider_set_home_value()
    #         #     print("-------COLLISION - GOING HOME-------")
    #         self.send_msg(self.grid_column_pos, theta[self.grid_column_pos])
    #         if (self.gripper_locked):
    #             self.send_msg(3, theta[3])
    #             self.send_msg(4, theta[4])
    #     else:
    #         self.send_msg(self.grid_column_pos, self.current_value.get())
    #     # if (self.gripper_locked):
    #     #     self.send_msg(3, theta[3])
    #     #     self.send_msg(4, theta[4])

    # def is_collision(self, motor_ind, theta):
    #     # print('is_collision Theta:', theta, motor_ind)
    #     self.kinematics.set_theta_by_motor_ind(theta, motor_ind)
    #     if(self.gripper_locked):
    #         self.kinematics.pos_gripper_down()
    #     return self.kinematics.check_collision_by_theta()

    def get_slider_angel(self):
        return self.current_value.get()

    def send_kinematic_angel(self):
        # msg = ('run:' + str(motor_ind) + ':' + str(self.angel_2_pwm(self.current_value.get())))
        # print(msg)
        # self.arduino_msg.sendToArduino(msg)
        # time.sleep(0.1)
        self.send_msg(self.grid_column_pos, self.kinematics.theta[self.grid_column_pos])

    def send_slider_angel(self):
        # msg = ('run:' + str(motor_ind) + ':' + str(self.angel_2_pwm(self.current_value.get())))
        # print(msg)
        # self.arduino_msg.sendToArduino(msg)
        # time.sleep(0.1)
        self.send_msg(self.grid_column_pos, self.current_value.get())

    def send_msg(self, ind, val):
        if self.arduino_msg != None:
            self.arduino_msg.send_msg_by_values(ind, self.angel_2_pwm(val))

    # def set_arm_pos(self, is_collision, ind, val):
    #     if not (is_collision):
    #         self.last_value = val
    #         self.kinematics.set_theta_by_motor_ind(val, ind)
    #         # print('ind: ', ind)
    #         # print('current val: ',val)
    #     else:
    #         last_val = self.last_value
    #         # print('ind: ', ind)
    #         # print('current val: ', val)
    #         # print('last val: ', last_val)
    #         self.slider_set_value(last_val)
    #         self.kinematics.set_theta_by_motor_ind(last_val, ind)
    #     theta = self.kinematics.theta
    #     if (self.gripper_locked):
    #         theta = self.kinematics.pos_gripper_down()
    #         # print('locked gripper: ', theta)
    #     H, T = self.kinematics.run_forward(theta)
    #     is_collision = False
    #     if self.enable_collision:
    #         is_collision = self.kinematics.check_collision(T)
    #     self.plot_tools.show_kinematics(theta, H, T)
    #     # return self.current_value.get()
    #
    #     return theta, is_collision

class SlidersTabUI:
    def __init__(self, tab_sliders, config, arduino_msg):
        self.tab_sliders = tab_sliders
        lst = config['servo_spec']['min']
        # self.a = config['kinematics']['link_size']
        self.num_of_joint = len(lst)
        self.joint_slider = []
        self.plot_tools = PlotTools(config['kinematics']['link_size'])
        self.kinematics = ArmKinematics(config)#Kinematics(config)
        for i in range(0, self.num_of_joint):
            name = 'Joint' + str(i+1)
            pwm2angel = config['servo_spec']['angel_max'][i] / (config['servo_spec']['gear_ratio'][i]*(config['servo_spec']['pwm_max'][i]-config['servo_spec']['pwm_min'][i]))
            slider = SliderFrame(self.kinematics, tab_sliders, arduino_msg, self.plot_tools, name, i, 0,
                                            config['servo_spec']['home'][i],
                                            config['servo_spec']['min'][i],
                                            config['servo_spec']['max'][i],
                                            config['servo_spec']['servo_direction'][i],
                                            pwm2angel)
            self.joint_slider.append(slider)

    def send_values_buttom(self):
        b1 = tk.Button(self.tab_sliders, text='Send msg',
                        command=lambda: self.send_sliders_values())
        b1.grid(row=1, columnspan=6)

    def sliders_set_values(self, values):
        for i in range(0, self.num_of_joint):
            print(values[i])
            self.joint_slider[i].set_value(values[i])

    def sliders_set_collision_flg(self, flg):
        self.kinematics.set_avoid_collision(flg)
        # for i in range(0, self.num_of_joint):
        #     self.joint_slider[i].enable_collision = flg

    def sliders_set_lock_gripper(self, flg):
        self.kinematics.set_gripper_locked(flg)
        # for i in range(0, self.num_of_joint):
        #     self.joint_slider[i].gripper_locked = flg

    def sliders_set_home_values(self):
        # self.sliders_set_lock_gripper(False)
        # self.sliders_set_collision_flg(True)
        for i in range(0, self.num_of_joint):
            self.joint_slider[i].set_home_value()

    def sliders_set_min_values(self):
        # self.sliders_set_lock_gripper(False)
        # self.sliders_set_collision_flg(True)
        self.joint_slider[0].set_home_value()
        for i in range(1, self.num_of_joint):
            self.joint_slider[i].set_min_value()
        # self.show_self_kinematics()

    def sliders_set_max_values(self):
        # self.sliders_set_lock_gripper(False)
        # self.sliders_set_collision_flg(True)
        self.joint_slider[0].set_home_value()
        for i in range(1, self.num_of_joint):
            self.joint_slider[i].set_max_value()
        # self.show_self_kinematics()

    def sliders_set_span_values(self):
        # self.sliders_set_lock_gripper(False)
        # self.sliders_set_collision_flg(True)
        self.joint_slider[0].set_home_value()
        self.joint_slider[1].set_value(70)
        self.joint_slider[2].set_home_value()
        self.joint_slider[3].set_home_value()
        self.joint_slider[4].set_home_value()
        self.joint_slider[5].set_home_value()

    def sliders_set_squeeze_values(self):
        # self.sliders_set_lock_gripper(False)
        # self.sliders_set_collision_flg(True)
        self.joint_slider[0].set_home_value()
        self.joint_slider[1].set_min_value()
        self.joint_slider[2].set_max_value()
        self.joint_slider[3].set_home_value()
        self.joint_slider[4].set_min_value()
        self.joint_slider[5].set_home_value()

    def sliders_set_rand_values(self):
        for i in range(0, self.num_of_joint):
            self.joint_slider[i].set_rand_value()
        # self.show_self_kinematics()

    def send_sliders_values(self):
        for i in range(0, self.num_of_joint):
            self.joint_slider[i].send_slider_pos()
            #time.sleep(0.1)

    def show_values_buttom(self):
        b1 = tk.Button(self.tab_sliders, text='Show',
                        command=lambda: self.show_values())
        b1.grid(row=1, columnspan=6)

    def show_values(self):
        for i in range(0, self.num_of_joint):
            print('Joint value ' + str(i) + ':' + str(self.joint_slider[i].current_value.get()))

    def draw_circle(self):
    # def sliders_set_squeeze_values(self):
        x0 = 0
        y0 = 0
        z0 = 0
        vx = []
        vy = []
        vz = []

        for counter in range(100):
            x = 3 * math.cos(2 * math.pi * counter / 100) + x0
            y = 3 * math.sin(2 * math.pi * counter / 100) + y0
            z = z0
            print([x,y, z])
            vx.append(x)
            vy.append(y)
            vz.append(z)
            theta1, theta2 = self.kinematics.inv_kinematics([x, y, z])
            theta1 = np.append(theta1, 0)
            theta2 = np.append(theta2, 0)
            print(theta1)
            # self.sliders_set_values(theta1)
            # time.sleep(1)
            # self.plot_tools.draw_circle(vx,vy,vz)


