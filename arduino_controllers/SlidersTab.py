import tkinter as tk
import time
import random
import sys
# from kinematics import Kinematics
sys.path.append('../../kinematics')
from ArmKinematics import ArmKinematics

class SliderFrame:
    def __init__(self, kinematics, container, arduino_msg, motor_name, grid_column_pos, grid_row_pos,
                 home_motor_val, min_motor_val, max_motor_val,
                servo_direction, pwm2angel):
        self.kinematics = kinematics
        self.current_value = tk.IntVar()
        self.current_value.set(0)
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
        return (val-self.home_motor_val)*self.pwm2angel

    def angel_2_pwm(self, val):
        val = val * self.servo_direction
        return self.home_motor_val+val/self.pwm2angel

    def slider_set_value(self, value):
        self.joint_slider.set(value)

    def slider_set_home_value(self):
        self.joint_slider.set(0)
        # self.send_msg(self.grid_column_pos, 0)

    def slider_set_rand_value(self):
        value = random.randint(min(self.min_motor_angel, self.max_motor_angel), max(self.min_motor_angel, self.max_motor_angel))
        self.joint_slider.set(value)

    def slider_set_min_value(self):
        self.joint_slider.set(self.min_motor_angel)
        # self.send_msg(self.grid_column_pos, self.min_motor_val)

    def slider_set_max_value(self):
        self.joint_slider.set(self.max_motor_angel)
        # self.send_msg(self.grid_column_pos, self.max_motor_val)

    def slider_changed(self, event):
        # print(self.current_value.get())
        # print('slider chacnge')
        self.send_msg(self.grid_column_pos, self.angel_2_pwm(self.current_value.get()))
        # self.msg = ('run:' + str(self.grid_column_pos) + ':' + str(self.angel_2_pwm(self.current_value.get())))
        # time.sleep(0.1)
        # print(self.msg)
        # self.arduino_msg.sendToArduino(self.msg)

    def is_collision(self, theta, motor_ind):
        theta = abs(theta)
        #print('Theta:', theta, motor_ind)
        self.kinematics.set_theta_by_motor_ind(theta, motor_ind)
        return self.kinematics.check_collision_by_theta()

    def send_slider_pos(self, motor_ind):
        # msg = ('run:' + str(motor_ind) + ':' + str(self.angel_2_pwm(self.current_value.get())))
        # print(msg)
        # self.arduino_msg.sendToArduino(msg)
        # time.sleep(0.1)
        self.send_msg(motor_ind, self.angel_2_pwm(self.current_value.get()))

    def send_msg(self, ind, val):
        if not (self.is_collision(self.pwm_2_angel(val), ind)):
            self.arduino_msg.send_msg_by_values(ind, val)
        # self.msg = ('run:' + str(ind) + ':' + str(round(val)))
        # time.sleep(0.1)
        # print(self.msg)
        # self.arduino_msg.sendToArduino(self.msg)

class SlidersTabUI:
    def __init__(self, tab_sliders, config, arduino_msg):
        self.tab_sliders = tab_sliders
        lst = config['servo_spec']['min']
        self.num_of_joint = len(lst)
        self.joint_slider = []
        self.kinematics = ArmKinematics(config)#Kinematics(config)
        for i in range(0, self.num_of_joint):
            name = 'Joint' + str(i+1)
            pwm2angel = config['servo_spec']['angel_max'][i] / (config['servo_spec']['gear_ratio'][i]*(config['servo_spec']['pwm_max'][i]-config['servo_spec']['pwm_min'][i]))
            slider = SliderFrame(self.kinematics, tab_sliders, arduino_msg, name, i, 0,
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
            self.joint_slider[i].slider_set_value(values[i])

    def sliders_set_home_values(self):
        for i in range(0, self.num_of_joint):
            self.joint_slider[i].slider_set_home_value()

    def sliders_set_min_values(self):
        for i in range(0, self.num_of_joint):
            self.joint_slider[i].slider_set_min_value()

    def sliders_set_max_values(self):
        for i in range(0, self.num_of_joint):
            self.joint_slider[i].slider_set_max_value()

    def sliders_set_rand_values(self):
        for i in range(0, self.num_of_joint):
            self.joint_slider[i].slider_set_rand_value()

    def send_sliders_values(self):
        for i in range(0, self.num_of_joint):
            self.joint_slider[i].send_slider_pos(i)
            #time.sleep(0.1)

    def show_values_buttom(self):
        b1 = tk.Button(self.tab_sliders, text='Show',
                        command=lambda: self.show_values())
        b1.grid(row=1, columnspan=6)

    def show_values(self):
        for i in range(0, self.num_of_joint):
            print('Joint value ' + str(i) + ':' + str(self.joint_slider[i].current_value.get()))