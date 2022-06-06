import tkinter as tk
import time

class SliderFrame:
    def __init__(self, container, arduino_msg, motor_name, grid_column_pos, grid_row_pos,
                 home_motor_val, min_motor_val, max_motor_val):
        self.current_value = tk.IntVar()
        self.current_value.set(home_motor_val)
        self.container = container
        self.arduino_msg = arduino_msg
        self.grid_column_pos = grid_column_pos
        self.grid_row_pos = grid_row_pos
        self.min_motor_val = min_motor_val
        self.max_motor_val = max_motor_val
        self.home_motor_val = home_motor_val
        self.motor_name = motor_name

        self.container.columnconfigure(grid_column_pos, weight=1)
        self.container.rowconfigure(grid_row_pos, weight=1)
        self.frame = tk.Frame(self.container)
        self.frame.grid(column=self.grid_column_pos, row=self.grid_row_pos)

        self.create_slider_params()

    def create_slider_params(self):
        self.joint_slider_label = tk.Label(self.frame, text=self.motor_name)
        self.joint_slider_label.grid(column=0, row=0)
        self.joint_slider = tk.Scale(self.frame, from_=self.min_motor_val, to=self.max_motor_val, orient='vertical',
                                     command=self.slider_changed,
                                     variable=self.current_value, length=200, takefocus=False)
        self.joint_slider.grid(column=0, row=1, columnspan=2)

    def slider_set_value(self, value):
        self.joint_slider.set(value)

    def slider_set_home_value(self):
        self.joint_slider.set(self.home_motor_val)

    def slider_set_min_value(self):
        self.joint_slider.set(self.min_motor_val)

    def slider_set_max_value(self):
        self.joint_slider.set(self.max_motor_val)

    def slider_changed(self, event):
        self.msg = ('run:' + str(self.grid_column_pos) + ':' + str(self.current_value.get()))
        time.sleep(0.1)
        print(self.msg)
        self.arduino_msg.sendToArduino(self.msg)

    def send_slider_pos(self, motor_ind):
        msg = ('run:' + str(motor_ind) + ':' + str(self.current_value.get()))
        print(msg)
        self.arduino_msg.sendToArduino(msg)
        time.sleep(0.1)

class SlidersTabUI:
    def __init__(self, tab_sliders, config, arduino_msg):
        self.tab_sliders = tab_sliders
        lst = config['servo_joint_limit']['min']
        self.num_of_joint = len(lst)
        self.joint_slider = []
        for i in range(0, self.num_of_joint):
            name = 'Joint' + str(i+1)
            slider = SliderFrame(tab_sliders, arduino_msg, name, i, 0,
                                            config['servo_joint_limit']['home'][i],
                                            config['servo_joint_limit']['min'][i],
                                            config['servo_joint_limit']['max'][i])
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

