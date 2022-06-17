import yaml
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

class Kinematics():
    def __init__(self, config):
        a = config['kinematics']['link_size']
        theta0 = config['kinematics']['theta0']
        self.theta0 = theta0
        self.a = a
        self.z_limit = config['kinematics']['z_limit']
        self.d_h_table = np.array([[np.deg2rad(theta0[0]), np.deg2rad(90), 0, a[0]],
                              [np.deg2rad(theta0[1]), np.deg2rad(0), a[1], 0],
                              [np.deg2rad(theta0[2]+90), np.deg2rad(90), a[3], 0],
                              [np.deg2rad(theta0[3]), np.deg2rad(-90), a[4], a[2]+a[5]],
                              [np.deg2rad(theta0[4]-90), np.deg2rad(0), 0, 0],
                              [np.deg2rad(0), np.deg2rad(0), a[6], 0]])

        self.theta = [0, 0, 0, 0, 0, 0]
        self.H = []
        self.T = np.eye(4)

    def rot_tran_matrix(self, theta, alpha, r, d):
        return np.array([[np.cos(theta), -np.sin(theta) * np.cos(alpha),
                                np.sin(theta) * np.sin(alpha),
                                r * np.cos(theta)],
                               [np.sin(theta), np.cos(theta) * np.cos(alpha),
                                -np.cos(theta) * np.sin(alpha),
                                r * np.sin(theta)],
                               [0, np.sin(alpha), np.cos(alpha), d],
                               [0, 0, 0, 1]])

    def run_forward(self):
        theta = self.theta
        H = []
        T = np.eye(4)
        for i in range(0, len(self.d_h_table)):
            res = self.rot_tran_matrix(self.d_h_table[i, 0]+np.deg2rad(theta[i]), self.d_h_table[i, 1], self.d_h_table[i, 2], self.d_h_table[i, 3])
            T = T @ res
            H.append(res)

        self.H = H
        self.T = T

    def check_collision(self):
        if(self.T[2,3]< self.z_limit):
            print('Collision: ', self.T)
            return True
        return False

    def set_theta_by_motor_ind(self, theta, motor_ind):
        self.theta[motor_ind] = int(theta)
        print(self.theta)

    def set_theta(self, theta):
        self.theta = theta

    def check_collision_by_theta(self):
        self.run_forward()
        print(self.T)
        return self.check_collision()

    def show_kinematics(self):
        ma = max(self.a)
        x = []
        y = []
        z = []
        coords = []
        v = [0, 0, 0, 1]
        x.append(v[0])
        y.append(v[1])
        z.append(v[2])
        T_ = np.eye(4)
        for h in self.H:
            T_ = T_ @ h
            res = T_ @ v
            x.append(res[0])
            y.append(res[1])
            z.append(res[2])
            coords.append(res)

        res = self.T @ v
        x_ = res[0]
        y_ = res[1]
        z_ = res[2]
        fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
        ax.plot(x, y, z, c='r', marker='o')
        ax.plot(x, y, z, c='g')
        ax.plot(x_, y_, z_, c='b', marker='o')
        ax.set_xlim(-ma, ma)
        ax.set_ylim(-ma, ma)
        ax.set_zlim(0, ma)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        plt.show()

if __name__ == "__main__":
    with open("config.yml", 'r') as conf_file:
        config = yaml.load(conf_file, Loader=yaml.FullLoader)
    ind = 1
    show = True
    theta = [0, 0, 0, 0, 0, 0]
    k = Kinematics(config)
    for t in range(0, 180, 10):
        theta[ind] = t
        print(theta)
        k.set_theta(theta)
        b = k.check_collision_by_theta()
        # k.show_kinematics()



