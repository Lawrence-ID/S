import math
import numpy as np


# 定义车辆类
class vehicle:
    def __init__(
            self,
            v_id,  # 车id
            v_speed,  # 车速度
            v_acc,  # 车加速度
            # v_angle_steer,  # 车方向盘偏航角
            # v_angle,  # 车中心偏离x轴的大小，用来画图
            v_x,  # 车后端中心x坐标
            v_y,  # 车后端中心y坐标
            v_vMax,  # 车的最大速度
    ):
        self.v_id = v_id
        self.v_speed = v_speed
        self.v_acc = v_acc
        self.v_yaw_speed = 0    # 偏航角速度
        # self.v_yaw_acc = v_yaw_acc
        self.v_angle_steer = 0
        self.v_angle = 0        # 车辆中轴线与x轴的夹角。
        self.v_angle_bt = 0
        self.v_x = v_x          # (v_x, v_y)代表车后端中心点坐标  建议将(v_x, v_y)改为：(x_r, y_r) 2022.10.25 px注
        self.v_y = v_y
        self.v_vMax = v_vMax
        self.v_laneId = 0
        self.v_length = 4.5
        self.v_width = 2
        #   v_x0车头的x坐标
        #   (v_x0, v_y0)代表车中心点坐标  建议将(v_x0, v_y0)改为：(x_c, y_c)  2022.10.25 px注
        self.v_x0 = v_x + self.v_length / 2 * math.cos(self.v_angle / 180 * math.pi)
        self.v_y0 = v_y + self.v_length / 2 * math.sin(self.v_angle / 180 * math.pi)
        #   xy1 代表什么呢
        #   (v_x1, v_y1)代表车前端点的坐标，建议将(v_x1, v_y1)改为：(x_f, y_f) 2022.10.25 px注
        self.v_x1 = self.v_x0 + self.v_length / 2 * math.cos(self.v_angle / 180 * math.pi)
        self.v_y1 = self.v_y0 + self.v_length / 2 * math.sin(self.v_angle / 180 * math.pi)
        self.v_yaw_speed_pre = 0  # 上一步偏航速度
        self.v_yaw_speed_var = 0  # 偏航角速度的变化率
        self.v_angle_steer_pre = 0  # 方向盘角度
        self.v_angle_steer_var = 0

    def control(self):  # 这里要返回一个如果是agent的话，返回当前agent的observation
        # 车辆前进函数
        t = 100  # 100ms 也就是0.1s
        self.v_angle_steer_var = (self.v_angle_steer - self.v_angle_steer_pre) / 0.1
        self.v_angle_steer_pre = self.v_angle_steer

        # 改变的是angle_steer, 即根据当前车速，获得了一个新的角速度，先计算新角速度，改变angle，然后再根据angle改变x和y的坐标，OK

        # 此处参考论文中公式(4)的最后一个公式, 0.5 = l_r / (l_r + l_f) 2022.10.25 px注
        self.v_angle_bt = math.atan(0.5 * math.tan(self.v_angle_steer / 180 * math.pi)) * 180 / math.pi  # 这里有问题
        self.v_yaw_speed_pre = self.v_yaw_speed

        # 此处```0.5 * self.v_length```代表l_r, 建议将v_length用l_r, l_f取代，与论文中一致 2022.10.25 px注
        self.v_yaw_speed = self.v_speed / (0.5 * self.v_length) * math.sin(
            self.v_angle_bt / 180 * math.pi) * 180 / math.pi

        self.v_yaw_speed_var = (self.v_yaw_speed - self.v_yaw_speed_pre) / 0.1
        self.v_angle += self.v_yaw_speed * t / 1000

        # 车纵向加速度取值范围（-3.5,+2）m/s^2，车正向速度最大为self.v_vMax，车速不能小于0
        if self.v_speed + self.v_acc * t / 1000 >= self.v_vMax:
            # 等效于0.1s内做匀速运动 2022.10.25 px注
            self.v_speed = self.v_vMax
            self.v_x0 = self.v_x0 + (self.v_speed * t / 1000) * math.cos(
                (self.v_angle + self.v_angle_bt) / 180 * math.pi)
            self.v_y0 = self.v_y0 + (self.v_speed * t / 1000) * math.sin(
                (self.v_angle + self.v_angle_bt) / 180 * math.pi)
        elif self.v_speed + self.v_acc * t / 1000 <= 0:
            self.v_speed = 0
        else:
            # 等效于0.1s内做匀加速运动 2022.10.25 px注
            self.v_x0 = self.v_x0 + (self.v_speed * t / 1000 + 0.5 * self.v_acc * math.pow((t / 1000), 2)) * math.cos(
                (self.v_angle + self.v_angle_bt) / 180 * math.pi)
            self.v_y0 = self.v_y0 + (self.v_speed * t / 1000 + 0.5 * self.v_acc * math.pow((t / 1000), 2)) * math.sin(
                (self.v_angle + self.v_angle_bt) / 180 * math.pi)
            self.v_speed = self.v_speed + self.v_acc * t / 1000

        # 检测一下laneID
        if self.v_y0 <= 3:
            self.v_laneId = 0
        elif 3 < self.v_y0 <= 6:
            self.v_laneId = 1
        else:
            self.v_laneId = 2
        # 重新计算后端中心点位置，用来画图
        # self.v_x0 = self.v_x + self.v_length / 2 * math.cos(
        #     self.v_angle / 180 * math.pi)
        # self.v_y0 = self.v_y + self.v_length / 2 * math.sin(
        #     self.v_angle / 180 * math.pi)

        # 根据中部中心点重新计算头部中心点(v_x1, v_y1)与尾部中心点(v_x, v_y) 2022.10.25 px注
        self.v_x = self.v_x0 - self.v_length / 2 * math.cos(
            self.v_angle / 180 * math.pi)
        self.v_y = self.v_y0 - self.v_length / 2 * math.sin(
            self.v_angle / 180 * math.pi)
        self.v_x1 = self.v_x0 + self.v_length / 2 * math.cos(
            self.v_angle / 180 * math.pi)
        self.v_y1 = self.v_y0 + self.v_length / 2 * math.sin(
            self.v_angle / 180 * math.pi)

        return self.v_yaw_speed_var, self.v_angle_steer_var
