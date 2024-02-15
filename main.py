import argparse
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from decimal import Decimal
import numpy as np
import pandas as pd
import sys
import os
import datetime
from tqdm import tqdm, trange
from GeometricUtils import *

class CarStateFrame:
    def __init__(self, id: int, center: Point, vx: float, vy: float, pre_vx = 0.0, pre_vy = 0.0, length = 5, width = 1.8):
        """
        以速度方向为指针,逆时针旋转,顶点依次为p1,p2,p3,p4
        p2          p5          p1    
        +----------*-----------+
        |                      |
        |     car  *-->(vx,vy) |
        |                      |
        +----------*-----------+
        p3         p6           p4
        """
        self.id = id
        self.center = center
        self.vx = vx
        self.vy = vy
        self.pre_vx = pre_vx
        self.pre_vy = pre_vy
        self.length = max(length, width)  # 单位:m
        self.width = min(length, width)
        self.angle = get_angle_by_vec(self.pre_vx, self.pre_vy)
        self.p5 = rotate_point(Point(self.center.x + (self.width / 2), self.center.y), self.center,  (math.pi / 2 + self.angle))
        self.p6 = rotate_point(Point(self.center.x + (self.width / 2), self.center.y), self.center,  (math.pi / 2 + self.angle + math.pi))
        self.p1 = rotate_point(Point(self.p5.x + self.length / 2, self.p5.y), self.p5, self.angle)
        self.p2 = rotate_point(Point(self.p5.x + self.length / 2, self.p5.y), self.p5, self.angle + math.pi)
        self.p3 = rotate_point(Point(self.p6.x + self.length / 2, self.p6.y), self.p6, self.angle + math.pi)
        self.p4 = rotate_point(Point(self.p6.x + self.length / 2, self.p6.y), self.p6, self.angle)

    def get_vertices(self):
        return [self.p1, self.p2, self.p3, self.p4]
    
    def get_vertices_array(self):
        return [[self.p1.x, self.p1.y], [self.p2.x, self.p2.y], [self.p3.x, self.p3.y], [self.p4.x, self.p4.y]]
    
    def get_polygon(self):
        return Polygon(self.get_vertices_array(), closed=True, fill=None, edgecolor='black')
    
class Car:
    def __init__(self, car_id: int, st: int, ed: int, tracks=[CarStateFrame]) -> None:
        self.car_id = car_id
        self.st = st
        self.ed = ed
        self.tracks = tracks
    def get_car_pos_by_time(self, time: int) -> CarStateFrame:
        if(time < self.st or time > self.ed):
            return None
        return self.tracks[time - self.st]

def get_line_car_cross_point(source: Point, end: Point, csf: CarStateFrame):
    [p1, p2, p3, p4] = csf.get_vertices()
    sensor_line = Line(source, end)

    cross_point1 = get_line_seg_cross_point(sensor_line, p1, p2)
    cross_point2 = get_line_seg_cross_point(sensor_line, p2, p3)
    cross_point3 = get_line_seg_cross_point(sensor_line, p3, p4)
    cross_point4 = get_line_seg_cross_point(sensor_line, p4, p1)
    
    if cross_point1 != None and cross_point2 != None:
        c1, c2 = cross_point1, cross_point2
    elif cross_point1 != None and cross_point3 != None:
        c1, c2 = cross_point1, cross_point3
    elif cross_point1 != None and cross_point4 != None:
        c1, c2 = cross_point1, cross_point4
    elif cross_point2 != None and cross_point3 != None:
        c1, c2 = cross_point2, cross_point3
    elif cross_point2 != None and cross_point4 != None:
        c1, c2 = cross_point2, cross_point4
    elif cross_point3 != None and cross_point4 != None:
        c1, c2 = cross_point3, cross_point4
    else:
        return None

    if get_distance(source, c1) < get_distance(source, c2):
        return c1
    else:
        return c2
    
def get_line_allcars_cross_points(args, source: Point, end: Point, csfs=[CarStateFrame]):
    ret_point = None
    min_dist = float("inf")
    for csf in csfs:
        cross_point = get_line_car_cross_point(source, end, csf)
        if args.save_mode == 2 or args.save_mode == 3:
            display_point(cross_point)
        if cross_point != None:
            dist = get_distance(source, cross_point)
            if dist < min_dist:
                ret_point = cross_point
                min_dist = dist
    return ret_point

def display_point(p, color='black'):
    if p != None and isinstance(p, Point):
        plt.plot(p.x, p.y, marker='*', color=color)

def create_tracks_from_xlsx():
    tra_list = pd.read_excel('traj.xlsx')
    car_list = []
    min_time = sys.maxsize
    max_time = 0
    # 读取轨迹真实数据
    for index, row in tra_list.iterrows():
        tracks = []
        car_id = int(''.join((ch if ch in '0123456789.-e' else ' ') for ch in row['ID']))
        start_time = row['start time']
        end_time = row['end time']

        min_time = min(start_time, min_time)
        max_time = max(end_time, max_time)

        newstr = ''.join((ch if ch in '0123456789.-e' else ' ') for ch in row['state'])
        tra = [float(i) for i in newstr.split()]
        # print(tra)

        pre_vx, pre_vy = tra[2]/10, tra[3]/10
        for i in range(int(len(tra) / 4)):
            # print(i, tra[0+4*i], tra[1+4*i], tra[2+4*i]/10, tra[3+4*i]/10, pre_vx, pre_vy)
            csf = CarStateFrame(id=car_id, center=Point(tra[0+4*i], tra[1+4*i]), vx=tra[2+4*i]/10, vy=tra[3+4*i]/10, pre_vx=pre_vx, pre_vy=pre_vy)
            tracks.append(csf)
            if i > 0 and (abs(tra[2+4*i] - 0) > 1e-2 or abs(tra[3+4*i] - 0) > 1e-2):
                pre_vx, pre_vy = tra[2+4*i]/10, tra[3+4*i]/10

        car_list.append(Car(car_id=car_id, st=start_time, ed=end_time, tracks=tracks))

    return min_time, max_time, car_list

def sensor1_scan(args, f_time: int, source: Point, csfs=[CarStateFrame], density=1):
    """在一帧内传感器扫描所有车辆,返回扫描得到的点
    args:
        f_time: 帧ID.
        source: 传感器位置.
        csfs: 当前帧的所有车辆状态.
        density: 传感器射线密度
    """
    end_points = []
    y_set = np.linspace(int(source.y), int(-source.y), abs(2 * int(source.y)) * density)
    for i in range(len(y_set)):
        p = Point(x=-source.x, y=y_set[i])
        if args.save_mode == 2 or args.save_mode == 3:
            display_point(p, 'blue')
        end_points.append(p)
    x_set = np.linspace(int(source.x), int(-source.x), abs(2 * int(source.x)) * density)
    for ii in range(len(x_set)):
        p = Point(x=x_set[ii], y=-source.y)
        if args.save_mode == 2 or args.save_mode == 3:
            display_point(p, 'blue')
        end_points.append(p)

    ans_points = []
    for i in range(len(end_points)):
        sensor_light = Line(source, end_points[i])
        ans_point = get_line_allcars_cross_points(args, source=sensor1_pos, end=end_points[i], csfs=csfs)
        if args.save_mode == 2 or args.save_mode == 3:
            plt.plot([source.x, end_points[i].x], [source.y, end_points[i].y], linewidth=0.5)
            display_point(ans_point, 'red')
        if ans_point != None:
            ans_points.append(ans_point)

    return ans_points

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--save_mode', type=int, default = 2, required=False, help="1: save result as txt, 2: save result as picture, 3: both")
    parser.add_argument('-o', '--out_path', type=str, default = './res', required=False, help="output path")
    args = parser.parse_args()

    if args.save_mode == 2 or args.save_mode == 3:
        fig1 = plt.figure(num=1, figsize=(5, 5))  # 确保正方形在屏幕上显示一致，固定figure的长宽相等
        axes1 = fig1.add_subplot(1, 1, 1)
        axes1.set_xlim([-30, 30])
        axes1.set_ylim([-40, 40])

    min_time, max_time, car_list = create_tracks_from_xlsx()

    # 显示所有车的轨迹
    # for i in range(len(car_list)):
    #     for j in range(len(car_list[i].tracks)):
    #         axes1.add_patch(car_list[i].tracks[j].get_polygon())

    # 对所有车的轨迹按帧划分
    cars_per_frame = [[] for i in range(max_time)]
    for i in range(len(car_list)):
        for j in range(len(car_list[i].tracks)):
            frame_time = j + car_list[i].st
            csf = car_list[i].tracks[j]
            cars_per_frame[frame_time].append(csf)

    # ===================显示某一帧的所有车辆,调试用====================
    # frame_time = 705
    # for i in range(len(cars_per_frame[frame_time])):
    #     # 显示所有车辆的中心点，打印车辆速度
    #     cars_per_frame[frame_time][i].center.display()
    #     print(cars_per_frame[frame_time][i].pre_vx, cars_per_frame[frame_time][i].pre_vy, cars_per_frame[frame_time][i].angle)

    #     axes1.add_patch(cars_per_frame[frame_time][i].get_polygon())

    # sensor1_pos = Point(-20, 20)
    # ans_points = sensor1_scan(f_time=frame_time, source=sensor1_pos, csfs=cars_per_frame[frame_time], density=2)
    # plt.show()
    # ============================================================
    
    # ==========================完整运行============================
    figure_save_path = args.out_path
    if not os.path.exists(figure_save_path):
        os.makedirs(figure_save_path)

    ans_points = [[] for i in range(max_time)]
    for frame_time in trange(min_time, max_time):
        # 左上角放置传感器1
        sensor1_pos = Point(-20, 20)
        ans_points_per_frame = sensor1_scan(args, frame_time, sensor1_pos, cars_per_frame[frame_time])
        ans_points[frame_time] = ans_points_per_frame

        if args.save_mode == 2 or args.save_mode == 3:
            # 显示某一帧的所有车辆
            for i in range(len(cars_per_frame[frame_time])):
                axes1.add_patch(cars_per_frame[frame_time][i].get_polygon())
            figname = str(frame_time) + '.jpg'
            plt.savefig(os.path.join(figure_save_path , figname))
            # plt.show()
            plt.cla()

    if args.save_mode == 1 or args.save_mode == 3:
        f = open(args.out_path + '/result.txt', 'a+')
        f.truncate(0)
        for i in range(len(ans_points)):
            print("Frame %d, Sensor %d" % (min_time + i, 1), file=f)
            points_per_frame = ans_points[i]
            for j in range(len(points_per_frame)):
                point = points_per_frame[j]
                print('(%.2f, %.2f)' % (point.x, point.y), end=' ', file=f)
            print('', file=f)
        f.close()
    # =============================================================