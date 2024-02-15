#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2022-10-27 16:57X
# @Author : PengXiao
import math
import matplotlib.pyplot as plt

from shapely.geometry import Polygon

from GeometricUtils import Circle, get_distance, ratio_point, rotate_point, Line, get_circle_tangent_point, \
    get_line_cross_point, Point, test_draw_circles, test_draw_points
from Vehicle_model import vehicle


def get_sub_occupancy_polygon(c1: Circle, c2: Circle):
    """找到包含两圆的突多边形
        大圆为前, 小圆为后
        q0...q5 依次为多边形顺时针旋转的顶点
    """
    center_dis = get_distance(c1.center_point, c2.center_point)

    p2 = ratio_point(c2.center_point, c1.center_point, 1 + c1.radius / center_dis)
    p0 = rotate_point(p2, c1.center_point, -math.pi / 2)
    p1 = rotate_point(p2, c1.center_point, math.pi / 2)

    p5 = ratio_point(c1.center_point, c2.center_point, 1 + c2.radius / center_dis)
    p3 = rotate_point(p5, c2.center_point, math.pi / 2)
    p4 = rotate_point(p5, c2.center_point, -math.pi / 2)

    # l0...l5:各点切线
    l0 = Line(point1=p0, point2=rotate_point(c1.center_point, p0, math.pi / 2))
    l1 = Line(point1=p1, point2=rotate_point(c1.center_point, p1, math.pi / 2))
    l2 = Line(point1=p2, point2=rotate_point(c1.center_point, p2, math.pi / 2))
    l3 = Line(point1=p3, point2=rotate_point(c2.center_point, p3, math.pi / 2))
    l4 = Line(point1=p4, point2=rotate_point(c2.center_point, p4, math.pi / 2))
    l5 = Line(point1=p5, point2=rotate_point(c2.center_point, p5, math.pi / 2))

    # l6:大圆外切点连成的直线
    points = get_circle_tangent_point(c1, c2)
    l6 = Line(point1=points[2], point2=points[3])

    q0 = get_line_cross_point(l0, l2)
    q1 = get_line_cross_point(l3, l6)
    q2 = get_line_cross_point(l3, l5)
    q3 = get_line_cross_point(l4, l5)
    q4 = get_line_cross_point(l4, l6)
    q5 = get_line_cross_point(l1, l2)

    return [q0, q1, q2, q3, q4, q5]


def get_occupancy_polygon_by_circle(c1: Circle, c2: Circle, length: float, width: float):
    if c1.radius > c2.radius:
        c1, c2 = c2, c1

    q = get_sub_occupancy_polygon(c1, c2)

    # 得到centerLine与x轴正向的夹角
    center_line = Line(point1=c1.center_point, point2=c2.center_point)
    if center_line.B == 0:
        angle = math.pi / 2
    else:
        angle = math.atan(-center_line.A / center_line.B)
    # print("angle = %f" % angle)

    if c2.center_point.x == c1.center_point.x:
        if c2.center_point.y > c1.center_point.y:
            p0 = Point(q[0].x - 0.5 * width, q[0].y - 0.5 * length)
            p1 = Point(q[1].x - 0.5 * width, q[1].y - 0.5 * length)
            p2 = Point(q[2].x - 0.5 * width, q[2].y + 0.5 * length)
            p3 = Point(q[3].x + 0.5 * width, q[3].y + 0.5 * length)
            p4 = Point(q[4].x + 0.5 * width, q[4].y - 0.5 * length)
            p5 = Point(q[5].x + 0.5 * width, q[5].y - 0.5 * length)
        elif c2.center_point.y < c1.center_point.y:
            p0 = Point(q[0].x + 0.5 * width, q[0].y + 0.5 * length)
            p1 = Point(q[1].x + 0.5 * width, q[1].y + 0.5 * length)
            p2 = Point(q[2].x + 0.5 * width, q[2].y - 0.5 * length)
            p3 = Point(q[3].x - 0.5 * width, q[3].y - 0.5 * length)
            p4 = Point(q[4].x - 0.5 * width, q[4].y + 0.5 * length)
            p5 = Point(q[5].x - 0.5 * width, q[5].y + 0.5 * length)
        else:
            print("Wrong: The center points of Little Circle and Large Circle are the same!")
            exit(0)
    else:
        if c2.center_point.x > c1.center_point.x:
            p0 = rotate_point(Point(q[0].x - 0.5 * length, q[0].y + 0.5 * width), q[0], angle)
            p1 = rotate_point(Point(q[1].x - 0.5 * length, q[1].y + 0.5 * width), q[1], angle)
            p2 = rotate_point(Point(q[2].x + 0.5 * length, q[2].y + 0.5 * width), q[2], angle)
            p3 = rotate_point(Point(q[3].x + 0.5 * length, q[3].y - 0.5 * width), q[3], angle)
            p4 = rotate_point(Point(q[4].x - 0.5 * length, q[4].y - 0.5 * width), q[4], angle)
            p5 = rotate_point(Point(q[5].x - 0.5 * length, q[5].y - 0.5 * width), q[5], angle)
        else:
            p0 = rotate_point(Point(q[0].x + 0.5 * length, q[0].y - 0.5 * width), q[0], angle)
            p1 = rotate_point(Point(q[1].x + 0.5 * length, q[1].y - 0.5 * width), q[1], angle)
            p2 = rotate_point(Point(q[2].x - 0.5 * length, q[2].y - 0.5 * width), q[2], angle)
            p3 = rotate_point(Point(q[3].x - 0.5 * length, q[3].y + 0.5 * width), q[3], angle)
            p4 = rotate_point(Point(q[4].x + 0.5 * length, q[4].y + 0.5 * width), q[4], angle)
            p5 = rotate_point(Point(q[5].x + 0.5 * length, q[5].y + 0.5 * width), q[5], angle)

    return [p0, p1, p2, p3, p4, p5]


def get_occupancy_polygon_by_vehicle(veh: vehicle, a_max: float, N_e: int, dt: float, length: float, width: float):
    """外包函数，根据车辆对象获取此时车辆的
    args:
        veh: vehicle类的对象
        a_max: 最大加速度
        N_e: time step的个数
        dt: time step
        length: 应比车辆长度稍大
        width: 应比车辆宽度稍大
    return:
        返回多边形的6个点
    """
    x_little = veh.v_x0
    y_little = veh.v_y0
    v_speed = veh.v_speed
    v_speed_x = v_speed * math.cos((veh.v_angle + veh.v_angle_bt) / 180 * math.pi)
    v_speed_y = v_speed * math.sin((veh.v_angle + veh.v_angle_bt) / 180 * math.pi)
    r_little = v_speed * N_e * dt

    x_large = x_little + v_speed_x * N_e * dt
    y_large = y_little + v_speed_y * N_e * dt
    r_large = r_little + 0.5 * a_max * (N_e * dt) * (N_e * dt)

    c1 = Circle(center_point=Point(x_little, y_little), radius=r_little)
    c2 = Circle(center_point=Point(x_large, y_large,), radius=r_large)

    polygon_points = get_occupancy_polygon_by_circle(c1, c2, length, width)

    location = []
    for point in polygon_points:
        location.append((point.x, point.y))

    return location


if __name__ == '__main__':
    c0 = Circle(Point(0, 0), 1)

    for center_point in [Point(0, 4), Point(4, 4), Point(4, 0), Point(4, -4), Point(0, -4), Point(-4, -4), Point(-4, 0),
                         Point(-4, 4)]:
        ct = Circle(center_point=center_point, radius=2)
        test_draw_circles(c0, ct)

        veh_l = 2 * get_distance(c0.center_point, ct.center_point)
        veh_w = 2 * max(c0.radius, ct.radius)

        sub_points = get_sub_occupancy_polygon(c0, ct)
        test_draw_points(sub_points, color='r')

        # 测试函数
        polygon_points = get_occupancy_polygon_by_circle(c0, ct, veh_l, veh_w)
        test_draw_points(polygon_points, color='b')

        location = []
        for point in polygon_points:
            location.append((point.x, point.y))

        # polygon = Polygon(location).convex_hull

        plt.xlim(-20, 20)
        plt.ylim(-20, 20)
        plt.show()