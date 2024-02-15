#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2024-2-14
# @Author : PengXiao

import math
from decimal import Decimal
import matplotlib.pyplot as plt


class Point(object):
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
    def display(self):
        print('(%.2f, %.2f)' % (self.x, self.y), end=' ')
    def get_val(self):
        return [self.x, self.y]


class Circle:
    def __init__(self, center_point: Point, radius: float):
        self.center_point = center_point
        self.radius = radius


class Line:
    def __init__(self, point1=None, point2=None, k=None, b=None):
        try:
            if isinstance(point1, Point) and isinstance(point2, Point):
                # print("Line has been created by two Points.")
                self.A = point1.y - point2.y
                self.B = point2.x - point1.x
                self.C = point1.x * point2.y - point2.x * point1.y
            elif k is not None and b is not None:
                # print("Line has been created by k and b.")
                self.A = k
                self.B = -1
                self.C = b
            else:
                raise Exception("Line creator loss args!")
        except Exception as e:
            print(e)
            exit(0)


def get_distance(a: Point, b: Point):
    return math.sqrt(math.pow(a.x - b.x, 2) + math.pow(a.y - b.y, 2))


def get_angle(ab: float, ac: float, bc: float):
    """
    三角形ABC中，根据点A,B,C的距离ab,ac,bc获取”角BAC“的角度(绝对值)
    """
    return math.acos((ab * ab + ac * ac - bc * bc) / (2.0 * ab * ac))

def get_angle_by_vec(dx1,dy1):
    if abs(dx1 - 0) < 1e-3 and abs(dy1 - 0) < 1e-3:
        print("error")
    angle1 = math.atan2(-dy1, dx1)
    angle1 = -int(angle1 * 180 / math.pi)
    if angle1 < 0:
        angle1 = 360 + angle1
    return angle1 * math.pi / 180


def get_line_cross_point(l1: Line, l2: Line):
    a0, b0, c0 = l1.A, l1.B, l1.C
    a1, b1, c1 = l2.A, l2.B, l2.C
    D = a0 * b1 - a1 * b0
    if D == 0:
        return None
    x = (b0 * c1 - b1 * c0) / D
    y = (a1 * c0 - a0 * c1) / D
    return Point(x, y)

def get_line_seg_cross_point(l: Line, p1: Point, p2: Point):
    cross_point = get_line_cross_point(l, Line(p1, p2))

    if cross_point == None:
        return None
    else:
        if ((Decimal(min(p1.x, p2.x)) <= Decimal(cross_point.x)) or (abs(min(p1.x, p2.x) - cross_point.x) < 1e-8)) and ((Decimal(max(p1.x, p2.x)) >= Decimal(cross_point.x)) or (abs(max(p1.x, p2.x) - cross_point.x) < 1e-8)) and \
           ((Decimal(min(p1.y, p2.y)) <= Decimal(cross_point.y)) or (abs(min(p1.y, p2.y) - cross_point.y) < 1e-8)) and ((Decimal(max(p1.y, p2.y)) > Decimal(cross_point.y)) or (abs(max(p1.y, p2.y) - cross_point.y) < 1e-8)):
            return cross_point
        else:
            return None

def ratio_point(start_point: Point, end_point: Point, ratio: float):
    """在起始点到终止点之间的直线上，找到比率为ratio的点
    args:
        start_point: 起始点.
        center: 终止点.
        ratio: 比率.
    """
    if start_point is None:
        start_point = Point(0, 0)
    x = end_point.x - start_point.x
    y = end_point.y - start_point.y
    return Point(x * ratio + start_point.x, y * ratio + start_point.y)


def rotate_point(point: Point, center: Point, angle: float):
    """将点point以点center为中心点，旋转angle角度
    args:
        point: 原始点.
        center: 中心点.
        angle: 旋转角度.
    """
    if center is None:
        center = Point(0, 0)
    x = point.x - center.x
    y = point.y - center.y
    ret = Point(0, 0)
    ret.x = (x * math.cos(angle) - y * math.sin(angle)) + center.x
    ret.y = (x * math.sin(angle) + y * math.cos(angle)) + center.y
    return ret


def get_circle_tangent_point_not_intersect(c1: Circle, c2: Circle):
    center_dis = get_distance(c1.center_point, c2.center_point)
    # 内切线交点
    p0 = Point(0, 0)
    p0.x = (c1.center_point.x * c2.radius + c2.center_point.x * c1.radius) / (c1.radius + c2.radius)
    p0.y = (c1.center_point.y * c2.radius + c2.center_point.y * c1.radius) / (c1.radius + c2.radius)

    l1 = center_dis * c1.radius / (c1.radius + c2.radius)
    l2 = center_dis * c2.radius / (c1.radius + c2.radius)
    angle = math.acos(c1.radius / l1)

    # p1,p2:圆心连线与c1,c2的交点
    p1 = ratio_point(c1.center_point, p0, c1.radius / l1)
    p2 = ratio_point(p0, c2.center_point, (l2 - c2.radius) / l2)

    # 四个内切点
    p3 = rotate_point(p1, c1.center_point, angle)
    p4 = rotate_point(p1, c1.center_point, -angle)
    p5 = rotate_point(p2, c2.center_point, angle)
    p6 = rotate_point(p2, c2.center_point, -angle)

    if c1.radius == c2.radius:
        angle = math.pi / 2
        q1 = ratio_point(c1.center_point, c2.center_point, c1.radius / center_dis)
        q2 = ratio_point(c1.center_point, c2.center_point, (center_dis - c2.radius) / center_dis)
    else:
        if c1.radius > c2.radius:
            c1, c2 = c2, c1

        # 外切线交点
        q0 = Point(0, 0)
        q0.x = (c1.center_point.x * c2.radius - c2.center_point.x * c1.radius) / (c2.radius - c1.radius)
        q0.y = (c1.center_point.y * c2.radius - c2.center_point.y * c1.radius) / (c2.radius - c1.radius)

        l1 = center_dis * c1.radius / (c2.radius - c1.radius)
        l2 = center_dis * c2.radius / (c2.radius - c1.radius)
        angle = math.acos(c1.radius / l1)

        q1 = ratio_point(c1.center_point, q0, c1.radius / l1)
        q2 = ratio_point(c2.center_point, q0, c2.radius / l2)

    # 四个外切点
    q3 = rotate_point(q1, c1.center_point, angle)
    q4 = rotate_point(q1, c1.center_point, -angle)
    q5 = rotate_point(q2, c2.center_point, angle)
    q6 = rotate_point(q2, c2.center_point, -angle)

    return [q3, q4, q5, q6, p3, p4, p5, p6]


def get_circle_tangent_point_intersect(c1: Circle, c2: Circle):
    center_dis = get_distance(c1.center_point, c2.center_point)
    # p0:圆心连线与c1的交点
    p0 = ratio_point(c1.center_point, c2.center_point, c1.radius / center_dis)

    # p1:圆心连线与c2的交点
    p1 = ratio_point(c1.center_point, c2.center_point, (center_dis - c2.radius) / center_dis)

    angle_r1 = get_angle(c1.radius, center_dis, c2.radius)
    angle_r2 = get_angle(c2.radius, center_dis, c1.radius)
    angle = math.acos(abs(c1.radius - c2.radius) / center_dis)

    # p2,p3:两圆的交点
    p2 = rotate_point(p0, c1.center_point, angle_r1)
    p3 = rotate_point(p1, c2.center_point, angle_r2)

    if c1.radius <= c2.radius:
        p4 = rotate_point(p0, c1.center_point, angle)
        p5 = rotate_point(p0, c1.center_point, -angle)
        p6 = rotate_point(p1, c2.center_point, angle - math.pi)
        p7 = rotate_point(p1, c2.center_point, math.pi - angle)
    else:
        p4 = rotate_point(p0, c1.center_point, math.pi - angle)
        p5 = rotate_point(p0, c1.center_point, angle - math.pi)
        p6 = rotate_point(p1, c2.center_point, -angle)
        p7 = rotate_point(p1, c2.center_point, angle)
    return [p4, p5, p6, p7]


def get_circle_tangent_point(c1: Circle, c2: Circle):
    """外包函数，以便直接调用
    找到两圆的切点
        若两圆外离, 返回8个切点[p0, p1, p2, p3, p4, p5, p6, p7]
                  [p0,...,p3]为外切点,p0, p1为小圆外切点, p2, p3为大圆外切点, p0,p2同侧构成一条外切线, p1,p3同侧构成一条外切线
                  [p4,...,p7]为内切点,p4, p5为小圆内切点, p6, p7为大圆内切点, p4,p6同侧构成一条内切线, p5,p7同侧构成一条外切线
        若两圆相交, 返回4个切点[p0, p1, p2, p3], p0,p1为小圆切点, p2,p3为大圆切点, p0,p2同侧构成一条外切线, p1,p3同侧构成一条外切线
        若两圆内含, 返回None
    """
    center_dis = get_distance(c1.center_point, c2.center_point)
    if c1.radius + c2.radius < center_dis:
        return get_circle_tangent_point_not_intersect(c1=c1, c2=c2)
    elif abs(c1.radius - c2.radius) < center_dis <= c1.radius + c2.radius:
        return get_circle_tangent_point_intersect(c1=c1, c2=c2)
    else:
        return None


def test_draw_circles(c1_show: Circle, c2_show: Circle):
    plt.figure(figsize=(7, 7))

    circle1 = plt.Circle((c1_show.center_point.x, c1_show.center_point.y), c1_show.radius, color='r', fill=False)
    circle2 = plt.Circle((c2_show.center_point.x, c2_show.center_point.y), c2_show.radius, color='r', fill=False)
    plt.gcf().gca().add_artist(circle1)
    plt.gcf().gca().add_artist(circle2)

    plt.axis('equal')


def test_draw_points(points_show, color):
    if points_show is None:
        return
    i = 0
    for point in points_show:
        # print("p%d: (%f, %f)" % (i, point.x, point.y))
        i += 1
        plt.plot(point.x, point.y, 'o', color=color)
        plt.text(point.x, point.y, str(i))


if __name__ == '__main__':
    c0 = Circle(Point(0, 0), 1)
    c1 = Circle(Point(4, 0), 2)
    test_draw_circles(c0, c1)

    points = get_circle_tangent_point(c0, c1)
    test_draw_points(points, color='r')

    plt.xlim(-10, 10)
    plt.ylim(-10, 10)
    plt.show()

