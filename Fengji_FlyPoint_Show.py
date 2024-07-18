import sys
import numpy as np
from Fengji_GUI import Ui_MainWindow
from PyQt5.QtWidgets import QApplication, QMainWindow, QSizePolicy, QWidget, QVBoxLayout
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
# from matplotlib.figure import Figure
# import matplotlib as mpl
import matplotlib.pyplot as plt
import math
import scipy.linalg as linalg
import requests

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super().__init__()
        self.axAzim = 350
        self.axElev = 20
        self.setupUi(self)

        self.ax = None
        # 设置风机的基本结构参赛
        self.H_tower = 120.0  # 塔筒高度
        self.L_blade = 80.0  # 叶片长度
        self.L_cabin_front = 8.0  # 机舱前部突出塔筒中心至轮毂中心的长度，一般大约5米
        self.L_cabin_tail = 8  # 机舱尾部突出塔筒中心长度，一般大约8～10米

        # 航点位置的基本参赛
        self.LeadingEdge_Offset_Y = 15.0  # 前缘 Y 方向偏移量  相当于水平方向
        self.LeadingEdge_Offset_Z = 15.0  # 前缘 Z 方向偏移量  相当于垂直方向
        self.TrailingEdge_Offset_Y = 15.0  # 后缘 Y 方向偏移量  相当于水平方向
        self.TrailingEdge_Offset_Z = 15.0  # 后缘 Z 方向偏移量  相当于垂直方向
        self.TargetPoint_StepLength = 10.0  # 叶片上的检测目标点的间距  实际应由变焦镜头的实际视场角能覆盖的叶片长度来决定

        # 无人机的航向角 默认=0,指向正北， 顺时针为正 逆时针为负
        self.UAV_Flight_Heading_Direction = 0

        # 实际沿Y轴、Z轴旋转的角度
        self.angle_yawY = 0  # 叶片1在X-Z平面上与X轴正向的夹角，代表绕Y轴逆时针转的角度. angle_yawY=0时，1号叶片在水平位置，叶尖方向指向正东方向
        self.angle_yawZ = 0  # 3叶片组成的大平面与X轴在X-Y平面上与X轴的夹角，也即是绕Z轴逆时针时的夹角， angle_yawZ=0时，机舱纵向中心线正向指向正北方向

        # 在图上是否显示航线
        self.ShowFlyPoints = False

        self.SetBaseParameters()
        self.CallBackFunctions()

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot([0, 0, 1, 1], projection='3d')
        self.canvas = FigureCanvas(self.fig)
        # # 放入界面中
        self.LayoutCentralFrame.addWidget(self.canvas)

        self.plot(self.angle_yawY, self.angle_yawZ)

    def CallBackFunctions(self):
        self.btnYZero.clicked.connect(self.SetYValueToZero)
        self.btnYAdd.clicked.connect(self.SetYValueAdd)
        self.btnYSub.clicked.connect(self.SetYValueSub)
        self.btnZZero.clicked.connect(self.SetZValueToZero)
        self.btnZAdd.clicked.connect(self.SetZValueAdd)
        self.btnZSub.clicked.connect(self.SetZValueSub)
        self.btnViewUpToDown.clicked.connect(self.SetViewUpToDown)
        self.btnSouthToNorth.clicked.connect(self.SetViewSouthToNorth)
        self.btnEastToWest.clicked.connect(self.SetViewEastToWest)
        self.btnSetBaseParameters.clicked.connect(self.ReSetBaseParameters)

    def SetBaseParameters(self):
        self.H_tower = float(self.H_towerLE.text())
        self.L_blade = float(self.L_bladelE.text())
        self.L_cabin_front = float(self.L_cabin_frontlE.text())
        self.L_cabin_tail = float(self.L_cabin_taillE.text())
        self.LeadingEdge_Offset_Y = float(self.lE_LeadingEdge_Offset_Y.text())
        self.LeadingEdge_Offset_Z = float(self.lE_LeadingEdge_Offset_Z.text())
        self.TrailingEdge_Offset_Y = float(self.lE_TrailingEdge_Offset_Y.text())
        self.TrailingEdge_Offset_Z = float(self.lE_TrailingEdge_Offset_Z.text())
        self.TargetPoint_StepLength = float(self.lE_TargetPoint_StepLength.text())

    def SetYValueSub(self):
        self.angle_yawY = self.angle_yawY - 5
        self.Y_Anglelcd.setProperty("value", str(self.angle_yawY))
        self.plot(self.angle_yawY, self.angle_yawZ)

    def SetYValueAdd(self):
        self.angle_yawY = self.angle_yawY + 5
        self.Y_Anglelcd.setProperty("value", str(self.angle_yawY))
        self.plot(self.angle_yawY, self.angle_yawZ)

    def SetYValueToZero(self):
        self.angle_yawY = 0
        self.Y_Anglelcd.setProperty("value", str(self.angle_yawY))
        self.plot(self.angle_yawY, self.angle_yawZ)

    def SetZValueToZero(self):
        self.angle_yawZ = 0
        self.Z_Anglelcd.setProperty("value", str(self.angle_yawZ))
        self.plot(self.angle_yawY, self.angle_yawZ)

    def SetZValueAdd(self):
        self.angle_yawZ = self.angle_yawZ + 5
        self.Z_Anglelcd.value = self.angle_yawZ
        self.Z_Anglelcd.setProperty("value", str(self.angle_yawZ))
        self.plot(self.angle_yawY, self.angle_yawZ)

    def SetZValueSub(self):
        self.angle_yawZ = self.angle_yawZ - 5
        self.Z_Anglelcd.setProperty("value", str(self.angle_yawZ))
        self.plot(self.angle_yawY, self.angle_yawZ)

    def ReSetBaseParameters(self):
        self.SetBaseParameters()
        self.plot()

    def SetViewUpToDown(self):
        self.axElev = 90
        self.axAzim = -90
        self.ax.view_init(elev=self.axElev, azim=self.axAzim)
        self.plot(self.angle_yawY, self.angle_yawZ)

    def SetViewSouthToNorth(self):
        self.axElev = 0
        self.axAzim = -90
        self.ax.view_init(elev=self.axElev, azim=self.axAzim)
        self.plot(self.angle_yawY, self.angle_yawZ)

    def SetViewEastToWest(self):
        self.axElev = 0
        self.axAzim = 0
        self.ax.view_init(elev=self.axElev, azim=self.axAzim)
        self.plot(self.angle_yawY, self.angle_yawZ)

    def plot(self, yaw_Y=0, yaw_Z=0):
        """
        按照实际的转角计算风机各参数并画出
        :param yaw_Y:   风机绕 Y 轴转的角度（叶片的垂直旋转角度，即1号叶片相对于X轴正方向的夹角）
        :param yaw_Z:   风机绕 Z 轴转的角度 (叶片的水平旋转角度，即1号叶片相对于X轴正方向的夹角)
        :return: None
        """
        self.ax.clear()
        self.ax.set_xlim([-100, 100])
        self.ax.set_ylim([-100, 100])
        self.ax.set_zlim([-120, 90])

        self.ax.set_xlabel('X(->East)')
        self.ax.set_ylabel('Y(->North)')
        self.ax.set_zlabel('Z(->Up)')

        self.angle_yawY = yaw_Y
        self.angle_yawZ = yaw_Z

        # 画塔筒
        m = np.array([[0, 0, 0], [0, 0, - self.H_tower]])
        x = [x[0] for x in m]
        y = [x[1] for x in m]
        z = [x[2] for x in m]
        self.ax.plot(x, y, z, label='parametric curve')

        # 画机舱
        # 机舱只能绕Z轴转
        # 机舱的前端，它默认位置是在与X轴夹角90的位置
        p1 = [0, 0, 0]
        p2_s = [self.L_cabin_front, 0, 0]
        p2 = self.rotate(p2_s, 0, self.angle_yawZ + 90)
        m = np.array([p1, p2])
        x = [x[0] for x in m]
        y = [x[1] for x in m]
        z = [x[2] for x in m]
        self.ax.plot(x, y, z, label='parametric curve')

        # 这个点，是叶片的根部，也就是轮毂的中心点
        CenterPoint = p2

        # 机舱尾部标线
        # theta2 = (cabin_angle + 180) * np.pi / 180  # 绕Z轴角度转弧度
        p1 = [0, 0, 0]
        p2_s = [self.L_cabin_tail, 0, 0]
        p2 = self.rotate(p2_s, 0, self.angle_yawZ + 90 + 180)
        m = np.array([p1, p2])
        x = [x[0] for x in m]
        y = [x[1] for x in m]
        z = [x[2] for x in m]
        self.ax.plot(x, y, z, label='parametric curve')

        # 画叶片
        # 叶片的标准坐标
        p1 = CenterPoint
        P2_s = [self.L_blade, self.L_cabin_front, 0]
        # 叶片1
        p2 = self.rotate(P2_s, self.angle_yawY, self.angle_yawZ)
        m = np.array([p1, p2])
        x = [x[0] for x in m]
        y = [x[1] for x in m]
        z = [x[2] for x in m]
        self.ax.plot(x, y, z, label='parametric curve')
        # 计算叶片长度
        d1 = np.sqrt(np.sum(np.square(p2 - p1)))
        print("叶片1 旋转后 长度:", d1)

        # 叶片2
        p2 = self.rotate(P2_s, self.angle_yawY + 120, self.angle_yawZ)
        m = np.array([p1, p2])
        x = [x[0] for x in m]
        y = [x[1] for x in m]
        z = [x[2] for x in m]
        self.ax.plot(x, y, z, label='parametric curve')
        # 计算叶片长度
        d1 = np.sqrt(np.sum(np.square(p2 - p1)))
        print("叶片2 旋转后 长度:", d1)

        # 叶片3
        p2 = self.rotate(P2_s, self.angle_yawY + 120 + 120, self.angle_yawZ)
        m = np.array([p1, p2])
        x = [x[0] for x in m]
        y = [x[1] for x in m]
        z = [x[2] for x in m]
        self.ax.plot(x, y, z, label='parametric curve')
        # 计算叶片长度
        d1 = np.sqrt(np.sum(np.square(p2 - p1)))
        print("叶片3 旋转后 长度:", d1)

        # 画出各叶片上的目标点
        # 目标点初始位置数组
        # targets_num = 8
        targets_num = int(self.L_blade // self.TargetPoint_StepLength)
        target_points = np.arange(targets_num * 3, dtype=np.float32).reshape(targets_num, 3)
        for i in range(targets_num):
            target_points[i] = [self.TargetPoint_StepLength + i * self.TargetPoint_StepLength, self.L_cabin_front, 0]

        # 叶片1 的检测目标点
        target_points_1 = np.arange(targets_num * 3, dtype=np.float32).reshape(targets_num, 3)
        for i in range(targets_num):
            # t = [10 + i * 10, self.L_cabin_front, 0]
            t = target_points[i]
            target_points_1[i] = self.rotate(t, self.angle_yawY, self.angle_yawZ)
        x = [x[0] for x in target_points_1]
        y = [x[1] for x in target_points_1]
        z = [x[2] for x in target_points_1]
        self.ax.scatter(x, y, z, c='b', marker='o')

        # 叶片2 的检测目标点
        target_points_2 = np.arange(targets_num * 3, dtype=np.float32).reshape(targets_num, 3)
        for i in range(targets_num):
            # t = [10 + i * 10, self.L_cabin_front, 0]
            t = target_points[i]
            target_points_2[i] = self.rotate(t, self.angle_yawY + 120, self.angle_yawZ)
        x = [x[0] for x in target_points_2]
        y = [x[1] for x in target_points_2]
        z = [x[2] for x in target_points_2]
        self.ax.scatter(x, y, z, c='b', marker='o')

        # 叶片3 的检测目标点
        target_points_3 = np.arange(targets_num * 3, dtype=np.float32).reshape(targets_num, 3)
        for i in range(targets_num):
            # t = [10 + i * 10, self.L_cabin_front, 0]
            t = target_points[i]
            target_points_3[i] = self.rotate(t, self.angle_yawY + 120 + 120, self.angle_yawZ)
        x = [x[0] for x in target_points_3]
        y = [x[1] for x in target_points_3]
        z = [x[2] for x in target_points_3]
        self.ax.scatter(x, y, z, c='b', marker='o')

        # 叶片1 的航拍点
        # 标准的航点数组
        # 航拍点相对于目标的坐标偏移量
        offset_Y = self.LeadingEdge_Offset_Y
        offset_Z = self.LeadingEdge_Offset_Z
        # 叶片前缘、上侧航线点--原始点位置
        flyPoints_front_up = np.arange(targets_num * 3, dtype=np.float32).reshape(targets_num, 3)
        # 叶片前缘、下侧航线点--原始点位置
        flyPoints_front_down = np.arange(targets_num * 3, dtype=np.float32).reshape(targets_num, 3)
        for i in range(targets_num):
            flyPoints_front_up[i][0] = target_points[i][0]
            flyPoints_front_up[i][1] = target_points[i][1] + offset_Y
            flyPoints_front_up[i][2] = target_points[i][2] + offset_Z
            flyPoints_front_down[i][0] = target_points[i][0]
            flyPoints_front_down[i][1] = target_points[i][1] + offset_Y
            flyPoints_front_down[i][2] = target_points[i][2] - offset_Z

        # 计算 叶片1巡检 前缘侧上方 实际航点位置
        flyPoints_front_up_1 = np.arange(targets_num * 3, dtype=np.float32).reshape(targets_num, 3)
        for i in range(targets_num):
            flyPoints_front_up_1[i] = self.rotate(flyPoints_front_up[i], self.angle_yawY, self.angle_yawZ)
        x = [x[0] for x in flyPoints_front_up_1]
        y = [x[1] for x in flyPoints_front_up_1]
        z = [x[2] for x in flyPoints_front_up_1]
        self.ax.scatter(x, y, z, c='r', marker='^')

        # 计算 叶片1巡检 前缘侧下方 实际航点位置
        flyPoints_front_down_1 = np.arange(targets_num * 3, dtype=np.float32).reshape(targets_num, 3)
        for i in range(targets_num):
            flyPoints_front_down_1[i] = self.rotate(flyPoints_front_down[i], self.angle_yawY, self.angle_yawZ)
        x = [x[0] for x in flyPoints_front_down_1]
        y = [x[1] for x in flyPoints_front_down_1]
        z = [x[2] for x in flyPoints_front_down_1]
        self.ax.scatter(x, y, z, c='r', marker='^')

        # 叶片2 的航拍点
        # 标准的航点数组
        # 航拍点相对于目标的坐标偏移量
        offset_Y = self.LeadingEdge_Offset_Y
        offset_Z = self.LeadingEdge_Offset_Z
        # 叶片前缘、上侧航线点--原始点位置
        flyPoints_front_up = np.arange(targets_num * 3, dtype=np.float32).reshape(targets_num, 3)
        # 叶片前缘、下侧航线点--原始点位置
        flyPoints_front_down = np.arange(targets_num * 3, dtype=np.float32).reshape(targets_num, 3)
        for i in range(targets_num):
            flyPoints_front_up[i][0] = target_points[i][0]
            flyPoints_front_up[i][1] = target_points[i][1] + offset_Y
            flyPoints_front_up[i][2] = target_points[i][2] + offset_Z
            flyPoints_front_down[i][0] = target_points[i][0]
            flyPoints_front_down[i][1] = target_points[i][1] + offset_Y
            flyPoints_front_down[i][2] = target_points[i][2] - offset_Z

        # 计算 叶片2巡检 前缘侧上方 实际航点位置
        flyPoints_front_up_2 = np.arange(targets_num * 3, dtype=np.float32).reshape(targets_num, 3)
        for i in range(targets_num):
            flyPoints_front_up_2[i] = self.rotate(flyPoints_front_up[i], self.angle_yawY + 120, self.angle_yawZ)
        x = [x[0] for x in flyPoints_front_up_2]
        y = [x[1] for x in flyPoints_front_up_2]
        z = [x[2] for x in flyPoints_front_up_2]
        self.ax.scatter(x, y, z, c='r', marker='^')

        # 计算 叶片2巡检 前缘侧下方 实际航点位置
        flyPoints_front_down_2 = np.arange(targets_num * 3, dtype=np.float32).reshape(targets_num, 3)
        for i in range(targets_num):
            flyPoints_front_down_2[i] = self.rotate(flyPoints_front_down[i], self.angle_yawY + 120, self.angle_yawZ)
        x = [x[0] for x in flyPoints_front_down_2]
        y = [x[1] for x in flyPoints_front_down_2]
        z = [x[2] for x in flyPoints_front_down_2]
        self.ax.scatter(x, y, z, c='r', marker='^')

        # 计算 叶片3巡检 前缘侧上方 实际航点位置
        flyPoints_front_up_3 = np.arange(targets_num * 3, dtype=np.float32).reshape(targets_num, 3)
        for i in range(targets_num):
            flyPoints_front_up_3[i] = self.rotate(flyPoints_front_up[i], self.angle_yawY + 120 + 120, self.angle_yawZ)
        x = [x[0] for x in flyPoints_front_up_3]
        y = [x[1] for x in flyPoints_front_up_3]
        z = [x[2] for x in flyPoints_front_up_3]
        self.ax.scatter(x, y, z, c='r', marker='^')

        # 计算 叶片2巡检 前缘侧下方 实际航点位置
        flyPoints_front_down_3 = np.arange(targets_num * 3, dtype=np.float32).reshape(targets_num, 3)
        for i in range(targets_num):
            flyPoints_front_down_3[i] = self.rotate(flyPoints_front_down[i], self.angle_yawY + 120 + 120,
                                                    self.angle_yawZ)
        x = [x[0] for x in flyPoints_front_down_3]
        y = [x[1] for x in flyPoints_front_down_3]
        z = [x[2] for x in flyPoints_front_down_3]
        self.ax.scatter(x, y, z, c='r', marker='^')

        # # 将所有航点组成一条航线
        # 叶片1—前-上 --> 叶片1-前-下（倒序） --> 叶片3-前-上 --> 叶片3-前-下（倒序） --> 叶片2-前-上 --> 叶片2-前-下(倒叙)
        # 叶片1—前-上
        flyPoints_temp = np.copy(flyPoints_front_up_1)
        flyPoints = np.copy(flyPoints_temp)
        # 叶片1—前-下(倒序)
        flyPoints_temp = np.copy(flyPoints_front_down_1[::-1])
        flyPoints = np.concatenate([flyPoints, flyPoints_temp])
        #  叶片3-前-上
        flyPoints_temp = np.copy(flyPoints_front_up_3)
        flyPoints = np.concatenate([flyPoints, flyPoints_temp])
        # 叶片3-前-下（倒序）
        flyPoints_temp = np.copy(flyPoints_front_down_3[::-1])
        flyPoints = np.concatenate([flyPoints, flyPoints_temp])
        #  叶片2-前-上
        flyPoints_temp = np.copy(flyPoints_front_up_2)
        flyPoints = np.concatenate([flyPoints, flyPoints_temp])
        # 叶片2-前-下（倒序）
        flyPoints_temp = np.copy(flyPoints_front_down_2[::-1])
        flyPoints = np.concatenate([flyPoints, flyPoints_temp])

        # 画标签
        for point, i in zip(flyPoints, range(len(flyPoints))):
            print(point)
            self.ax.text(point[0], point[1], point[2], i)
        # 画线
        # if self.ShowFlyPoints:
        if self.cBx_ShowFlyPoints.isChecked():
            x = [x[0] for x in flyPoints]
            y = [x[1] for x in flyPoints]
            z = [x[2] for x in flyPoints]
            self.ax.plot(x, y, z, c='b', linestyle='--')

        # 计算某航点的飞机航向、云台P\T\Z
        TestPointNo = targets_num - 1
        # 界面输出航点的坐标
        self.label_X.setText('X=' + str(round(flyPoints_front_down_1[TestPointNo][0], 2)))
        self.label_Y.setText('Y=' + str(round(flyPoints_front_down_1[TestPointNo][1], 2)))
        self.label_Z.setText('Z=' + str(round(flyPoints_front_down_1[TestPointNo][2], 2)))

        # 画实际点和目标点之间的连线

        p1 = flyPoints_front_down_1[TestPointNo]
        p2 = target_points_1[TestPointNo]
        m = np.array([p1, p2])
        x = [x[0] for x in m]
        y = [x[1] for x in m]
        z = [x[2] for x in m]
        self.ax.plot(x, y, z, c='y', linestyle='--')

        # 利用 flyPoints_front_down_1[TestPointNo] 这个航点，来进行验证
        # 计算 云台的俯仰角Tilt， 水平为0,向上为正，向下为负
        # 以无人机的航点为原点，画一条水平线，

        a1 = flyPoints_front_down_1[TestPointNo]
        a2 = np.arange(1 * 3, dtype=np.float32)
        np.copyto(a2, target_points_1[TestPointNo])
        a2[2] = flyPoints_front_down_1[TestPointNo][2]
        m = np.array([a1, a2])
        x = [x[0] for x in m]
        y = [x[1] for x in m]
        z = [x[2] for x in m]
        self.ax.plot(x, y, z, c='y', linestyle='--')
        # 计算 航拍点到目标点连线 与 航拍点到水平参考点连线 之间的夹角
        theta = self.IncludedAngle(flyPoints_front_down_1[TestPointNo], target_points_1[TestPointNo], a2)
        # 判断是仰角（为正）还是俯视角（为负）
        if flyPoints_front_down_1[TestPointNo][2] > target_points_1[TestPointNo][2]:
            # 航点比目标点高，为俯视
            theta = 0 - theta
        self.label_Tilt.setText('云台俯仰Tilt=' + str(round(theta, 2)))

        # 计算云台的航向偏转角度Pan
        # 先定义一个从该航点指向正北的点
        a3 = np.arange(1 * 3, dtype=np.float32)
        np.copyto(a3, flyPoints_front_down_1[TestPointNo])
        a3[1] = a3[1] + 30  # Y坐标加,指向北
        # 计算云台的转角和飞机实际航向
        self.UAV_Flight_Heading_Direction = 0.0
        theta2 = self.IncludedAngle(flyPoints_front_down_1[TestPointNo], a3, a2)
        # 判断正负角， 顺时针为正
        if flyPoints_front_down_1[TestPointNo][0] > target_points_1[TestPointNo][0]:
            theta2 = 0 - theta2
            if theta2 < -90:
                self.UAV_Flight_Heading_Direction = -90
                theta2 = theta2 - self.UAV_Flight_Heading_Direction
        else:
            if theta2 > 90:
                self.UAV_Flight_Heading_Direction = 90
                theta2 = theta2 - self.UAV_Flight_Heading_Direction

        # 根据飞机的实际航向，画出标志线
        # 先定义一个从该航点指向正北的点
        course_point = np.arange(1 * 3, dtype=np.float32)
        np.copyto(course_point, flyPoints_front_down_1[TestPointNo])
        if self.UAV_Flight_Heading_Direction == 0:
            course_point[1] = course_point[1] + 30  # Y坐标加,指向北
        elif self.UAV_Flight_Heading_Direction == 90:
            course_point[0] = course_point[0] + 30  # X坐标加,指向正东
        elif self.UAV_Flight_Heading_Direction == -90:
            course_point[0] = course_point[0] - 30  # X坐标减,指向正西

        # 画出这条线，代表无人机的航向
        m = np.array([flyPoints_front_down_1[TestPointNo], course_point])
        x = [x[0] for x in m]
        y = [x[1] for x in m]
        z = [x[2] for x in m]
        # 计算航拍点到目标点连线在XY面上的投影，与正北方向的夹角
        self.ax.plot(x, y, z, c='b', linestyle=':')

        self.lE_UAV_Flight_Heading_Direction.setText('飞机航向=' + str(round(self.UAV_Flight_Heading_Direction, 2)))
        self.label_Pan.setText('云台转角Pan=' + str(round(theta2, 2)))
        # 更新图形
        self.canvas.draw()

    # 计算水平与拍照线的夹角
    def angle(self, target_point, flyPoint, TempPoint):
        # # 计算拍照线的长度，相当于直角三角型的斜边
        # d1 = np.sqrt(np.sum(np.square(flyPoint - target_point)))
        # # 参考水平线的长度，相当于直角三角型的股边
        # d2 = np.sqrt(np.sum(np.square(flyPoint - TempPoint)))
        v1 = target_point - flyPoint
        v2 = TempPoint - flyPoint
        return np.degrees(np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))))

    # 旋转矩阵 欧拉角
    def rotate_mat(self, axis, radian):
        return linalg.expm(np.cross(np.eye(3), axis / linalg.norm(axis) * radian))

    # 绕轴旋转，对于风机来说，不存在绕X轴的旋转
    def rotate(self, p, yaw_Y=0, yaw_Z=0):
        """
        绕轴旋转
        :param p: 原始点三维坐标
        :param yaw_Y: 绕 X轴旋转的角度
        :param yaw_Z: 绕 X轴旋转的角度
        :return: 旋转后的点三维坐标
        """
        # 分别是x,y和z轴,也可以自定义旋转轴
        axis_x, axis_y, axis_z = [1, 0, 0], [0, 1, 0], [0, 0, 1]
        # 旋转角度,转成弧度
        radian_y = -yaw_Y * math.pi / 180
        radian_z = yaw_Z * math.pi / 180
        # 返回绕Y轴旋转矩阵, 相当于叶片旋转
        rot_matrix = self.rotate_mat(axis_y, radian_y)
        newP = np.dot(rot_matrix, p)
        # 返回绕Z轴旋转矩阵，相当于风机旋转
        rot_matrix = self.rotate_mat(axis_z, radian_z)
        newP = np.dot(rot_matrix, newP)
        return newP

    # 计算以a0为原点，到a1,a2点，两个向量之间的夹角
    def IncludedAngle(self, a0, a1, b1):
        a11 = a1 - a0
        b11 = b1 - a0
        cos_angle = np.dot(a11, b11) / (np.linalg.norm(a11) * np.linalg.norm(b11))
        angle = np.arccos(cos_angle)
        print('夹角为：', angle, '弧度')
        angle1 = angle * 180 / np.pi
        print('夹角为：', angle1, '度')
        return angle1


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
