# -*- coding: utf-8 -*-
import numpy as np
import math as m


class A_star:
    """A*算法类

    通过A*算法求解二维栅格地图中指定起点、终点的最短路径序列

    Attributes:
        closed: 类型：list，已经探索过且封闭的顶点
        unknown：类型：list. 未探索过的顶点
        now: 类型：Vertex，当前所处顶点
        start_point: 类型：Point，起点
        end_point: 类型：Point 终点
        map: 类型：array 栅格地图信息
        end_Vertex: 类型：Vertex 终点所对应的顶点
        max_iter: 类型：int 算法的最大迭代次数，超过此次数终止计算
        Vertex: 定义一个顶点类（class）,用于描述图中的顶点

    """

    def __init__(self,obstacles_point, start_point, end_point, planning_minx,planning_miny,planning_maxx,planning_maxy,grid,dist):
        """"由给定起点、终点以及地图初始化A*的数据"""
        self.closed = []  # 封闭的顶点
        self.unknown = []  # 未探索过的顶点
        self.now = 0       # 现在的Vertex
        self.start_point = (int((start_point[0]-planning_minx)/grid),int((start_point[1]-planning_miny)/grid))
        self.end_point = (int((end_point[0]-planning_minx)/grid),int((end_point[1]-planning_miny)/grid))
        self.end_Vertex = self.Vertex(self.end_point, self.end_point, m.inf)
        self.max_iter = 50000
        self.grid = grid
        self.dist = dist
        self.planning_minx = planning_minx
        self.planning_miny = planning_miny
        self.planning_maxx = planning_maxx
        self.planning_maxy = planning_maxy
        self.width = int((planning_maxx - planning_minx)/grid+1)
        self.height = int((planning_maxy - planning_miny)/grid+1)
        self.map = np.empty([self.height, self.width], dtype=float)
        for i in range(self.height):
            for j in range(self.width):
                self.temp=np.hypot(obstacles_point[:,0]-(i*grid+planning_minx),obstacles_point[:,1]-(j*grid+planning_miny))
                self.map[i,j] = min(self.temp)
                if self.map[i,j] > dist:
                    self.map[i,j] = 0
                else:
                    self.map[i,j] = 1

    class Vertex:
        """ 图中顶点类

        将栅格地图中的坐标点抽象为“图”中的顶点，储存更多信息，便于实现A*

        Attributes:
            point: 类型：Point 当前顶点所对应的坐标点
            endpoint: 类型： Point  终点坐标（本质上和上面的end_point一致，这里定义成类内变量更加方便后续的编程
            father: 类型：Vertex 类似一个指针，指向自己的上一个顶点，用于回溯
            g：类型：float A*算法中的g(n)函数值，表示现有已知代价
            h: 类型：float A*算法中的h(n)函数值，表示对当前节点的未来代价的评估
            f: 类型：float g(n)函数 + f(n)函数，用来评估该节点的总代价
        """

        def __init__(self, point, endpoint, g):
            """"使用给定坐标、终点坐标、已得g值进行初始化"""
            self.point = point        # 自己坐标
            self.endpoint = endpoint  # 终点坐标
            self.father = None        # 父节点，用来回溯最短路
            self.g = g                # 现有代价g值，是由起点到该点的累加值
            self.h =m.hypot((point[0]-endpoint[0]),(point[1]-endpoint[1]))
            self.f = self.g + self.h

        def search_next(self, vertical, horizontal):
            """搜寻并确定下一个可到达的顶点Vertex

            通过传入的水平位移dx - horizontal和竖直位移dy-vertical来确定下一个可到达的顶点

            Args:
                self: Vertex类所有变量均已知，包括当前坐标self.point
                vertical: 类型：int 表示竖直方向的坐标增量 dy
                horizontal： 类型：int 表示水平方向坐标增量 dx

            return:
                nearVertex: 类型：Vertex 下一个可探索的顶点
            """
            nextpoint =(self.point[0] + horizontal,
                              self.point[1] + vertical)
            if vertical != 0 and horizontal != 0:  # 走斜对角线，g值增加根号2
                nearVertex = A_star.Vertex(
                    nextpoint, self.endpoint, self.g + 1.414)
            else:    # 走十字方向，g值增加1
                nearVertex = A_star.Vertex(
                    nextpoint, self.endpoint, self.g + 1)
            return nearVertex

    def Process(self):
        """A*算法核心函数，整个算法的流程控制

        每次从未探索区域中找到f(n)最小的顶点，并进行探索和回溯
        最终得到终点的顶点end_Vertex,通过father指针逆向索引得到最优路径

        Args:
            self: A_star类所有变量均已知

        return:
            self.closed : 类型：list 列表中的每个元素为Vertex, 是所有曾探索过的点
            best_path: 类型：list 列表中的每个元素为Vertex，是从起点到终点的路径序列顶点集
        """
        start_Vertex = self.Vertex(self.start_point, self.end_point, 0)
        self.unknown.append(start_Vertex)
        arrive = 0
        count = 0
        while arrive != 1 and count < self.max_iter:  # arrive = 1时到达终点
            self.now = self.min_cost_Vertex()     # 每次从unknown中选取一个f(n)最小的Vertex
            self.closed.append(self.now)    # 起点不计入
            arrive = self.explore_next(self.now)  # 对选中的Vertex进行周边探索
            count += 1
        best_path_X = []
        best_path_Y = []
        temp_Vertex = self.end_Vertex  # 拿到终点顶点
        # while (temp_Vertex.point[0] != self.start_plloint[0] or temp_Vertex.point[1] != self.start_point[1]):
        while (temp_Vertex):
            best_path_X.append(temp_Vertex.point[0])
            best_path_Y.append(temp_Vertex.point[1])
            temp_Vertex = temp_Vertex.father
        best_path_X.reverse()  # 逆转链表
        best_path_Y.reverse()  # 逆转链表
        best_path_X=[float(i)*self.grid+self.planning_minx for i in best_path_X]
        best_path_Y=[float(i)*self.grid+self.planning_miny for i in best_path_Y]
        # print(best_path_X,best_path_Y)
        return best_path_X,best_path_Y

    def min_cost_Vertex(self):
        """在unknown list中找到最小f(n)的顶点
           将该顶点从unknown list中移除并添加到closed list中

        Args:
            self: A_star类所有变量均已知

        return:
            Vertex_min: 类型：Vertex  unknown顶点集中f(n)最小的顶点
        """
        Vertex_min = self.unknown[0]
        for Vertex_temp in self.unknown:
            if Vertex_temp.f < Vertex_min.f:
                Vertex_min = Vertex_temp
        self.unknown.remove(Vertex_min)
        return Vertex_min

    def is_unknown(self, Vertex):
        """判断顶点是否在未探索区域

        如果改顶点属于为探索区域，则返回未更新过的顶点

        Args:
            self: A_star类所有变量均已知
            Vertex: 类型：Vertex 待判断的顶点

        return:
            0: 表示不属于未探索区域
            Vertex_temp：unknown列表中未更新的Vertex
        """
        for Vertex_temp in self.unknown:
            if Vertex_temp.point == Vertex.point:
                return Vertex_temp
        return 0

    def is_closed(self, Vertex):
        """判断顶点是否在已探索区域

        Args:
            self: A_star类所有变量均已知
            Vertex: 类型：Vertex 待判断的顶点

        return:
            0/1: 1表示该顶点属于已探索区域，0表示不属于  
        """
        for closeVertex_temp in self.closed:
            if closeVertex_temp.point[0] == Vertex.point[0] and closeVertex_temp.point[1] == Vertex.point[1]:
                return 1
        return 0

    def is_obstacle(self, Vertex):
        """判断顶点是否位于障碍物区域

        Args:
            self: A_star类所有变量均已知
            Vertex: 类型：Vertex 待判断的顶点

        return:
            0/1: 1表示该顶点位于障碍物区域，0表示不位于  
        """
        if self.map[Vertex.point[0]][Vertex.point[1]] == 1:
            return 1
        return 0

    def explore_next(self, Vertex):
        """探索下一个顶点

        栅格地图中可以探索的有8个方向，可以用vertical和horizontal的循环表示
        相应地更新unknown序列，可有配合Vertex类中的函数search_next来实现

        注意事项:如果探索到的节点已经处于unknown list中,那么需要对比一下原有的f值
        与当前的f值，选取最小f值的保存到unknown list中

        Args:
            self: A_star类所有变量均已知
            Vertex: 类型：Vertex 当前所在的顶点

        return:
            0：表示还未探索到终点
            1：表示已经探索到终点，A*即将结束
        """
        num = {-1, 0, 1}
        for vertical in num:
            for horizontal in num:
                if vertical == 0 and horizontal == 0:
                    continue
                if Vertex.point[0] + horizontal < 0:
                    continue
                if Vertex.point[0] + horizontal >= self.width:
                    continue
                if Vertex.point[1] + vertical < 0:
                    continue
                if Vertex.point[1] + vertical >= self.height:
                    continue
                Vertex_next = Vertex.search_next(vertical, horizontal)
                if Vertex_next.point[0] == self.end_point[0] and Vertex_next.point[1] == self.end_point[1]:
                    Vertex_next.father = Vertex
                    self.end_Vertex = Vertex_next
                    return 1
                if self.is_closed(Vertex_next):
                    continue
                if self.is_obstacle(Vertex_next):
                    continue
                if self.is_unknown(Vertex_next) == 0:
                    Vertex_next.father = Vertex
                    self.unknown.append(Vertex_next)
                else:
                    Vertex_old = self.is_unknown(Vertex_next)
                    if Vertex_next.f < Vertex_old.f:
                        self.unknown.remove(Vertex_old)
                        Vertex_next.father = Vertex
                        self.unknown.append(Vertex_next)
        return 0
