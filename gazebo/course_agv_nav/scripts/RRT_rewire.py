# -*- coding: utf-8 -*-
from cmath import inf
import numpy as np
import math as m



class RRT:
    """RRT算法类

    Attributes:
        known：类型：Tree. 探索过的顶点
        start_point: 类型：Point，起点
        end_point: 类型：Point 终点
        start_node: 类型：Tree 起点所对应的顶点
        Vertex: 定义一个顶点类（class）,用于描述图中的顶点

    """

    def __init__(self,obstacles_point, start_point, end_point, planning_minx,planning_miny,planning_maxx,planning_maxy,grid,dist):
        """"由给定起点、终点初始化"""
        self.node_set = []
        self.start_point = start_point
        self.end_point = end_point
        self.start_node = self.RRT_Tree(start_point,[],[],end_point,start_point,grid)
        self.planning_minx = planning_minx
        self.planning_miny = planning_miny
        self.planning_maxx = planning_maxx
        self.planning_maxy = planning_maxy
        self.obstacles_point = obstacles_point
        self.grid = grid
        self.dist = dist
       
    class RRT_Tree:
        """RRT树

        Attributes:
            point 当前节点的坐标
            father 父节点
            child 子节点
        """
        def __init__(self,point,father,child,endpoint,startpoint,grid):
            self.point = point
            self.father = father
            self.child = child
            self.endpoint = endpoint  # 终点坐标
            self.startpoint = startpoint # 起点坐标
            if father:
                self.value = self.father.value+grid
            else:
                self.value = 0


    def Process(self):
        """RRT算法核心函数，整个算法的流程控制

        通过发散找点，最终到终点附近的时候进行回溯

        Args:
            self: RRT类所有变量均已知

        return:
            best_path_X:最有路径的x坐标
            best_path_Y:最有路径的y坐标
        """
        new_node = self.start_node
        self.node_set.append(new_node)
        while self.if_nearend(new_node) == 0:
            new_node = self.search_next()
            self.node_set.append(new_node)
        end_node = self.RRT_Tree(self.end_point,new_node,[],self.end_point,self.start_point,self.grid)
        best_path_X = []
        best_path_Y = []
        temp_node = end_node  # 拿到终点顶点
        while (temp_node):
            best_path_X.append(temp_node.point[0])
            best_path_Y.append(temp_node.point[1])
            temp_node = temp_node.father
        best_path_X.reverse()  # 逆转链表
        best_path_Y.reverse()  # 逆转链表
        return best_path_X,best_path_Y
    def search_next(self):
        """搜寻并确定下一个可到达的顶点near_node

        Args:
            self: RRT_Tree类所有变量均已知，包括当前坐标self.point

        return:
            near_node: 类型：Tree 下一个可探索的顶点
        """
        flag =1
        while flag:
            flag = 0
            # 利用rand()函数在[0,1]区间内随机生成一个数
            if np.random.rand() < 0.5:
                # 如果小于0.5，则在图 img_binary 的范围内随机采样一个点
                x_temp = np.random.randint(self.planning_minx,self.planning_maxx, dtype=int)
                y_temp = np.random.randint(self.planning_miny,self.planning_maxy, dtype=int)
                temp_point = [x_temp,y_temp]
            else:
                # 否则用目标点作为采样点
                temp_point = self.end_point
            node_near_set,dist,nearest_node = self.min_distance_node(temp_point)
            d = ([temp_point[i]-nearest_node.point[i] for i in range(len(temp_point))])/np.linalg.norm([temp_point[i]-nearest_node.point[i] for i in range(len(temp_point))])
            near_node = RRT.RRT_Tree(nearest_node.point+d*self.grid,nearest_node,[],nearest_node.endpoint,nearest_node.startpoint,self.grid)
            for node in node_near_set:
                if near_node.value>node.value+self.grid:
                    near_node.father = node
                if near_node.value+self.grid<node.value:
                    node.father = near_node
            for obstacles_point in self.obstacles_point:
                if np.linalg.norm(obstacles_point-near_node.point)<self.dist:
                    flag = 1
                    break 
        nearest_node.child.append(near_node)     
        return near_node

    def min_distance_node(self,temp_point):
        """搜寻并确定离temp_point最近的树节点

        Args:
            self: RRT_Tree类所有变量均已知，包括当前坐标self.point

        return:
            dist 距离，near_node: 类型：Tree 下一个可探索的顶点
        """
        near_set=[]
        dist = m.inf
        for node in self.node_set:
            temp_dist = np.linalg.norm([node.point[i]-temp_point[i] for i in range(len(temp_point))])
            if temp_dist < self.grid:
                near_set.append(node)
            if temp_dist<dist:
                dist = temp_dist
                near_node  = node
        return near_set,dist,near_node

    def if_nearend(self,node):
        """确定当前node 是否可以到达

        Args:
            self: RRT_Tree类所有变量均已知，包括当前坐标self.point

        return:
            dist 距离，near_node: 类型：Tree 下一个可探索的顶点
        """
        if np.linalg.norm(node.point-node.endpoint) < self.grid:
            return 1
        return 0

    