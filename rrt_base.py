"""this module has all the methods and classes in order to compute and help represent RRT
"""
import math
import random
import pygame

from pygame import rect


class RRTMap:
    """generates the obstacles, start and end point"""

    def __init__(self, start, goal, map_dimensions, obs_dim, obs_num):
        self.start = start
        self.goal = goal
        self.map_dimensions = map_dimensions
        self.obs_dim = obs_dim
        self.obs_num = obs_num
        self.map_h, self.map_w = self.map_dimensions

        # window settings
        self.map_window_name = "RRT path planning"
        pygame.display.set_caption(self.map_window_name)
        self.map = pygame.display.set_mode((self.map_w, self.map_h))
        self.map.fill((255, 255, 255))
        self.node_rad = 2
        self.node_thickness = 1
        self.edge_thickness = 1

        self.goal_flag = False

        self.obstacles = []

        # colors
        self.grey_color = (70, 70, 70)
        self.blue_color = (0, 0, 255)
        self.green_color = (0, 255, 0)
        self.red_color = (255, 0, 0)
        self.white_color = (255, 255, 255)

    def draw_map(self, obstacles):
        """draws the map to show RRT interaction

        Args:
            obstacles (list(pygame.Rect)): list of rectangles made by pygame
        """
        pygame.draw.circle(self.map, self.green_color, self.start, self.node_rad + 5, 0)

        pygame.draw.circle(self.map, self.red_color, self.goal, self.node_rad + 20, 1)

        self.draw_obs(obstacles)

    def draw_path(self, path):
        for node in path:
            pygame.draw.circle(self.map, self.red_color, node, self.node_rad + 3, 0)

    def draw_obs(self, obstacles):
        obstacles_list = obstacles.copy()
        while len(obstacles_list) > 0:
            obstacle = obstacles_list.pop(0)
            pygame.draw.rect(self.map, self.grey_color, obstacle)


class RRTGraph:
    """generates the actual graph gotten by RRT"""

    def __init__(self, start, goal, map_dimensions, obs_dim, obs_num):
        (_x, _y) = start
        self.start = start
        self.goal = goal
        self.goal_success = False
        self.map_h, self.map_w = map_dimensions
        self.x = []
        self.y = []
        self.parent = []

        # this is the start of the tree
        self.x.append(_x)
        self.y.append(_y)
        self.parent.append(0)

        self.obstacles = []
        self.obs_dim = obs_dim
        self.obs_num = obs_num

        self.goal_state = None
        self.path = []
        self.goal_flag = False

        self.neighbor = 50

    def make_random_rect(self):
        uppercornerx = int(random.uniform(0, self.map_w - self.obs_dim))
        uppercornery = int(random.uniform(0, self.map_h - self.obs_dim))

        return (uppercornerx, uppercornery)

    def make_obs(self):
        obs = []
        for i in range(0, self.obs_num):
            rectang = None
            startgoalcol = True
            while startgoalcol:
                upper = self.make_random_rect()
                rectang = pygame.Rect(upper, (self.obs_dim, self.obs_dim))
                if rectang.collidepoint(self.start) or rectang.collidepoint(self.goal):
                    startgoalcol = True
                else:
                    startgoalcol = False
            obs.append(rectang)
        self.obstacles = obs.copy()
        return obs

    def remove_node(self, n):
        self.x.pop(n)
        self.y.pop(n)

    def remove_edge(self, n):
        self.parent.pop(n)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent)

    def add_node(self, n, x, y):
        """register a node position and number

        Args:
            n (int): node number
            x (int): node at x position
            y (int): node at y position
        """
        self.x.insert(n, x)
        self.y.append(y)

    def number_of_nodes(self):
        """get the number of registered nodes

        Returns:
            int: amount of registered nodes
        """
        return len(self.x)

    def distance(self, n1, n2):
        """get distance between two nodes"""
        (_x1, _y1) = (self.x[n1], self.y[n1])
        (_x2, _y2) = (self.x[n2], self.y[n2])

        p_x = (float(_x1) - float(_x2)) ** 2
        p_y = (float(_y1) - float(_y2)) ** 2
        return (p_x + p_y) ** (0.5)

    def sample_env(self):
        _x = int(random.uniform(0, self.map_w))
        _y = int(random.uniform(0, self.map_h))
        return _x, _y

    def is_free(self):
        _n = self.number_of_nodes() - 1
        (_x, _y) = (self.x[_n], self.y[_n])
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectang = obs.pop(0)
            if rectang.collidepoint(_x, _y):
                self.remove_node(_n)
                return False
        return True

    def nearest(self, n):
        d_min = self.distance(0, n)
        nnear = 0
        for i in range(0, n):
            if self.distance(i, n) < d_min:
                d_min = self.distance(i, n)
                nnear = i
        return nnear

    def cross_obstacle(self, _x1, _x2, _y1, _y2):
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectang = obs.pop(0)
            for i in range(0, 101):
                _u = i / 100
                _x = _x1 * _u + _x2 * (1 - _u)
                _y = _y1 * _u + _y2 * (1 - _u)
                if rectang.collidepoint(_x, _y):
                    return True
        return False

    def connect(self, _n1, _n2):
        """makes a visual edge between two nodes

        Args:
            n1 (int): node number first
            n2 (int): node number second

        Returns:
            Bool: if a node could be connected or not
        """
        (_x1, _y1) = (self.x[_n1], self.y[_n1])
        (_x2, _y2) = (self.x[_n2], self.y[_n2])
        if self.cross_obstacle(_x1, _x2, _y1, _y2):
            self.remove_node(_n2)
            return False
        self.add_edge(_n1, _n2)
        return True

    def step(self, nnear, nrand, d_max=35):
        """computes the step in direction to the node generated

        Args:
            nnear (int): node number parent
            nrand (int): node number random
            d_max (int, optional): step size to directed node. Defaults to 35.
        """
        _d = self.distance(nnear, nrand)
        if _d > d_max:
            _u = d_max / _d
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            (p_x, p_y) = (xrand - xnear, yrand - ynear)
            theta = math.atan2(p_y, p_x)
            (_x, _y) = (
                int(xnear + d_max * math.cos(theta)),
                int(ynear + d_max * math.sin(theta)),
            )
            self.remove_node(nrand)
            if abs(_x - self.goal[0]) < d_max and abs(_y - self.goal[1]) < d_max:
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goal_state = nrand
                self.goal_flag = True
            else:
                self.add_node(nrand, _x, _y)

    def path_to_goal(self):
        if self.goal_flag:
            self.path = []
            self.path.append(self.goal_state)
            new_pos = self.parent[self.goal_state]
            while new_pos != 0:
                self.path.append(new_pos)
                new_pos = self.parent[new_pos]
            self.path.append(0)
        return self.goal_flag

    def get_path_coords(self):
        """returns the coordinates of the defined path"""
        path_coords = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            path_coords.append((x, y))
        return path_coords

    def bias(self, ngoal):
        """defines bias to goal

        Args:
            ngoal (int): number of goal node

        Returns:
            int, int, int: the numbers corresponding to a node
        """
        _n = self.number_of_nodes()
        self.add_node(_n, ngoal[0], ngoal[1])
        nnear = self.nearest(_n)
        self.step(nnear, _n)
        self.connect(nnear, _n)
        return self.x, self.y, self.parent

    def optimize_edges(self):
        n = self.number_of_nodes() - 1  # cantidad de nodos
        neighbor_node_list = []
        for i in range(0, n):
            if self.distance(i, n) < self.neighbor:
                neighbor_node_list.append(i)
        min_index = neighbor_node_list.index(min(neighbor_node_list))
        del neighbor_node_list[min_index]

        cost_new_node = self.cost(n)
        for i in neighbor_node_list:
            (_x1, _y1) = (self.x[n], self.y[n])
            (_x2, _y2) = (self.x[i], self.y[i])
            if not self.cross_obstacle(_x1, _x2, _y1, _y2):
                last_cost = self.cost(i)
                temp_cost = cost_new_node + self.distance(n, i)
                if temp_cost < last_cost:
                    self.remove_edge(i)
                    self.add_edge(n, i)
        return self.x, self.y, self.parent

    def expand(self):
        """expands nodes

        Returns:
            int, int, int: the numbers corresponding to a node
        """
        _n = self.number_of_nodes()
        _x, _y = self.sample_env()
        self.add_node(_n, _x, _y)
        if self.is_free():
            xnearest = self.nearest(_n)
            self.step(xnearest, _n)
            self.connect(xnearest, _n)
        return self.x, self.y, self.parent

    def cost(self, n):
        """computes cost for different obtained paths"""
        ninit = 0
        n = n
        parent = self.parent[n]
        c = 0
        while n is not ninit:
            c = c + self.distance(n, parent)
            n = parent
            if n is not ninit:
                parent = self.parent[n]
        return c
