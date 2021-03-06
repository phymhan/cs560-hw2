"""

Path Planning Sample Code with RRT for car like robot.

author: AtsushiSakai(@Atsushi_twi)

"""

import sys
sys.path.append("ReedsSheppPath")

import random
import math
import copy
import numpy as np
import reeds_shepp_path_planning
import matplotlib.pyplot as plt
import argparse
# from Gazebo import Gazebo

random.seed(0)
np.random.seed(0)


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea,
                 goalSampleRate=10, maxIter=400, star=True,
                 curvature=1, step_size=0.1, sample_control=False,
                 agent=None):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(start[0], start[1], start[2])
        self.end = Node(goal[0], goal[1], goal[2])
        # self.minrand = randArea[0]
        # self.maxrand = randArea[1]
        self.minrand_x = randArea[0]
        self.maxrand_x = randArea[1]
        self.minrand_y = randArea[2]
        self.maxrand_y = randArea[3]
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList
        self.star = star
        self.curvature = curvature
        self.step_size = step_size
        self.sample_control = sample_control

    def Planning(self, animation=True):
        """
        Pathplanning

        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        for i in range(self.maxIter):
            rnd = self.get_random_point()
            if self.sample_control:
                # sample a new rnd
                nind = self.GetNearestListIndex(self.nodeList, rnd)
                rnd = self.sample_via_control(self.nodeList[nind])
            nind = self.GetNearestListIndex(self.nodeList, rnd)

            newNode = self.steer(rnd, nind)
            if newNode is None:
                continue

            if self.CollisionCheck(newNode, self.obstacleList):
                nearinds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode, nearinds)
                if newNode is None:
                    continue
                self.nodeList.append(newNode)
                if self.star:
                    self.rewire(newNode, nearinds)

            if animation and i % 5 == 0:
                self.DrawGraph(rnd=rnd)

        # generate coruse
        lastIndex = self.get_best_last_index()
        if lastIndex is None:
            return None
        path = self.gen_final_course(lastIndex)
        return path
    
    def sample_via_control(self, node):
        kappa = random.uniform(-self.curvature, self.curvature)
        arclen = random.random() * 1
        x, y, yaw = node.x, node.y, node.yaw
        if abs(kappa) < 0.001:
            # straight line
            nx = x + arclen*math.cos(yaw)
            ny = y + arclen*math.sin(yaw)
        else:
            R = 1 / abs(kappa)
            angle = arclen / R
            da = 0
            if kappa < 0:
                ox = x + R*math.cos(yaw-math.pi/2)
                oy = y + R*math.sin(yaw-math.pi/2)
                a = math.pi + (yaw - math.pi/2) - angle
                da = -angle
            else:
                ox = x + R*math.cos(yaw+math.pi/2)
                oy = y + R*math.sin(yaw+math.pi/2)
                a = math.pi - (yaw + math.pi/2) + angle
                da = angle
            nx = ox + R*math.cos(a)
            ny = oy + R*math.sin(a)
        return Node(nx, ny, self.pi_2_pi(yaw+da))

    def choose_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:
            tNode = self.steer(newNode, i)
            if tNode is None:
                continue

            if self.CollisionCheck(tNode, self.obstacleList):
                dlist.append(tNode.cost)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return newNode

        newNode = self.steer(newNode, minind)

        return newNode

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def steer(self, rnd, nind):

        nearestNode = self.nodeList[nind]

        px, py, pyaw, mode, clen = reeds_shepp_path_planning.reeds_shepp_path_planning(
            nearestNode.x, nearestNode.y, nearestNode.yaw, rnd.x, rnd.y, rnd.yaw, self.curvature, self.step_size)

        if px is None:
            return None

        newNode = copy.deepcopy(nearestNode)
        newNode.x = px[-1]
        newNode.y = py[-1]
        newNode.yaw = pyaw[-1]

        newNode.path_x = px
        newNode.path_y = py
        newNode.path_yaw = pyaw
        newNode.cost += sum([abs(c) for c in clen])
        newNode.parent = nind

        return newNode

    def get_random_point(self):

        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [random.uniform(self.minrand_x, self.maxrand_x),
                   random.uniform(self.minrand_y, self.maxrand_y),
                   random.uniform(-math.pi, math.pi)
                   ]
        else:  # goal point sampling
            rnd = [self.end.x, self.end.y, self.end.yaw]

        node = Node(rnd[0], rnd[1], rnd[2])

        return node

    def get_best_last_index(self):
        #  print("get_best_last_index")

        YAWTH = np.deg2rad(3.0)
        XYTH = 0.5

        goalinds = []
        for (i, node) in enumerate(self.nodeList):
            if self.calc_dist_to_goal(node.x, node.y) <= XYTH:
                goalinds.append(i)
        #  print("OK XY TH num is")
        #  print(len(goalinds))

        # angle check
        fgoalinds = []
        for i in goalinds:
            if abs(self.nodeList[i].yaw - self.end.yaw) <= YAWTH:
                fgoalinds.append(i)
        #  print("OK YAW TH num is")
        #  print(len(fgoalinds))

        if len(fgoalinds) == 0:
            return None

        mincost = min([self.nodeList[i].cost for i in fgoalinds])
        for i in fgoalinds:
            if self.nodeList[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind):
        path = [[self.end.x, self.end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            for (ix, iy) in zip(reversed(node.path_x), reversed(node.path_y)):
                path.append([ix, iy])
            #  path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeList)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        #  r = self.expandDis * 5.0
        dlist = [(node.x - newNode.x) ** 2 +
                 (node.y - newNode.y) ** 2 +
                 (node.yaw - newNode.yaw) ** 2
                 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    def rewire(self, newNode, nearinds):

        nnode = len(self.nodeList)

        for i in nearinds:
            nearNode = self.nodeList[i]
            tNode = self.steer(nearNode, nnode - 1)
            if tNode is None:
                continue

            obstacleOK = self.CollisionCheck(tNode, self.obstacleList)
            imporveCost = nearNode.cost > tNode.cost

            if obstacleOK and imporveCost:
                #  print("rewire")
                self.nodeList[i] = tNode

    def DrawGraph(self, rnd=None):
        """
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot(node.path_x, node.path_y, "-g")
                #  plt.plot([node.x, self.nodeList[node.parent].x], [
                #  node.y, self.nodeList[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "ok", ms=30 * size)

        reeds_shepp_path_planning.plot_arrow(
            self.start.x, self.start.y, self.start.yaw)
        reeds_shepp_path_planning.plot_arrow(
            self.end.x, self.end.y, self.end.yaw)

        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

        #  plt.show()
        #  input()

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd.x) ** 2 +
                 (node.y - rnd.y) ** 2 +
                 (node.yaw - rnd.yaw) ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))

        return minind

    def CollisionCheck(self, node, obstacleList):

        for (ox, oy, size) in obstacleList:
            for (ix, iy) in zip(node.path_x, node.path_y):
                dx = ox - ix
                dy = oy - iy
                d = dx * dx + dy * dy
                if d <= size ** 2:
                    return False  # collision

        return True  # safe


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.path_x = []
        self.path_y = []
        self.path_yaw = []
        self.cost = 0.0
        self.parent = None


def main(opt):
    print("Start rrt start planning")

    # ====Search Path with RRT====
    #  obstacleList = [
    #  (5, 5, 1),
    #  (3, 6, 2),
    #  (3, 8, 2),
    #  (3, 10, 2),
    #  (7, 5, 2),
    #  (9, 5, 2)
    #  ]  # [x,y,size(radius)]
    obstacleList = [
        (5, 5, 1),
        (4, 6, 1),
        (4, 8, 1),
        (4, 10, 1),
        (6, 5, 1),
        (7, 5, 1),
        (8, 6, 1),
        (8, 8, 1),
        (8, 10, 1)
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    start = [0.0, 0.0, np.deg2rad(90.0)]
    goal = [10.0, 2.0, np.deg2rad(0.0)]

    # agent = Gazebo()

    rrt = RRT(start, goal, randArea=[-2.0, 15.0, -2, 15], obstacleList=obstacleList,
              goalSampleRate=opt.goal_sample_rate, star=not opt.no_star,
              curvature=opt.curvature, step_size=opt.step_size, sample_control=opt.sample_control,
              )
    path = rrt.Planning(animation=opt.show_animation)

    if path is None:
        print('Path not found. Maximum # of iterations exceeded.')
        return

    # Draw final path
    if opt.show_animation:
        rrt.DrawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--no_star', action='store_true')
    parser.add_argument('--show_animation', action='store_true')
    parser.add_argument('--goal_sample_rate', type=float, default=10)
    parser.add_argument('--curvature', type=float, default=1.0)
    parser.add_argument('--step_size', type=float, default=0.1)
    parser.add_argument('--sample_control', action='store_true')
    opt = parser.parse_args()
    main(opt)
