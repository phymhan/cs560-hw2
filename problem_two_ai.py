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
from Gazebo import Gazebo
import time
import tf
from scipy.stats import mode

random.seed(0)
np.random.seed(0)

MIN_SPEED = 0.1
MAX_SPEED = 2
MAX_ANGLE = 10
MIN_DURATION = 1
MAX_DURATION = 3
CARLEN = 3
Z_VALUE = 0.1
TOL_GOAL = 2
MAX_NUM_DEGREE = 5


def voronoi_dist(p1, p2):
    x2 = (p1[0]-p2[0]) ** 2
    y2 = (p1[1]-p2[1]) ** 2
    a2 = min((p1[2]-p2[2]) ** 2, (p1[2]+2*math.pi-p2[2]) ** 2)
    return math.sqrt(x2+y2+a2)

class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea,
                 goalSampleRate=10, maxIter=400, star=True,
                 curvature=1, step_size=0.1, agent=None, opt=None):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(start[0], start[1], start[2])
        self.end = Node(goal[0], goal[1], goal[2])
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
        self.agent = agent
        self.opt = opt

    def Planning(self, animation=True):
        """
        Pathplanning

        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        for i in range(self.maxIter):
            # rnd, nind = self.get_voronoi_point()
            nind = len(self.nodeList)-1
            rnd = None
            self.DrawGraph(rnd=rnd, nind=nind)
            currState = self.get_state_from_index(nind)
            control = self.get_ai_control(nind)
            if control is None:
                control = self.sample_control(self.get_state_from_index(nind), self.get_state_from_node(rnd))
            print('sampled control: (%.1f, %.1f, %.1f)' % (control[0], control[1], control[2]))
            new_rnd = self.perform_control(currState, control)

            # add an edge: nodeList[nind] -> new_rnd
            newNode = self.steer(new_rnd, nind)

            if newNode is None:
                continue
            
            # add new node to the tree, and check if goal is reached
            if self.expand_tree(newNode):
                print('======= GOAL =======')
                # force stop
                self.agent.action(0, 0, 1)
                break
            
            # HACK
            self.save_tree()

        # generate coruse
        lastIndex = self.GetNearestListIndex(self.nodeList, self.end)
        if lastIndex is None:
            return None
        control, path = self.gen_final_course(lastIndex)
        return control, path
    
    def expand_tree(self, newNode):
        self.nodeList.append(newNode)
        print('--> new node inserted')
        
        # check if goal is reached
        d2 = (self.end.x - newNode.x) ** 2 + (self.end.y - newNode.y) ** 2
        return d2 < TOL_GOAL ** 2

    def get_state_from_index(self, nind):
        node = self.nodeList[nind]
        return [node.x, node.y, Z_VALUE], node.yaw
    
    def get_state_from_node(self, rnd):
        return [rnd.x, rnd.y, Z_VALUE], rnd.yaw

    def sample_control(self, currState, rndState):
        # node : node
        # state: (xyz, yaw)
        if random.random() < 0.1 and rndState != None:
            return self.calc_control(currState, rndState)
        # speed = random.uniform(-MAX_SPEED, MAX_SPEED)
        speed = random.uniform(MIN_SPEED, MAX_SPEED)
        if random.random() < 0.5:
            speed = -speed
        angle = random.uniform(-MAX_ANGLE, MAX_ANGLE)
        angle = np.deg2rad(angle)
        duration = random.uniform(MIN_DURATION, MAX_DURATION)
        
        # if random.random() < 0.5 and (currState[0][0]-self.end.x)**2 + (currState[0][1]-self.end.y)**2 < 3.5**2:
        #     return self.calc_control(currState, ([self.end.x, self.end.y, Z_VALUE], 0))

        return speed, angle, duration
    
    def calc_control(self, srcState, tarState):
        directPathAngle = math.atan2(tarState[0][1]-srcState[0][1], tarState[0][0]-srcState[0][0])
        print('--------------------------')
        print('direct angle: %.1f' % np.rad2deg(directPathAngle))
        # directPathAngle = self.pi_2_pi(directPathAngle)
        speed = random.uniform(0.1, MAX_DURATION)
        angle = directPathAngle - srcState[1]
        duration = math.sqrt((tarState[0][1]-srcState[0][1])**2+(tarState[0][0]-srcState[0][0])**2)/speed
        duration = min(duration, MAX_DURATION)
        if abs(angle) > math.pi/2:
            print('Ah')
            speed *= -1
            if angle > 0:
                angle = math.pi-abs(angle)
            else:
                angle = abs(angle)-math.pi
        if abs(angle) > MAX_ANGLE:
            angle = MAX_ANGLE if angle > 0 else -MAX_ANGLE
        return speed, angle/2, duration
    
    def euler2quart(self, euler):
        return tf.transformations.quaternion_from_euler(*euler)
    
    def set_state(self, state):
        euler = (0, 0, state[1])
        quart = self.euler2quart(euler)
        self.agent.setState(state[0], quart)
    
    def force_set_state(self, state):
        for _ in range(10):
            self.set_state(state)
            self.agent.action(0, 0, 0.1)
            time.sleep(0.1)
    
    def get_state(self):
        return self.agent.getState()

    def perform_control(self, state, control):
        print('setting state...')
        # first, set state
        self.set_state(state)
        # self.agent.setState([5,5,0], quart)
        # time.sleep(5)
        print('setting done.')
        # currState = self.agent.getState()
        # print('confirm state: ([%.1f, %.1f, %.1f], %.1f)' % (currState[0][0], currState[0][1], currState[0][2], np.rad2deg(currState[1])))
        print('action')
        # print('setting done. performing action...')
        # then, action
        self.agent.action(*control)
        time.sleep(control[2])
        # time.sleep(MAX_DURATION)
        print('action done.')
        self.agent.action(0, 0, 0.1)

        # get new state
        state_new = self.agent.getState()
        # print('got new state')
        # print(state_new)
        new_node = Node(state_new[0][0], state_new[0][1], state_new[1])
        new_node.control = control
        return new_node
    
    # def calc_control(self, srcNode, tarNode):
    #     directPathAngle = math.atan2(tarNode.y-srcNode.y, tarNode.x-srcNode.x)
    #     directPathAngle = self.pi_2_pi(directPathAngle)
    #     speed = 1
    #     angle = srcNode.yaw - directPathAngle
    #     return speed, angle, math.sqrt((tarNode.y-srcNode.y)**2+(tarNode.x-srcNode.x)**2)/speed

    # def choose_parent(self, newNode, nearinds):
    #     if len(nearinds) == 0:
    #         return newNode

    #     dlist = []
    #     for i in nearinds:
    #         tNode = self.steer(newNode, i)
    #         if tNode is None:
    #             continue

    #         if self.CollisionCheck(tNode, self.obstacleList):
    #             dlist.append(tNode.cost)
    #         else:
    #             dlist.append(float("inf"))

    #     mincost = min(dlist)
    #     minind = nearinds[dlist.index(mincost)]

    #     if mincost == float("inf"):
    #         print("mincost is inf")
    #         return newNode

    #     newNode = self.steer(newNode, minind)

    #     return newNode

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def steer(self, rnd, nind):
        # # old
        # nearestNode = self.nodeList[nind]

        # px, py, pyaw, mode, clen = reeds_shepp_path_planning.reeds_shepp_path_planning(
        #     nearestNode.x, nearestNode.y, nearestNode.yaw, rnd.x, rnd.y, rnd.yaw, self.curvature, self.step_size)

        # if px is None:
        #     return None

        # newNode = copy.deepcopy(nearestNode)
        # newNode.x = px[-1]
        # newNode.y = py[-1]
        # newNode.yaw = pyaw[-1]

        # newNode.path_x = px
        # newNode.path_y = py
        # newNode.path_yaw = pyaw
        # newNode.cost += sum([abs(c) for c in clen])
        # newNode.parent = nind

        # new
        srcNode = self.nodeList[nind]
        tarNode = rnd

        newNode = copy.deepcopy(rnd)
        newNode.cost = srcNode.cost + newNode.control[0]*newNode.control[2]
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
    
    def get_voronoi_point(self):
        # # generate lots of points
        # N = 1000
        # points = np.random.rand(N, 2)
        # points *= np.array([self.maxrand_x-self.minrand_x, self.maxrand_y-self.minrand_y])
        # points += np.array([self.minrand_x, self.minrand_y])
        # vor = []
        # for n in range(N):
        #     dlist = [(points[n][0]-node.x)**2 + (points[n][1]-node.y) for node in self.nodeList]
        #     vor.append(dlist.index(min(dlist)))
        # # plt.scatter(points[:,0],points[:,1],c=vor)
        # # plt.show()
        # pind = random.choice(range(N))
        # # nind = mode(vor)[0]
        # nind = vor[pind]
        # rnd = Node(points[pind][0], points[pind][1], 0)
        # return rnd, nind

        # generate lots of points
        N = 1000
        points = np.random.rand(N, 3)
        points *= np.array([self.maxrand_x-self.minrand_x, self.maxrand_y-self.minrand_y, 2*math.pi])
        points += np.array([self.minrand_x, self.minrand_y, -math.pi])
        vor = []
        for n in range(N):
            dlist = [voronoi_dist(points[n], node.get_state()) for node in self.nodeList]
            vor.append(dlist.index(min(dlist)))
        # plt.scatter(points[:,0],points[:,1],c=vor)
        # plt.show()
        if self.opt.greedy_voronoi:
            dgoal = np.array([voronoi_dist(p, self.end.get_state()) for p in points])
            p = -dgoal
            p = p-p.min()+20
            p = p / p.sum()
            pind = np.random.choice(range(N), p=p)
        else:
            pind = random.choice(range(N))

        # nind = mode(vor)[0]
        nind = vor[pind]
        rnd = Node(points[pind][0], points[pind][1], 0)
        return rnd, nind

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
        # # old
        # path = [[self.end.x, self.end.y]]
        # while self.nodeList[goalind].parent is not None:
        #     node = self.nodeList[goalind]
        #     for (ix, iy) in zip(reversed(node.path_x), reversed(node.path_y)):
        #         path.append([ix, iy])
        #     #  path.append([node.x, node.y])
        #     goalind = node.parent
        # path.append([self.start.x, self.start.y])
        # return path

        # new
        rootInd = None
        nodeList = []
        while self.nodeList[goalind].parent != rootInd:
            node = self.nodeList[goalind]
            nodeList.append(node)
            goalind = node.parent
        controls = []
        paths = [[self.start.x, self.start.y, self.start.yaw]]  # [[x, y, yaw]]
        for node in reversed(nodeList):
            controls.append(node.control)
            paths.append([node.x, node.y, node.yaw])
        return controls, paths

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeList)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        #  r = self.expandDis * 5.0
        dlist = [(node.x - newNode.x) ** 2 +
                 (node.y - newNode.y) ** 2 +
                 (node.yaw - newNode.yaw) ** 2 * 0
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

    def DrawGraph(self, rnd=None, nind=None):
        """
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        if nind is not None:
            nnode = self.nodeList[nind]
            plt.plot(nnode.x, nnode.y, "^r")
        for node in self.nodeList:
            if node.parent is not None:
                # plt.plot(node.path_x, node.path_y, "-g")
                pNode = self.nodeList[node.parent]
                plt.plot([pNode.x, node.x], [pNode.y, node.y], "-g")
                #  plt.plot([node.x, self.nodeList[node.parent].x], [
                #  node.y, self.nodeList[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "ok", ms=30 * size)

        reeds_shepp_path_planning.plot_arrow(
            self.start.x, self.start.y, self.start.yaw)
        reeds_shepp_path_planning.plot_arrow(
            self.end.x, self.end.y, self.end.yaw)

        plt.axis([self.minrand_x, self.maxrand_x, self.minrand_y, self.maxrand_y])
        plt.grid(True)
        plt.pause(0.01)

        #  plt.show()
        #  input()

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd.x) ** 2 +
                 (node.y - rnd.y) ** 2 +
                 (node.yaw - rnd.yaw) ** 2 for node in nodeList]
        # dlist = [(node.x - rnd.x) ** 2 +
        #          (node.y - rnd.y) ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))

        return minind
    
    def avoid_dead_end(self, nodeList, nind, rnd):
        control = None
        degrees = [0 for _ in range(len(nodeList))]
        for node in nodeList:
            if node.parent == None:
                continue
            degrees[node.parent] += 1
        if nind == degrees.index(max(degrees)):
            if nodeList[nind].parent is not None:
                nind = nodeList[nind].parent
            if degrees[nind] > MAX_NUM_DEGREE:
                control = self.get_ai_control(nind)
                return nind, control
            print('might be a dead end, re-select: %d' % nind)
        return nind, control

    # def CollisionCheck(self, node, obstacleList):

    #     for (ox, oy, size) in obstacleList:
    #         for (ix, iy) in zip(node.path_x, node.path_y):
    #             dx = ox - ix
    #             dy = oy - iy
    #             d = dx * dx + dy * dy
    #             if d <= size ** 2:
    #                 return False  # collision

    #     return True  # safe

    def save_tree(self):
        # [parent, x, y, yaw, speed, angle, duration]
        tree = []
        for node in self.nodeList:
            if len(node.control) == 0:
                node.control = (None, None, None)
            tree.append([node.parent, node.x, node.y, node.yaw, node.control[0], node.control[1], node.control[2]])
        np.save(self.opt.tree_filename, tree)
    
    def load_tree(self):
        nodeList = []
        tree = np.load(self.opt.tree_filename)
        for a in tree:
            node = Node(a[1], a[2], a[3])
            node.parent = None if a[0] is None else int(a[0])
            node.control = (a[4], a[5], a[6])
            nodeList.append(node)
        self.nodeList = nodeList
    
    def replay(self, controls, paths):
        print('===========================================')
        print('REPLAY')
        # set init state
        initState = ([self.start.x, self.start.y, Z_VALUE], self.start.yaw)
        self.set_state(initState)
        for control, path in zip(controls, paths[1:]):
            print(control)
            print('--> action: speed %.1f, angle %.1f, duration %.1f' % (control[0], control[1], control[2]))
            self.agent.action(*control)
            time.sleep(control[2])
            self.agent.action(0, 0, 0.1)
            expState = ([path[0], path[1], Z_VALUE], path[2])
            self.set_state(expState)
        print('=== YES! ===')
        # self.stop()
    
    def stop(self):
        self.set_state(self.get_state())
        self.agent.action(0, 0, 60)
    
    def get_ai_control(self, nind=0):
        currState = self.get_state_from_index(nind)
        self.force_set_state(currState)
        print('curr state: ([%.1f, %.1f, %.1f], %.1f)' % (currState[0][0], currState[0][1], currState[0][2], np.rad2deg(currState[1])))
        control = raw_input('enter control (speed, angle, duration):')
        if control == '':
            return None
        control = [float(x) for x in control.split()]
        return control

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
        self.control = []
        self.cost = 0.0
        self.parent = None
    
    def get_state(self):
        return [self.x, self.y, self.yaw]


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
    obstacleList = []  # [x,y,size(radius)]

    # Set Initial parameters
    start = [-8., -6., np.deg2rad(90.)]
    goal = [8., 4., np.deg2rad(0.0)]
    # goal = [-6, 5, np.deg2rad(90)]

    agent = Gazebo()

    rrt = RRT(np.array(start), np.array(goal), randArea=[-9, 10, -7.5, 6.5], obstacleList=obstacleList,
              goalSampleRate=opt.goal_sample_rate, star=not opt.no_star,
              curvature=opt.curvature, step_size=opt.step_size, agent=agent, maxIter=100000, opt=opt)
    
    # path planning
    if not opt.load_and_replay:
        control, path = rrt.Planning(animation=opt.show_animation)
        # save
        rrt.save_tree()
    else:
        print('=-=-=-=-=-=-=-=-= load from npy file')
        rrt.load_tree()
        control, path = rrt.gen_final_course(rrt.GetNearestListIndex(rrt.nodeList, rrt.end))
    
    if len(path) > 1:
        # draw
        rrt.DrawGraph()
        plt.plot([x for (x, y, _) in path], [y for (x, y, _) in path], '-r')
        plt.pause(0.001)
        plt.show()

        # replay
        rrt.replay(control, path)

    if path is None:
        print('Path not found. Maximum # of iterations exceeded.')
        return

    # Draw final path
    # if opt.show_animation:
    #     rrt.DrawGraph()
    #     plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
    #     plt.grid(True)
    #     plt.pause(0.001)
    #     plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--no_star', action='store_true')
    parser.add_argument('--show_animation', action='store_true')
    parser.add_argument('--goal_sample_rate', type=float, default=10)
    parser.add_argument('--curvature', type=float, default=1.0)
    parser.add_argument('--step_size', type=float, default=0.2)
    parser.add_argument('--tree_filename', type=str, default='tree_naive.npy')
    parser.add_argument('--load_and_replay', action='store_true')
    parser.add_argument('--use_voronoi', action='store_true')
    parser.add_argument('--greedy_voronoi', action='store_true')
    opt = parser.parse_args()

    # set defaults for debuging
    opt.no_star = True
    # opt.show_animation = False
    opt.curvature = math.tan(np.deg2rad(MAX_ANGLE)) / CARLEN

    main(opt)
