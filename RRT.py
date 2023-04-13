# Things to import
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random


# treeNode class
class treeNode:
    def __init__(self, locationX, locationY):
        self.locationX = locationX          # X Location
        self.locationY = locationY          # Y Location
        self.children = []  # children List
        self.parent = None  # parent node reference


# RRT Algorithm class
class RRTAlgorithm():
    def __init__(self, start, goal, grid, stepSize):
        # The RRT (root position)
        self.randomTree = treeNode(start[0], start[1])
        self.goal = treeNode(goal[0], goal[1])         # goal position
        self.nearestNode = None  # nearest node
        self.grid = grid  # the map
        self.rho = stepSize  # length of each branch
        self.path_distance = 0  # total path distance
        self.nearestDist = 10000  # distance to neareast node
        self.numWaypoints = 0  # number of waypoints
        self.Waypoints = []  # the waypoints

    # add the point to the nearest node and add goal when reached
    def addChild(self, locationX, locationY):
        if (locationX == self.goal.locationX):
            # add the goal node to the children of the nearest node
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode
        else:
            tempNode = treeNode(locationX, locationY)
            # add tempNode to children of nearest node
            self.nearestNode.children.append(tempNode)
            tempNode.parent = self.nearestNode

    # sample a random point within grid Limits
    def sampleAPoint(self):
        x = random.randint(1, grid.shape[1])
        y = random.randint(1, grid.shape[0])
        point = np.array([x, y])
        return point

    # steer a distance stepsize from start to end Location
    def steerToPoint(self, locationStart, locationEnd):
        offset = self.rho*self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0],
                         locationStart.locationY + offset[1]])
        if point[0] >= grid.shape[1]:
            point[0] = grid.shape[1] - 1
        if point[1] >= grid.shape[0]:
            point[1] = grid.shape[0] - 1
        if point[0] < 0:
            point[0] = 0
        if point[1] < 0:
            point[1] = 0
        return point

    # check if obstacle lies between the start and end point of the edge
    def isInObstacle(self, locationStart, locationEnd):
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0.0, 0.0])
       
        for i in range(self.rho):
            testPoint[0] = locationStart.locationX + i*u_hat[0]
            testPoint[1] = locationStart.locationY + i*u_hat[1]
            
            y = np.round(testPoint[1]).astype(np.int64)
            x = np.round(testPoint[0]).astype(np.int64)
            if y < 0 or x < 0 or y >= self.grid.shape[0] or x >= self.grid.shape[1] or self.grid[y, x] == 0:
                return True
        return False

    # find unit vector between 2 points which form a vector
    def unitVector(self, locationStart, locationEnd):
        v = np.array([locationEnd[0] - locationStart.locationX,
                     locationEnd[1] - locationStart.locationY])
        u_hat = v/np.linalg.norm(v)
        return u_hat

    # find the nearest node from a given unconnected point (Euclidean distance)
    def findNearest(self, root, point):
        if not root:
            return
        dist = self.distance(root, point)
        if dist <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
        # recursively call by iterating through the childern
        for child in root.children:
            self.findNearest(child, point)

    # find euclidean distance between a node and an XY point
    def distance(self, node1, point):
        dist = np.sqrt(
            (node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)
        return dist

    # check if the goal has been reached within step size
    def goalFound(self, point):
        if self.distance(self.goal, point) <= self.rho:
            return True

    # reset nearestNode and nearest Distance
    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDist = 10000

    # trace the path from goal to start
    def retraceRRTPath(self, goal):
        if goal.locationX == self.randomTree.locationX:
            return
        self.numWaypoints += 1
        currentPoint = np.array([goal.locationX, goal.locationY])
        self.Waypoints.insert(0, currentPoint)
        self.path_distance += self.rho
        self.retraceRRTPath(goal.parent)


# Load the grid, set start and goal <x, y> positions, number of iterations, step size
grid = np.load('map1.npy')
print("Dimensions of the grid: ", grid.shape)
fig = plt.figure("RRT Algorithm")
plt.imshow(grid, cmap='gray')
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')
# plt.show()

# x, y = map(int, input("Enter Start point coordinates: ").split())
start = np.array([25.0, 300.0])
plt.plot(start[0], start[1], 'ro')

# [370.0, 125.0]
# x, y = map(int, input("Enter End point coordinates: ").split())
goal = np.array([370.0, 125.0])
plt.plot(goal[0], goal[1], 'bo')

# 20
# stepSize = int(input("Enter step size: "))
stepSize = 20
goalRegion = patches.Circle(
    (goal[0], goal[1]), stepSize, color='b', fill=False)
ax = fig.gca()
ax.add_patch(goalRegion)

# plt.show()
# Code
rrt = RRTAlgorithm(start, goal, grid, stepSize)
totalNodes = 0
totalIterations = 0
while (1):
    n = int(input("Number of nodes to be added: "))
    if n == 0:
        break
    else:
        i = n
        while (i):
            totalIterations += 1
            # Reset nearest values
            rrt.resetNearestValues()
            # algo begins
            point = rrt.sampleAPoint()
            rrt.findNearest(rrt.randomTree, point)
            new = rrt.steerToPoint(rrt.nearestNode, point)
            bool = rrt.isInObstacle(rrt.nearestNode, new)
            i = i - 1
            if (bool == False):
                totalNodes += 1
                rrt.addChild(new[0], new[1])
                plt.pause(0.10)
                plt.plot([rrt.nearestNode.locationX, new[0]], [
                    rrt.nearestNode.locationY, new[1]], 'go', linestyle='-')
                # if goal found, append the path
                if (rrt.goalFound(new)):
                    rrt.addChild(goal[0], goal[1])
                    print("\nGoal Found!")
                    break
            else:
                i = i + 1

# trace back path returned, and add start to waypoints
rrt.retraceRRTPath(rrt.goal)
rrt.Waypoints.insert(0, start)
print("Total Number of Nodes added: ", totalNodes)
print("Total Number of Iterations: ", totalIterations)
print("Number of waypoints: ", rrt.numWaypoints)
print("Path Distance (m): ", rrt.path_distance)
print("Waypoints: ", rrt.Waypoints)

# plot waypoints
# plot the waypoints
for i in range(len(rrt.Waypoints)-1):
    plt.plot([rrt.Waypoints[i][0], rrt.Waypoints[i+1][0]],
             [rrt.Waypoints[i][1], rrt.Waypoints[i+1][1]], 'ro', linestyle='-')
    plt.pause(0.10)

plt.pause(1.50)
