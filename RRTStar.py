# Import required libraries
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random


# Define the treeNode class
class treeNode:
    def __init__(self, locationX, locationY):
        self.locationX = locationX          # X Location
        self.locationY = locationY          # Y Location
        self.children = []  # Children List
        self.parent = None  # Parent node reference
        self.cost = 0  # Cost from the start to this node


# Define the RRTStarAlgorithm class
class RRTStarAlgorithm():
    def __init__(self, start, goal, grid, stepSize, radius):
        # The RRT (root position)
        self.start = treeNode(start[0], start[1])
        self.goal = treeNode(goal[0], goal[1])         # Goal position
        self.grid = grid  # The map
        self.rho = stepSize  # Length of each branch
        self.radius = radius  # Radius for searching nearby nodes
        self.path_distance = 0  # Total path distance
        self.numWaypoints = 0  # Number of waypoints
        self.waypoints = []  # The waypoints

    # Add a node to the tree
    def addNode(self, parent, locationX, locationY, cost):
        tempNode = treeNode(locationX, locationY)
        tempNode.parent = parent
        tempNode.cost = cost
        parent.children.append(tempNode)

    # Sample a random point within grid limits
    def sampleAPoint(self):
        x = random.randint(1, self.grid.shape[1])
        y = random.randint(1, self.grid.shape[0])
        point = np.array([x, y])
        return point

    # Steer from start to end location with a maximum step size
    def steerToPoint(self, start, end):
        v = np.array([end[1] - start.locationX, end[0] - start.locationY])
        if np.linalg.norm(v) > self.rho:
            v = self.rho * v / np.linalg.norm(v)
        point = np.array([start.locationX + v[0], start.locationY + v[1]])
        return point

    # Check if obstacle lies between the start and end point of the edge
    def isInObstacle(self, start, end):
        u_hat = self.unitVector(start, end)
        testPoint = np.array([0.0, 0.0])
        for i in range(int(np.linalg.norm(u_hat))):
            testPoint[0] = start.locationX + i * u_hat[0]
            testPoint[1] = start.locationY + i * u_hat[1]
            y = np.round(testPoint[1]).astype(np.int64)
            x = np.round(testPoint[0]).astype(np.int64)
            if y < 0 or x < 0 or y >= self.grid.shape[0] or x >= self.grid.shape[1] or self.grid[y, x] == 0:
                return True
        return False

    # Find unit vector between 2 points which form a vector
    def unitVector(self, start, end):
        v = np.array([end.locationX - start.locationX,
                     end.locationY - start.locationY])
        u_hat = v / np.linalg.norm(v)
        return u_hat

    # Find the nearest node from a given unconnected point (Euclidean distance)
    def nearestNode(self, point, nodeList):
        minDist = float('inf')
        nearestNode = None
        for node in nodeList:
            dist = np.linalg.norm(
                np.array([node.locationX, node.locationY]) - point)
            if dist < minDist:
                minDist = dist
                nearestNode = node
        return nearestNode

    # Check if a node is within the radius of another node
    def isWithinRadius(self, node1, node2):
        dist = np.linalg.norm(np.array(
            [node1.locationX, node1.locationY]) - np.array([node2.locationX, node2.locationY]))
        if dist <= self.radius:
            return True
        else:
            return False

    # Find the cost of reaching a node from the start node
    def costToReachNode(self, node):
        cost = 0
        while node.parent is not None:
            cost += np.linalg.norm(np.array([node.locationX, node.locationY]) - np.array(
                [node.parent.locationX, node.parent.locationY]))
            node = node.parent
        return cost

    # Check if a node is in the goal region
    def isNodeInGoal(self, node):
        dist = np.linalg.norm(np.array(
            [node.locationX, node.locationY]) - np.array([self.goal.locationX, self.goal.locationY]))
        if dist <= self.radius:
            return True
        else:
            return False

    # Update the parent of a node with a new parent
    def updateParent(self, node, newParent):
        node.parent = newParent
        node.cost = self.costToReachNode(node)

    # Rewire the tree with RRT* algorithm
    def rewireTree(self, newNode, nearNodes):
        for nearNode in nearNodes:
            if self.isWithinRadius(nearNode, newNode) and not self.isInObstacle(nearNode, newNode):
                cost = self.costToReachNode(nearNode) + np.linalg.norm(np.array(
                    [nearNode.locationX, nearNode.locationY]) - np.array([newNode.locationX, newNode.locationY]))
                if cost < newNode.cost:
                    self.updateParent(newNode, nearNode)

    # Extract the path from the tree
    def extractPath(self):
        path = []
        node = self.goal
        while node.parent is not None:
            path.append([node.locationX, node.locationY])
            node = node.parent
        path.append([self.start.locationX, self.start.locationY])
        path.reverse()
        self.waypoints = path


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

# 30
# radius = int(input("Enter radius: "))
radius = 10

# plt.show()
# Code
rrt = RRTStarAlgorithm(start, goal, grid, stepSize, radius)
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
            randPoint = rrt.sampleAPoint()
            nearestNode = rrt.nearestNode(randPoint, [rrt.start])
            newNode = treeNode(0, 0)
            newNode.locationX, newNode.locationY = rrt.steerToPoint(
                nearestNode, randPoint)
            i = i - 1
            if not rrt.isInObstacle(nearestNode, newNode):
                nearNodes = [node for node in rrt.start.children if np.linalg.norm(np.array(
                    [node.locationX, node.locationY]) - np.array([newNode.locationX, newNode.locationY])) <= rrt.radius]
                rrt.addNode(nearestNode, newNode.locationX, newNode.locationY, rrt.costToReachNode(nearestNode) + np.linalg.norm(
                    np.array([nearestNode.locationX, nearestNode.locationY]) - np.array([newNode.locationX, newNode.locationY])))
                rrt.rewireTree(newNode, nearNodes)
                if rrt.isNodeInGoal(newNode):
                    rrt.updateParent(rrt.goal, newNode)
                    print("\nGoal Found!")
                    break
                plt.pause(0.10)
                plt.plot([nearestNode.locationX, newNode.locationX], [
                         nearestNode.locationY, newNode.locationY], 'go', linestyle='-')
            else:
                i = i + 1

# trace back path returned, and add start to waypoints
rrt.extractPath()
print("Total Number of Nodes added: ", totalNodes)
print("Total Number of Iterations: ", totalIterations)
print("Number of waypoints: ", rrt.numWaypoints)
print("Path Distance (m): ", rrt.path_distance)
print("Waypoints: ", rrt.waypoints)

# plot waypoints
# plot the waypoints
for i in range(len(rrt.waypoints)-1):
    plt.plot([rrt.waypoints[i][0], rrt.waypoints[i+1][0]],
             [rrt.waypoints[i][1], rrt.waypoints[i+1][1]], 'ro', linestyle='-')
    plt.pause(0.10)

plt.pause(1.50)
