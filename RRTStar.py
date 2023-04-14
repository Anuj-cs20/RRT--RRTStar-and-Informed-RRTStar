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
        return tempNode

    # Sample a random point within grid limits
    def sampleAPoint(self):
        x = random.randint(1, self.grid.shape[1])
        y = random.randint(1, self.grid.shape[0])
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

    # Check if obstacle lies between the start and end point of the edge
    def isInObstacle(self, start, end):
        u_hat = self.unitVector(start, end)
        dist = self.distanceNaP(start, end)
        dist = np.round(dist).astype(np.int64)
        for i in range(dist):
            x = start.locationX + i * u_hat[0]
            y = start.locationY + i * u_hat[1]
            y = np.round(y).astype(np.int64)
            x = np.round(x).astype(np.int64)
            if y < 0 or x < 0 or y >= self.grid.shape[0] or x >= self.grid.shape[1] or self.grid[y, x] == 0:
                return True
        return False

    # Find unit vector between 2 points which form a vector
    def unitVector(self, start, end):
        v = np.array([end[0] - start.locationX,
                     end[1] - start.locationY])
        u_hat = v / np.linalg.norm(v)
        return u_hat

    # Find the nearest node from a given unconnected point (Euclidean distance)
    def nearestNode(self, root, point):
        minDist = float('inf')
        nearestNode = None
        stack = [root]  # Use a stack to simulate the recursive function call
        while stack:
            current_node = stack.pop()
            if current_node is not None:
                dist = rrt.distanceNaP(current_node, point)
                if dist <= minDist:
                    minDist = dist
                    nearestNode = current_node
                # Add the children of the current node to the stack
                stack.extend(current_node.children)
        return nearestNode

    # Find the cost of reaching a node from the start node

    def costToReachNode(self, node):
        cost = 0
        while node.parent is not None:
            cost += rrt.distanceNaN(node, node.parent)
            node = node.parent
        return cost

    # Check if a node is in the goal region
    def isNodeInGoal(self, node):
        dist = self.distanceNaN(node, self.goal)
        if dist <= self.radius:
            return True
        else:
            return False

    # Update the parent of a node with a new parent
    def updateParent(self, node, newParent, cost):
        node.parent.children.pop()
        node.parent = newParent
        node.cost = cost
        newParent.children.append(node)

    def updateParentGoal(self, parent, cost):
        self.goal.parent = parent
        self.goal.cost = cost
        newParent.children.append(self.goal)

    # Rewire the tree with RRT* algorithm
    def rewireTree(self, newNode, nearNodes):
        newParent = newNode.parent
        newcost = newNode.cost
        for nearNode in nearNodes:
            point = np.array([newNode.locationX, newNode.locationY])
            if not self.isInObstacle(nearNode, point):
                cost = nearNode.cost + self.distanceNaN(nearNode, newNode)
                if cost < newNode.cost:
                    newParent = nearNode
                    newcost = cost
        self.updateParent(newNode, newParent, newcost)
        return newParent

    # find euclidean distance between a node and an XY point
    def distanceNaP(self, node1, point):
        dist = np.sqrt(
            (node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)
        return dist

    def distanceNaN(self, node1, node2):
        dist = np.sqrt(
            (node1.locationX - node2.locationX)**2 + (node1.locationY - node2.locationY)**2)
        return dist

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
grid = np.load('map0.npy')
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

# 30
# radius = int(input("Enter radius: "))
radius = 40

# 20
# stepSize = int(input("Enter step size: "))
stepSize = 20
goalRegion = patches.Circle(
    (goal[0], goal[1]), radius, color='b', fill=False)
ax = fig.gca()
ax.add_patch(goalRegion)


# plt.show()
# Code
rrt = RRTStarAlgorithm(start, goal, grid, stepSize, radius)
totalNodes = 0
totalIterations = 0
tree = [rrt.start]

while (1):
    n = int(input("Number of nodes to be added: "))
    if n == 0:
        break
    else:
        i = n
        while (i):
            totalIterations += 1
            randPoint = rrt.sampleAPoint()
            nearestNode = rrt.nearestNode(rrt.start, randPoint)
            newPoint = rrt.steerToPoint(nearestNode, randPoint)
            i = i - 1
            if not rrt.isInObstacle(nearestNode, newPoint):
                newNode = rrt.addNode(
                    nearestNode, newPoint[0], newPoint[1], nearestNode.cost + rrt.distanceNaP(nearestNode, newPoint))

                nearNodes = []
                for node in tree:
                    dist = rrt.distanceNaN(node, newNode)
                    if dist <= radius:
                        nearNodes.append(node)

                newParent = rrt.rewireTree(newNode, nearNodes)
                totalNodes += 1
                tree.append(newNode)
                plt.pause(0.10)
                plt.plot([newParent.locationX, newNode.locationX], [
                         newParent.locationY, newNode.locationY], 'go', linestyle='-')
                if rrt.isNodeInGoal(newNode):
                    print("\nGoal Found!")
                    rrt.updateParentGoal(
                        newNode, newNode.cost + rrt.distanceNaN(newNode, rrt.goal))
                    plt.pause(0.10)
                    plt.plot([newNode.locationX, rrt.goal.locationX], [
                        newNode.locationY, rrt.goal.locationY], 'go', linestyle='-')
                    plt.pause(1.0)
                    break
            else:
                i = i + 1

# trace back path returned, and add start to waypoints
rrt.extractPath()
print("\nTotal Number of Nodes added: ", totalNodes)
print("Total Number of Iterations: ", totalIterations)
print("Number of waypoints: ", len(rrt.waypoints))
print("Path Distance (m): ", rrt.goal.cost)
print("Waypoints: ", rrt.waypoints)

# plot waypoints
# plot the waypoints
for i in range(len(rrt.waypoints)-1):
    plt.plot([rrt.waypoints[i][0], rrt.waypoints[i+1][0]],
             [rrt.waypoints[i][1], rrt.waypoints[i+1][1]], 'ro', linestyle='-')
    plt.pause(0.10)

plt.pause(1.50)
