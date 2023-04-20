# Import necessary libraries
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class Node:
    def __init__(self, x, y):
        self.locationX = x
        self.locationY = y
        self.parent = None
        self.cost = 0.0

class InformedRRTStarAlgorithm:
    def __init__(self, start, goal, grid, step_size, radius, heuristic):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.grid = grid
        self.step_size = step_size
        self.radius = radius
        self.heuristic = heuristic
        self.nodes = [self.start]
        self.num_waypoints = 0
        self.waypoints = []
        self.path_distance = 0.0

    # Helper function to calculate Euclidean distance between two nodes
    def distance(self, node1, node2):
        return np.sqrt((node1.locationX - node2.locationX) ** 2 +
                       (node1.locationY - node2.locationY) ** 2)

    # Helper function to check if a node is in collision (inside an obstacle)
    def is_in_collision(self, node):
        x = int(node.locationX)
        y = int(node.locationY)
        if self.grid[x, y] == 1:
            return True
        else:
            return False

    # Helper function to sample a point uniformly within the search space
    def sample_uniform_point(self):
        x = np.random.uniform(0, self.grid.shape[0])
        y = np.random.uniform(0, self.grid.shape[1])
        return Node(x, y)

    # Helper function to sample a point based on the ellipsoidal heuristic
    def sample_informed_point(self):
        # Define the ellipsoidal heuristic based on the current best solution (goal node)
        ellipsoidal_radius_x = self.heuristic * self.step_size
        ellipsoidal_radius_y = self.heuristic * self.step_size

        # Sample a point within the ellipsoidal heuristic region
        theta = np.random.uniform(0, 2 * np.pi)
        r = np.random.uniform(0, 1)
        x = self.goal.locationX + ellipsoidal_radius_x * r * np.cos(theta)
        y = self.goal.locationY + ellipsoidal_radius_y * r * np.sin(theta)

        return Node(x, y)

    # Helper function to find the nearest node to a given point
    def nearest_node(self, point):
        min_distance = float('inf')
        nearest_node = None
        for node in self.nodes:
            d = self.distance(node, point)
            if d < min_distance:
                min_distance = d
                nearest_node = node
        return nearest_node

    # Helper function to steer from a node towards a given point
    def steer_towards_point(self, node, point):
        d = self.distance(node, point)
        if d <= self.step_size:
            return point
        else:
            theta = np.arctan2(point.locationY - node.locationY,
                               point.locationX - node.locationX)
            x = node.locationX + self.step_size * np.cos(theta)
            y = node.locationY + self.step_size * np.sin(theta)
            return Node(x, y)

    # Helper function to calculate the cost from the start node to a given node

    def calculate_cost(self, node):
        return node.cost + self.distance(node, self.start)

    # Helper function to rewire the tree by considering nearby nodes


    def rewire(self, new_node):
        nearby_nodes = [node for node in self.nodes if self.distance(
            node, new_node) <= self.radius]
        for node in nearby_nodes:
            cost = self.calculate_cost(new_node)
            if node.cost > cost and not self.is_in_collision(new_node):
                node.parent = new_node
                node.cost = cost

# Main function to run the Informed RRT* algorithm


    def run(self, max_iterations):
        for i in range(max_iterations):
            # Sample a point either uniformly or based on the ellipsoidal heuristic
            if np.random.rand() < 0.1:  # 10% chance of sampling from the goal node
                sample_node = self.goal
            else:
                sample_node = self.sample_informed_point()

            # Find the nearest node to the sampled point
            nearest_node = self.nearest_node(sample_node)

            # Steer from the nearest node towards the sampled point
            new_node = self.steer_towards_point(nearest_node, sample_node)

            # Check if the new node is in collision
            if not self.is_in_collision(new_node):
                # Calculate the cost of the new node
                new_node.cost = self.calculate_cost(
                    nearest_node) + self.distance(nearest_node, new_node)

                # Find nearby nodes and rewire the tree
                self.rewire(new_node)

                # Add the new node to the tree
                self.nodes.append(new_node)

                # Update the best solution if the new node is closer to the goal
                if self.distance(new_node, self.goal) <= self.step_size and not self.is_in_collision(new_node):
                    self.goal.parent = new_node
                    self.goal.cost = self.calculate_cost(new_node)
                    self.path_distance = self.goal.cost
                    self.num_waypoints = 0
                    self.waypoints = []
                    while self.goal is not None:
                        self.waypoints.append(
                            (self.goal.locationX, self.goal.locationY))
                        self.goal = self.goal.parent
                        self.num_waypoints += 1
                    break

    # Helper function to extract the path from the start to the goal
    def extract_path(self):
        if self.goal.parent is not None:
            path = [(self.goal.locationX, self.goal.locationY)]
            parent = self.goal.parent
            while parent is not None:
                path.append((parent.locationX, parent.locationY))
                parent = parent.parent
            self.waypoints = path[::-1]
            self.path_distance = self.goal.cost
            self.num_waypoints = len(self.waypoints)
