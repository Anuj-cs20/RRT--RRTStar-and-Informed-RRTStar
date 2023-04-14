import numpy as np
import sys
import matplotlib.pyplot as plt


def heuristic(start, goal):
    # Euclidean distance between start and goal
    dist = 0
    print("start ",start,"goal ",goal)
    # try:
    dist= np.sqrt((start[0] - goal[0]) ** 2 + (start[1] - goal[1]) ** 2)
    # except:
    #     # Print the error
    #     print("Error is ", sys.exc_info()[0])
    #     print("start ",start,"goal ",goal)
    # return dist


def a_star(space, start, goal):
    # Initialize open and closed lists
    open_list = []
    closed_list = []

    # Add the start node to the open list
    open_list.append(start)

    # While the open list is not empty
    while open_list:
        # Get the node with the lowest f cost from the open list
        current = open_list[0]
        for node in open_list:
            if node[2] + node[3] < current[2] + current[3]:
                current = node
                
        # Remove the current node from the open list
        open_list.remove(current)

        # Add the current node to the closed list
        closed_list.append(current)

        # If the current node is the goal, return the path
        if current[0] == goal[0] and current[1] == goal[1]:
            path = []
            while current != start:
                path.append(current)
                current = current[3]
            path.append(start)
            return path[::-1]

        # For each neighbor of the current node
        for neighbor in [
            (current[0] + 1, current[1]),
            (current[0] - 1, current[1]),
            (current[0], current[1] + 1),
            (current[0], current[1] - 1),
        ]:
            # If the neighbor is not in the closed list and is walkable
            if neighbor not in closed_list and space[neighbor[0]][neighbor[1]] == 1:
                # Calculate the g cost for the neighbor
                g = current[2] + 1

                # Calculate the h cost for the neighbor
                h = heuristic(neighbor, goal)

                # Calculate the f cost for the neighbor
                f = g + h

                # Create a new node for the neighbor
                new_node = (neighbor[0], neighbor[1], g, h, current) # Node = (x, y, g, h, parent)

                # If the neighbor is not in the open list, add it to the open list
                if new_node not in open_list:
                    open_list.append(new_node)

    # If the open list is empty, there is no path from the start to the goal
    return None


def main():
    # Load the space
    space = np.load("cspace.npy")

    # Set the start and goal coordinates along with heuristic
    #node = (x, y, g, h, parent)
    start = (100, 100, 0, heuristic((100, 100), (600, 250)),None)
    goal = (600, 250, 0, heuristic((600, 250), (600, 250)),None)
    # Find the path from the start to the goal
    path = a_star(space, start, goal)

    # If there is a path, plot it
    if path is not None:
        plt.imshow(space, cmap="gray")
        for node in path:
            plt.plot(node[0], node[1], "o")
        plt.show()


if __name__ == "__main__":
    main()
