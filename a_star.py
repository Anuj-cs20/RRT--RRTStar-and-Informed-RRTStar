import numpy as np
import sys
import matplotlib.pyplot as plt


def heuristic(start, goal):
	# Euclidean distance between start and goal
	dist = 0
	# try:
	dist = np.sqrt((start[0] - goal[0]) ** 2 + (start[1] - goal[1]) ** 2)
	# except:
	#     # Print the error
	#     print("Error is ", sys.exc_info()[0])
	#     print("start ",start,"goal ",goal)
	return dist


def a_star(space, start, goal):
	plt.imshow(space, cmap='Greys', origin='lower')
	# Plot the start and goal
	plt.plot(start[0], start[1], 'ro') 
	plt.plot(goal[0], goal[1], 'bo') 
	# Initialize open and closed lists
	open_list = []
	closed_list = []

	# Add the start node to the open list
	open_list.append(start)
	# if the size of list is not 1
	# While the open list is not empty
  
	while open_list:
		print("Test")
		# Get the node with the lowest f cost from the open list
		current = open_list[0]
		for node in open_list:
			if node[2]!=None and current[2]!=None and node[3]!=None and current[3]!=None:
				if node[2] + node[3] < current[2] + current[3]:
					current = node

		# Remove the current node from the open list
		open_list.remove(current)
		# Plot the current node as point
		plt.plot(current[0], current[1], 'ro')
		plt.pause(0.0001)
		# Add the current node to the closed list
		closed_list.append(current)

		# If the current node is the goal, print that the path is found
		if current[0] == goal[0] and current[1] == goal[1]:
			print("Path found")
			return current

		# For each neighbor of the current node
		for neighbor in [
			(current[0] + 1, current[1]),
			(current[0] - 1, current[1]),
			(current[0], current[1] + 1),
			(current[0], current[1] - 1),
		]:
			# If the neighbor is not in the closed list and is walkable(white)
			if neighbor not in closed_list and space[neighbor[0]][neighbor[1]] == 255:
				# Calculate the g cost for the neighbor
				g = current[2] + 1

				# Calculate the h cost for the neighbor
				h = heuristic(neighbor, (goal[0], goal[1]))
				if h is None:
					print("h is none")
					print(neighbor,goal)
				# Calculate the f cost for the neighbor
				f = g + h

				# Create a new node for the neighbor
				new_node = (
					neighbor[0],
					neighbor[1],
					g,
					h,
					current,
				)  # Node = (x, y, g, h, parent)

				# If the neighbor is not in the open list, add it to the open list
				if new_node not in open_list:
					open_list.append(new_node)

	# If there is no path, return None
	return None


def main():
	# Load the space
	space = np.load("map0.npy")

	# Set the start and goal coordinates along with heuristic
	# node = (x, y, g, h, parent)
	start = (25, 300, 0, heuristic((25, 300), (370, 125)), (25,300))
	#  plot the point on the map
	goal = (35, 310, 0, heuristic((370, 125), (370, 125)), None)
	# Find the path from the start to the goal
	path = a_star(space, start, goal)

	# If there is a path, plot it
	if path is None:
		print("No path found")

if __name__ == "__main__":
	main()
