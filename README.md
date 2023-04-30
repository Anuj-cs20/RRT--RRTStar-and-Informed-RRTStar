# Artificial Intelligence Mini Project
## Problem:

Search-based methods are commonly used in artificial intelligence for finding optimal solutions to problems by searching through an ample space of possible solutions. While these methods have been successful in many applications, they do face some challenges.

One of the main challenges faced by search-based methods is the curse of dimensionality. As the number of variables in the problem increases, the search space grows exponentially, making it increasingly difficult to find an optimal solution. This problem is particularly acute in high-dimensional problems such as those found in robotics and control systems.

Another challenge faced by search-based methods is that they can be computationally expensive. Searching through an ample space of possible solutions can take a significant amount of time, and in some cases, may not be feasible in real-time applications. This can limit the effectiveness of these methods in real-world scenarios where quick decision-making is required.

Search-based methods, such as Dijkstra's algorithm or A* search, work well in environments with well-defined search spaces and clear pathfinding objectives. However, they can struggle in high-dimensional spaces, where the search space is vast, complex, poorly understood, and the optimal path is difficult to determine.

Additionally, the challenge is to plan the path of autonomous vehicles or robots in dynamic environments while avoiding obstacles. Search-based algorithms with heuristics, including A* and others, are not suitable for this task as they are designed for static environments and require the algorithm to be restarted if obstacles change.

## Solution:
Sampling-based search algorithms can be more robust than search-based methods with heuristics in some cases, but they have their own set of advantages and limitations. Sampling-based methods, such as Rapidly-exploring Random Trees (RRT), are typically more effective at handling high-dimensional and complex environments, where the state space is too large to be explored exhaustively. They also do not require a well-defined heuristic function to operate.

These methods involve sampling the search space to reduce the number of dimensions that need to be explored. Examples of sampling-based techniques include Monte Carlo methods, particle filters, and random sampling.

Sampling-based methods, such as Rapidly-exploring Random Trees (RRT) and Probabilistic Roadmap (PRM), can reduce the computational cost by exploring only a representative subset of the state space. These methods use random sampling to generate a small set of states and build a graph connecting them. This graph can then be searched to find a path to the goal.

Sampling-based methods such as RRT and PRM can be used to navigate dynamic and uncertain environments where the obstacles are not known beforehand. These methods randomly sample the search space and connect the samples to create a graph that represents the search space. The robot can then navigate in this graph to reach the goal.

Rapidly exploring random trees (RRT) and its variants RRT* and Informed RRT* are sampling-based methods for motion planning that can handle dynamic environments. They work by incrementally building a tree of possible paths through the environment by randomly sampling points and connecting them to the tree. These algorithms can adjust to changes in the environment by dynamically updating the tree rather than restarting the planning process. Additionally, Informed RRT* uses heuristics to guide the search toward the goal, reducing the search space and increasing efficiency.

## Rapidly exploring random tree (RRT):
The RRT (Rapidly-Exploring Random Tree) algorithm is a motion planning algorithm used to find feasible paths for a robot or a mobile agent to navigate through a workspace cluttered with obstacles. It is a probabilistically complete algorithm, which means that it can find a solution if one exists, given enough time and resources.

It is particularly useful when the agent needs to navigate through cluttered environments with obstacles that have irregular shapes and configurations. It is generally used in Robot Path Planning, Autonomous vehicle navigation, Virtual environment exploration and Motion planning for unmanned aerial vehicles (UAVs).

The RRT algorithm has several advantages over other motion planning algorithms, such as A*:
● It can handle high-dimensional spaces: The RRT algorithm can be applied to spaces with high dimensions, such as robotic manipulator configuration spaces.
● It is probabilistically complete: The RRT algorithm is guaranteed to find a solution if one exists, given enough time and resources.
● Itcanhandlecomplexobstacleshapes:TheRRTalgorithmcanhandlecomplex obstacle shapes, as it generates random configurations and extends the tree toward them, which allows it to explore the search space more efficiently.

To perform the RRT algorithm, you start with an initial configuration (e.g., the starting point of a robot) and randomly sample a new configuration in the space. The algorithm then attempts to connect the sampled configuration to the nearest configuration in the existing tree. If the connection is feasible (i.e., it does not collide with obstacles), the new configuration is added to the tree. This process is repeated until a desired goal configuration is reached or a certain number of iterations are completed. The resulting tree represents a connected roadmap of the configuration space, which can be used to find a collision-free path from the initial to the goal configuration.

### Pseudocode:
'''
function RRT(start, goal, max_iter, step_size):
    create an empty tree T with only the start node
    for i = 1 to max_iter do:
        q_rand = randomly generate a configuration in the search space
        q_near = find the node in T that is closest to q_rand
        q_new = extend(q_near, q_rand, step_size)
        if q_new is not None and not in collision:
            T.add(q_new, q_near)
            if q_new is close to goal:
                path = find_path(T, q_new, goal)
                return path
return None

function extend(q_near, q_rand, step_size):
    q_new = move q_near towards q_rand by step_size
    if q_new is in collision:
        return None
    else:
return q_new

function find_path(T, q_start, q_goal):
    path = [q_goal]
    q_curr = q_goal
    while q_curr != q_start:
        q_prev = T.parent(q_curr)
        path.append(q_prev)
        q_curr = q_prev
    path.reverse()
    return path
'''

RRT has various real-time applications in robotics and autonomous systems. For example, RRT can be used for the path planning of drones, autonomous vehicles, and robotic arms. In the context of computer games, RRT can be used to generate realistic character motions or to navigate game agents in dynamic environments. RRT can also be used in other fields, such as virtual reality, computer graphics, and simulation-based training, where real-time motion planning is required.

