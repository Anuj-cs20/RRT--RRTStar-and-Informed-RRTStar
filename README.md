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
```
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
```

RRT has various real-time applications in robotics and autonomous systems. For example, RRT can be used for the path planning of drones, autonomous vehicles, and robotic arms. In the context of computer games, RRT can be used to generate realistic character motions or to navigate game agents in dynamic environments. RRT can also be used in other fields, such as virtual reality, computer graphics, and simulation-based training, where real-time motion planning is required.

## RRTStar:
RRT* (Rapidly-exploring Random Trees Star) is an extension of the original RRT (Rapidly-exploring Random Trees) algorithm that offers several advantages. First, RRT* aims to find an optimal solution by minimizing the path cost, while RRT only finds a feasible path. Second, RRT* maintains a more balanced and evenly distributed tree structure, which leads to a more efficient exploration of the state space compared to RRT. Lastly, RRT* can efficiently handle changes in the environment, making it more suitable for dynamic environments compared to RRT.

The key difference between RRT and RRT* is that the RRT* algorithm maintains a cost-to-come function at each node in the search tree, which represents the lowest cost path from the start configuration to that node. This cost-to-come function is updated as the tree grows, ensuring that each node in the tree has the lowest possible cost-to-come value. Additionally, the RRT* algorithm re-wires the search tree to improve the cost-to-come value of the nodes, by connecting them to other nodes with lower costs, if such a connection can be made without introducing any collisions with obstacle

### Pseudocode:
```
RRT*(q_start, q_goal, K, delta_q, gamma):
    create a search tree T
    add q_start to T as the root node
    for k = 1 to K do
        q_rand = sample a random configuration in the search space
        q_near = nearest_neighbor(q_rand, T)
        q_new = extend(q_near, q_rand, delta_q)
        if q_new is not null and collision_free(q_near, q_new) then
            T.add_node(q_new)
            near_nodes = near_neighbors(q_new, T, gamma)
            for q_near in near_nodes do
                if collision_free(q_new, q_near) and cost(q_new, q_near) +
cost_to_come(q_new) < cost_to_come(q_near) then
end if end for
    return T
end function
T.remove_edge(parent(q_near), q_near)
T.add_edge(q_new, q_near)
nearest_neighbor(q_rand, T):
    find the node q_near in T that is closest to q_rand
    return q_near
extend(q_near, q_rand, delta_q):
    q_new = interpolate(q_near, q_rand, delta_q)
    if collision_free(q_near, q_new) then
        return q_new
    else
        return null
    end if
near_neighbors(q_new, T, gamma):
create an empty set near_nodes
    for each node q in T do
        if distance(q, q_new) < gamma then
            add q to near_nodes
end if end for
    return near_nodes
cost(q1, q2):
    return the cost of the path from q1 to q2
cost_to_come(q):
    return the lowest cost path from the start configuration to q
parent(q):
    return the parent node of q in T
interpolate(q1, q2, delta_q):
    return a new configuration that is delta_q away from q1 in the
direction of q2
collision_free(q1, q2):
    return true if the path from q1 to q2 is collision-free, false
otherwise
```
Gamma controls the size of the neighborhood around each node in the search tree that is used for re-wiring.

RRT* has real-time applications in robotics, autonomous vehicles, and motion planning. For example, in robotics, RRT* can be used for planning collision-free paths for robot arms or drones in complex and dynamic environments. In autonomous vehicles, RRT* can assist in finding optimal paths for navigation while avoiding obstacles in real time. RRT* can also be used for planning trajectories for robotic surgeries, animation, and video games where real-time decision-making is critical.


https://user-images.githubusercontent.com/77829961/235368599-dffb84db-4fdf-48ba-adae-10055010814b.mp4

## Informed RRTStar:
Informed RRT* is an improvement over the classic RRT* algorithm, which is a popular path-planning algorithm used in robotics and autonomous systems. Informed RRT* improves the efficiency of RRT* by biasing the exploration towards the goal region, leading to faster convergence to a near-optimal solution. This is achieved by using heuristics or information about the goal region to guide the sampling and exploration process, resulting in fewer unnecessary samples and a more efficient search toward the goal.

To perform Informed RRT*, the following steps are typically followed:
● Define a heuristic function that estimates the cost from a given point to the goal region. This can be based on Euclidean distance, occupancy grid information, or any other domain-specific information.
● Initialize the tree with the starting point as the root.
● Sample a random point in the search space, biased towards the goal region
based on the heuristic function.
● Extend the tree towards the sampled point, considering collision checking to
avoid obstacles.
● Update the cost of the path from the root to the new node and rewire the tree to
improve the path's quality.
● Repeat the sampling, extension, and rewiring steps until the goal region is
reached or a desired solution quality is achieved.

<img width="477" alt="Screenshot 2023-04-30 at 11 29 40 PM" src="https://user-images.githubusercontent.com/77829961/235368847-ab711e16-bba4-4b2c-bb1a-b4af40b62a07.png">

### Pseudocode:
```
InformedRRT*(q_start, q_goal, K, delta_q, gamma, heuristic):
    create a search tree T
    add q_start to T as the root node
    set cost_to_come(q_start) = 0
    set heuristic(q_start) = estimate_cost(q_start, q_goal)
    for k = 1 to K do
q_new)
q_rand = sample a random configuration in the search space
q_near = nearest_neighbor(q_rand, T)
q_new = extend(q_near, q_rand, delta_q)
if q_new is not null and collision_free(q_near, q_new) then
    T.add_node(q_new)
    set cost_to_come(q_new) = cost_to_come(q_near) + cost(q_near,
    set heuristic(q_new) = estimate_cost(q_new, q_goal)
    near_nodes = near_neighbors(q_new, T, gamma)
    for q_near in near_nodes do
                if collision_free(q_new, q_near) and cost_to_come(q_new) +
cost(q_new, q_near) < cost_to_come(q_near) then
cost(q_new, q_near)
end if end for
    return T
end function
T.remove_edge(parent(q_near), q_near)
T.add_edge(q_new, q_near)
set cost_to_come(q_near) = cost_to_come(q_new) +
update_heuristic(q_near, q_goal, T)
estimate_cost(q1, q2):
    return an estimate of the cost of the path from q1 to q2
update_heuristic(q, q_goal, T):
    set heuristic(q) = min(heuristic(q), estimate_cost(q, q_goal))
    for each node q_child in children(q) do
        update_heuristic(q_child, q_goal, T)
```        
Informed RRT* has real-time applications in various fields, such as robotics, autonomous vehicles, and video games. For example:
● Robotic path planning: Informed RRT* can be used in real-time to plan efficient and collision-free paths for autonomous robots, allowing them to navigate complex environments and avoid obstacles while converging towards a goal.
● Autonomous vehicles: Informed RRT* can be used in real-time for trajectory planning and obstacle avoidance in self-driving cars, enabling them to plan safe and efficient routes on the fly while considering real-time sensor data.
● Video games: Informed RRT* can be used in real-time for generating realistic and dynamic motion planning for virtual characters in video games, providing interactive and responsive navigation behavior for game agents.
<img width="477" alt="Screenshot 2023-04-30 at 11 31 11 PM" src="https://user-images.githubusercontent.com/77829961/235368917-4d5f7568-8771-4573-9d92-2a721470a3e3.png">
