# RRT, RRTStar and Informed RRTStar
 
### Problem:
## Artificial Intelligence Mini Project

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
