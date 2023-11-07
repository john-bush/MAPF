# PRIMAL2 Implementation Notes

## Model Action Space in PRIMAL2 ##
### Action space: (Tuple) ###
    agent_id: positive integer [0, n-1]

    action: {0:STILL, 1:MOVE_NORTH, 2:MOVE_EAST, 3:MOVE_SOUTH, 4:MOVE_WEST,
             5:NE, 6:SE, 7:SW, 8:NW, 5,6,7,8 not used in non-diagonal world}
    Reward: ACTION_COST for each action, GOAL_REWARD when robot arrives at target

### Reward ###
    ACTION_COST for each action, GOAL_REWARD when robot arrives at target

## Model Action Space for Robot Runners Competition ##
### Action Space ###
    agent_id: positive integer [0, num_of_robots-1]

    action: {FW:forward, CR:Clockwise rotate, CCR:Counter clockwise rotate, W:Wait, NA:Unknown actions}

    Reward: 

## PRIMAL2 Observation Space ##
Each observation space map is an 11x11 map centered around an agent.

Agents are also provided with the unit vector {u_x, u_y} and the euclidean distance to the goal at all times.


### Obstacle Map ###
A binary map (0's and 1's indicating where any obstacles are around an agent. This is just a local view of the world map)

### Goal Map ###
A binary map showing the location of the goal (if in local FOV)

### Other Agent Map ###
A binary map showing the locations of other agents. (1 if there is an agent on a space in the local FOV, 0 otherwise)

### Other Goals Map ### 
A binary map showing the locations of other agent goals.

### Path Length Maps ### 
Each agent has a path length map that contains the (normalized) shortest-path distance to its goal from each non-obstacle cell within its FOV.

We can speed up the computation of this map during runtime using the precomputed A* matrix

### Future Step Maps (some number >= 1) ###
These maps show where other agents within the FOV of the agent could be n time steps in the future according to their A* search paths to their goals.

### Delta Map ###
- Delta X Map
    - 

- Delta Y Map

### Blocking Map ###


# Modification of Observation Space for Robot Runners #
## general modifications for the action space ##
In general, we will need to use a modification of the A* algorithm that considers facing direction of the agent when calculating neighbors.

## Precomputation of the A* map ##
Since we know the maps ahead of time, we can precompute the A* path that is used in the the Path Length and the A* Maps. In the case of the path length map, where the observation size is 11x11, we can reduce computation from 121 A* searches that run in O(n^2) time to 121 reads from a matrix

**Precomputed matrix representation:**

```
Stored as a serialized numpy matrix.

For a map with n rows and m columns:

The size of the A* matrix is [n*m x n*m * 4]

For an element M[i, j, k]:

i -> linearized starting index calculated as (row * m) + column
j -> linearized goal index
k -> facing direction at start. k in {0, 3}

Pseudocode:

For i_start in range (n*m):
    For i_goal in range (n*m):
        For dir in dirs:
            Store: M[i_start, i_goal, dir] = Run A*(i_start, i_goal, dir)

            -> {compute flow}

-> {normalize flow}

```

**Precomputed Flow Representation**

Another thing that we can precompute in this competition is a representation of the choke points in the map (if any). We can do this by adding another matrix and counting the number of paths that cross given locations in the map.
```
Flow: matrix of size n x m or vector of size 1 x n*m

(During the computation loop described above):

compute flow:
    for node in A* path:
        index <- linearize node
        flow[index]++

normalize flow:
    after A* matrix is calculated, normalize the flow matrix by dividing each element by the max flow node.

This could be even more informative if we used it to represent some sort of probabalistic conflict measure... for instance if we could track direction of path and then generate a higher weight in the flow matrix for greater numbers of paths in opposite directions. 

```

higher values in the flow matrix indicates that a node is a high traffic node. If we can encode this flow matrix with information that indicates how likely a collision or deadlock is to occur at this node, we can then try to reduce such deadlocks in the policy by penalizing turning or waiting in these areas.

## Desirable Behaviors ##
* Teaching robots to not wait or turn on high traffic nodes
* Have a more variable cost function that allows for different costs for waiting, turning, and moving forward.
    * This is because in PRIMAL2, their action space allows for arbitrary direction changes.





