from collections import deque
import heapq
import time
import obstacles
from typing import Optional, Set, Deque, Tuple, Dict

def find_path(start: Tuple, goal: Tuple) -> Dict:
    """
    Searches for the optimal path from a starting point to a goal point.

    Args:
        start (tuple): x (float), y (float) of the starting point
        goal (tuple): x (float), y (float) of the goal point

    Returns:
        Dict consisting of:
        - path (dict), 
        - visited_nodes (list), 
        - algorithm-runtime (float)
        - success-status (bool)
        - message (str)
    """
    start_time = time.time()
    
    # Initialize data structures
    open_list = [] # list used to make heapq
    closed_set = set() 
    parent_map = {} # keeps track of nodes & matching parent
    cost_map = {} # keeps track of nodes & their costs
    
    # Setup start node
    heapq.heappush(open_list, (0, start))
    cost_map[start] = 0
    parent_map[start] = None
    
    # Define possible movements (8-directional: up, down, forward, back, and four diagonals)
    # with their corresponding costs
    movements = (
        (0, 1, 1), (0, -1, 1), (-1, 0, 1), (1, 0, 1),
        (1, 1, 1.4), (-1, 1, 1.4), (-1, -1, 1.4), (1, -1, 1.4)
    )
    
    while open_list:
        current_cost, current = heapq.heappop(open_list)
        
        if current == goal:
            path = _reconstruct_path(parent_map, current)
            return {
                'path': path,
                'visited_nodes': list(parent_map.keys()),
                'runtime': time.time() - start_time,
                'success': True,
                'message': "Path found!"
            }
        
        closed_set.add(current) # add popped node to closed set
        # make new nodes by moving in 8-directions
        for delta_x, delta_y, move_cost in movements:
            new_x = current[0] + delta_x
            new_y = current[1] + delta_y
            neighbor = (new_x, new_y)
            # Skip nodes that are in the obstacle space
            if obstacles.is_collision(new_x, new_y):
                continue
            if neighbor not in closed_set:
                new_cost = current_cost + move_cost                 
                # Choose node with lowest-cost, if the same node is found with a different path.
                if neighbor not in cost_map or new_cost < cost_map[neighbor]:
                    cost_map[neighbor] = new_cost #update cost_map
                    parent_map[neighbor] = current #update parent_map
                    heapq.heappush(open_list, (new_cost, neighbor)) # add node to heapq
    
    return {
        'path': None,
        'visited_nodes': list(parent_map.keys()),
        'runtime': time.time() - start_time,
        'success': False,
        'message': "No Path found!"
    }

def _reconstruct_path(parent_map: dict, current: Tuple) -> Deque:
    """ 
    backtracks to create a path from the current node back to the initial node. 
    Returns the final path.
    
    Args:
        current (tuple): x,y coordinate of final point
        parent_node_map (dict): nodes and their parent nodes

    Returns:
        path (deque): deque containing x,y of points on the final path.
    """
    path = deque([current])
    while current in parent_map:
        current = parent_map[current]
        if current is None:
            break
        path.appendleft(current)
    return path
