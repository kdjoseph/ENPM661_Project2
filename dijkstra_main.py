import heapq as hq
import sys
import math
import time
from collections import deque
import pygame
from pygame.locals import *
pygame.init()

def is_in_obstacle(x, y, window_width, window_height, bloated_obstacles):
    """
    Checks if a point is on any of the obstacles, then returns True if it is, and False if it is not
    
    Args:
        x (float): x coordinate of the point.
        y (float): y coordinate of a point.
        window_width (int): width of the obstacle layout.
        window_height (int): height of the obstacle layout.
        bloated_obstacles (dict): dictionary containing information on the different bloated obstacle shapes
    """
    # Hexagon Obstacle
    if (bloated_obstacles['hexagon'][1][0][0]*x+bloated_obstacles['hexagon'][1][0][1]<=y<=\
        bloated_obstacles['hexagon'][1][2][0]*x+bloated_obstacles['hexagon'][1][2][1]) and\
        (bloated_obstacles['hexagon'][1][1][0]*x+bloated_obstacles['hexagon'][1][1][1]<=y<=\
            bloated_obstacles['hexagon'][1][3][0]*x+bloated_obstacles['hexagon'][1][3][1]) and\
        bloated_obstacles['hexagon'][1][4]<=x <= bloated_obstacles['hexagon'][1][5]:
        return True
    # First 2 rectangular obstacles
    if (bloated_obstacles['first_left_rectangle'][2][0] <= x <= \
        bloated_obstacles['first_left_rectangle'][2][1]\
        and y <= bloated_obstacles['first_left_rectangle'][2][2]) or \
            (bloated_obstacles['second_left_rectangle'][2][0] <= x <= \
             bloated_obstacles['second_left_rectangle'][2][1]\
        and y >= bloated_obstacles['second_left_rectangle'][2][2]):
        return True
    # 4th Obstacle, on the right of display
    if (bloated_obstacles['right_top_horizontal_rectangle'][0][0]<= x <= \
        bloated_obstacles['right_top_horizontal_rectangle'][0][0]+bloated_obstacles['right_top_horizontal_rectangle'][1][0]\
        and bloated_obstacles['right_top_horizontal_rectangle'][0][1]<=y<=\
        bloated_obstacles['right_top_horizontal_rectangle'][0][1]+bloated_obstacles['right_top_horizontal_rectangle'][1][1]) or\
        (bloated_obstacles['right_vertical_rectangle'][0][0]<= x <= \
        bloated_obstacles['right_vertical_rectangle'][0][0]+bloated_obstacles['right_vertical_rectangle'][1][0]\
        and bloated_obstacles['right_vertical_rectangle'][0][1]<=y<=\
        bloated_obstacles['right_vertical_rectangle'][0][1]+bloated_obstacles['right_vertical_rectangle'][1][1]) or \
        (bloated_obstacles['right_bottom_horizontal_rectangle'][0][0]<= x <= \
        bloated_obstacles['right_bottom_horizontal_rectangle'][0][0]+bloated_obstacles['right_bottom_horizontal_rectangle'][1][0]\
        and bloated_obstacles['right_bottom_horizontal_rectangle'][0][1]<=y<=\
        bloated_obstacles['right_bottom_horizontal_rectangle'][0][1]+bloated_obstacles['right_bottom_horizontal_rectangle'][1][1]):
        return True
    # Border region outside all obstacles
    if (x <=bloated_obstacles['clearance'] or \
        x >= (window_width-bloated_obstacles['clearance'])) or \
            (y <=bloated_obstacles['clearance'] or \
             y >= (window_height-bloated_obstacles['clearance'])):
        return True
    else:               # not in obstacle space
        return False

def retrace_steps(vertex, parent_node_map):
    """ 
    backtracks to create a path from the current node back to the initial node. 
    Returns the final path.
    
    Args:
        vertex (tuple): x,y coordinate of final point
        parent_node_map (dict): nodes and their parent nodes

    Returns:
        path (deque): deque containing x,y of points on the final path.
    """

    path = deque()     # deque used for backtracking
    # Keep backtracking until start node reached
    while vertex is not None:
        path.appendleft(vertex)  # Add the current node to the path
        vertex = parent_node_map.get(vertex)  # Move to the parent node to now search for its parent
    if len(path)>1:
        print('Path found!\n')
        return path
    else:
        print('Could not find a path \n')

def get_user_inputs(window_width, window_height, bloated_obstacles):
    """
    Asks the user to enter the start and goal points, and returns them
    
    Args:
        window_width (int): width of the obstacle layout
        window_height (int): height of the obstacle layout
        bloated_obstacles (dict): dictionary containing information on the shape & size
        of each obstacles

    Returns:
        start, goal (tuple): combined tuple of the x,y coordinates of the start & goal points

    Raises:
        ValueError: if the user does not enter a number
    """

    while True:
        try:
            # worst case start wrt to coord at bottom-left corner:
            # start (6,6), goal (1194, 162) or goal (1194, 338)
            start_x, startb_y = float(input('Enter starting point x-cordinate : ')),\
                float(input('Enter starting point y-cordinate: '))
            # convert from coordinate w.r.t the lower-left corner of display to
            # upper left-corner coordinate (pygame coord)
            start_y = 500 - startb_y
            if is_in_obstacle(start_x, start_y, window_width, window_height, bloated_obstacles):
                print('The chosen start point is in the obstacle space or',
                       'too close to the border or', 
                      'out of the display dimensions, choose another one.')
            else:
                break
        except ValueError:
            print('you did not enter a number, please enter only numbers')

    while True:
        try:
            goal_x, goalb_y = float(input('Enter goal point x- cordinate: ')), \
                float(input('Enter goal point y-cordinate: '))
            # convert from coordinate wrt to lower-left corner of display to
            # upper left-corner coordinate (pygame coord)
            goal_y = 500 - goalb_y  
            if is_in_obstacle(goal_x, goal_y, window_width, window_height, bloated_obstacles):
                print('The chosen goal point is in the obstacle space or',
                       ' too close to the border or',
                       ' out of the display dimensions, choose another one.')
                continue
            if (start_x,start_y) == (goal_x, goal_y):
                print('you chose the same starting and goal points,',
                       'choose different starting and goal points.')
                continue
            return (start_x, start_y), (goal_x, goal_y)
        except ValueError:
            print('you did not enter a number, please enter only numbers')

def dijkstra_optimal_path(start, goal, window_width, window_height, bloated_obstacles):
    """
    Searches for the optimal path from a starting point to a goal point.

    Args:
        start (tuple): x (float), y (float) of the starting point
        goal (tuple): x (float), y (float) of the goal point

    Returns:
        tuple: parent_node_map (dict), optimal-path (deque), algorithm-runtime (float)
    """
    open_list = []  # to be used for heapq for nodes in the open-list
    closed_list = set() # set to store the nodes without c2c for fast comparison with new nodes
    parent_node_map = {} # dict (key= visited-node, value=parent)
    lowest_cost2come_map ={} # dict (key=node, val=c2c) to keep track of the nodes and their cost

    (start_x, start_y), (goal_x, goal_y) = start, goal
    dijkstra_strt_time = time.time() # start time of algorithm
    cost2come_start = 0.0 # starting point cost-to-come

    #Push first node to heap queue which also simultaneously heapifies the queue
    hq.heappush(open_list, (cost2come_start,(start_x, start_y)))
    # Update lowest cost and parent node maps with starting point info
    lowest_cost2come_map[(start_x, start_y)] = cost2come_start
    parent_node_map[(start_x, start_y)] = None

    # Dijkstra while loop to generate new nodes
    while len(open_list) > 0:
        current_cost2come, (current_x, current_y) = hq.heappop(open_list)
        # Filter out nodes that had higher cost in the open-list compared to in the lowest cost map
        if lowest_cost2come_map.get(current_x, current_y) is not None and \
            current_cost2come > lowest_cost2come_map.get((current_x, current_y)):
            continue

        closed_list.add((current_x, current_y))  # add popped-node into closed set
        # Check if we've reached the goal,if yes, backtrack to find path
        if (current_x, current_y) == (goal_x, goal_y):
            print(f"\nGoal point {(current_x, window_height-current_y)} reached!")
            # Backtracking
            current_node = (current_x, current_y)
            final_path = retrace_steps(current_node, parent_node_map)
            dijkstra_end_time = time.time() # end time of algorithm
            return parent_node_map, final_path, dijkstra_end_time-dijkstra_strt_time

        # Explore neighbors with 8 possible actions (delta_x, delat_y, cost-to-come)
        for dx, dy, cost2come in ((0, 1, 1), (0, -1, 1), (-1, 0, 1), (1, 0, 1), \
                               (1, 1, 1.4), (-1, 1, 1.4), (-1, -1, 1.4), (1, -1, 1.4)):
            new_x, new_y = current_x + dx, current_y + dy
            # Ignore node if it's in the obstacle space.
            if is_in_obstacle(new_x,new_y, window_width, window_height, bloated_obstacles):
                continue

            if (new_x, new_y) not in closed_list:
                new_cost2come = current_cost2come + cost2come
                # Only update the heapq & other dictionaries if new node not in lowest-cost map
                # or if now it's the lowest-cost node
                if (new_x, new_y) not in lowest_cost2come_map or \
                    new_cost2come < lowest_cost2come_map[(new_x, new_y)]:
                    parent_node_map[(new_x, new_y)] = (current_x, current_y)
                    lowest_cost2come_map[(new_x,new_y)] = new_cost2come
                    hq.heappush(open_list, (new_cost2come, (new_x,new_y)))
        # Stop if the algorithm can't find a solution
        if len(open_list)== 0:
            print('No solution could be found')
            break

def draw_environment(WINDOW, background_color, obstacle_color, bloated_obstacle_color,\
                     obstacles, bloated_obstacles):
    """ 
    Draws the entire obstacle layout, including the obstacles & the background
    
    Args:
        window (constant): pygame surface
        background_color (tuple): RGB(int,int,int) values to define the background color
        obstacle_color (tuple): RGB(int,int,int) values to define the color of the obstacles
        bloated_obstacle_color (tuple): RGB(int,int,int) values to define the color of the bloated obstacles
        obstacles (dict): dictionary containing information on the different obstacle-shapes
        bloated_obstacles (dict): dictionary containing information on the different bloated obstacle-shapes
    """

    WINDOW.fill(background_color)
    # Highligh 5 unit bloat around obstacles
    pygame.draw.polygon(WINDOW, bloated_obstacle_color, (bloated_obstacles['hexagon'][0][0],
                                                         bloated_obstacles['hexagon'][0][1],
                                                         bloated_obstacles['hexagon'][0][2],
                                                         bloated_obstacles['hexagon'][0][3],
                                                         bloated_obstacles['hexagon'][0][4],
                                                         bloated_obstacles['hexagon'][0][5]))
    
    pygame.draw.rect(WINDOW, bloated_obstacle_color, (bloated_obstacles['first_left_rectangle'][0][0],
                                                      bloated_obstacles['first_left_rectangle'][0][1],
                                                      bloated_obstacles['first_left_rectangle'][1][0],
                                                      bloated_obstacles['first_left_rectangle'][1][1]))

    pygame.draw.rect(WINDOW, bloated_obstacle_color, (bloated_obstacles['second_left_rectangle'][0][0],
                                                      bloated_obstacles['second_left_rectangle'][0][1],
                                                      bloated_obstacles['second_left_rectangle'][1][0],
                                                      bloated_obstacles['second_left_rectangle'][1][1]))
    

    pygame.draw.rect(WINDOW, bloated_obstacle_color, (bloated_obstacles['right_top_horizontal_rectangle'][0][0],
                                                      bloated_obstacles['right_top_horizontal_rectangle'][0][1],
                                                      bloated_obstacles['right_top_horizontal_rectangle'][1][0],
                                                      bloated_obstacles['right_top_horizontal_rectangle'][1][1]))
    
    pygame.draw.rect(WINDOW, bloated_obstacle_color, (bloated_obstacles['right_vertical_rectangle'][0][0],
                                                      bloated_obstacles['right_vertical_rectangle'][0][1],
                                                      bloated_obstacles['right_vertical_rectangle'][1][0],
                                                      bloated_obstacles['right_vertical_rectangle'][1][1]))
    
    pygame.draw.rect(WINDOW, bloated_obstacle_color, (bloated_obstacles['right_bottom_horizontal_rectangle'][0][0],
                                                      bloated_obstacles['right_bottom_horizontal_rectangle'][0][1],
                                                      bloated_obstacles['right_bottom_horizontal_rectangle'][1][0],
                                                      bloated_obstacles['right_bottom_horizontal_rectangle'][1][1]))
    # Drawing the 5 unit bloat for walls
    pygame.draw.rect(WINDOW, bloated_obstacle_color, (bloated_obstacles['top_left_border'][0][0],
                                                      bloated_obstacles['top_left_border'][0][1],
                                                      bloated_obstacles['top_left_border'][1][0],
                                                      bloated_obstacles['top_left_border'][1][1]))
    
    pygame.draw.rect(WINDOW, bloated_obstacle_color, (bloated_obstacles['top_border'][0][0],
                                                      bloated_obstacles['top_border'][0][1],
                                                      bloated_obstacles['top_border'][1][0],
                                                      bloated_obstacles['top_border'][1][1]))
    
    pygame.draw.rect(WINDOW, bloated_obstacle_color, (bloated_obstacles['left_vertical_border'][0][0],
                                                      bloated_obstacles['left_vertical_border'][0][1],
                                                      bloated_obstacles['left_vertical_border'][1][0],
                                                      bloated_obstacles['left_vertical_border'][1][1]))
    
    pygame.draw.rect(WINDOW, bloated_obstacle_color, (bloated_obstacles['bottom_left_horizontal_border'][0][0],
                                                      bloated_obstacles['bottom_left_horizontal_border'][0][1],
                                                      bloated_obstacles['bottom_left_horizontal_border'][1][0],
                                                      bloated_obstacles['bottom_left_horizontal_border'][1][1]))
    
    pygame.draw.rect(WINDOW, bloated_obstacle_color, (bloated_obstacles['bottom_horizontal_border'][0][0],
                                                      bloated_obstacles['bottom_horizontal_border'][0][1],
                                                      bloated_obstacles['bottom_horizontal_border'][1][0],
                                                      bloated_obstacles['bottom_horizontal_border'][1][1]))
    
    pygame.draw.rect(WINDOW, bloated_obstacle_color, (bloated_obstacles['right_vertical_border'][0][0],
                                                      bloated_obstacles['right_vertical_border'][0][1],
                                                      bloated_obstacles['right_vertical_border'][1][0],
                                                      bloated_obstacles['right_vertical_border'][1][1]))
    # Actual obstacles
    pygame.draw.polygon(WINDOW, obstacle_color, (obstacles['hexagon'][0],
                                                 obstacles['hexagon'][1],
                                                 obstacles['hexagon'][2],
                                                 obstacles['hexagon'][3],
                                                 obstacles['hexagon'][4],
                                                 obstacles['hexagon'][5]))

    pygame.draw.rect(WINDOW, obstacle_color, (obstacles['first_left_rectangle'][0][0],
                                                      obstacles['first_left_rectangle'][0][1],
                                                      obstacles['first_left_rectangle'][1][0],
                                                      obstacles['first_left_rectangle'][1][1]))

    pygame.draw.rect(WINDOW, obstacle_color, (obstacles['second_left_rectangle'][0][0],
                                                      obstacles['second_left_rectangle'][0][1],
                                                      obstacles['second_left_rectangle'][1][0],
                                                      obstacles['second_left_rectangle'][1][1]))

    pygame.draw.rect(WINDOW, obstacle_color, (obstacles['right_top_horizontal_rectangle'][0][0],
                                                      obstacles['right_top_horizontal_rectangle'][0][1],
                                                      obstacles['right_top_horizontal_rectangle'][1][0],
                                                      obstacles['right_top_horizontal_rectangle'][1][1]))
    
    pygame.draw.rect(WINDOW, obstacle_color, (obstacles['right_vertical_rectangle'][0][0],
                                                      obstacles['right_vertical_rectangle'][0][1],
                                                      obstacles['right_vertical_rectangle'][1][0],
                                                      obstacles['right_vertical_rectangle'][1][1]))
    
    pygame.draw.rect(WINDOW, obstacle_color, (obstacles['right_bottom_horizontal_rectangle'][0][0],
                                                      obstacles['right_bottom_horizontal_rectangle'][0][1],
                                                      obstacles['right_bottom_horizontal_rectangle'][1][0],
                                                      obstacles['right_bottom_horizontal_rectangle'][1][1]))
    pygame.display.update()

def animate_explored_nodes(WINDOW, node_color, parent_node_map, nodes_per_frame=10):
    """ 
    Animates the explored nodes. Takes the display-window.
    
    Args:
        window (contant): pygame surface
        node_color (tuple): RGB (int, int, int) tuple to set the color of the nodes
        parent_node_map (dict): dictionairy containing the explored nodes & their corresponding parent-nodes
        nodes_per_frame (int): int to set how many nodes to display at once during each animation frame.
    """
    node_list = list(parent_node_map.keys())
    for i in range(0, len(node_list), nodes_per_frame):
        for node in node_list[i:i+nodes_per_frame]:
            pygame.draw.circle(WINDOW, node_color, (int(node[0]), int(node[1])), 1)
        pygame.display.update()
        pygame.time.delay(1)  # delay to adjust animation speed

def animate_optimal_path(WINDOW, path_color, path):
    """Function to animate moving from start to goal using the optimal path"""
    for node in path:
        pygame.draw.circle(WINDOW, path_color, (int(node[0]), int(node[1])), 2)
        pygame.display.update()
        pygame.time.delay(4)  # delay to adjust animation speed

#### MAIN FUNCTION (start algorithm, then animates) ###############
def main():
    """ 
    Uses the Dijkstra Algorithm to find the optimal path between user chosen start and goal points, 
    then creates & animation of the search process & of the optimal path
    """
    # Initialize Pygame window
    WINDOW_WIDTH = 1200
    WINDOW_HEIGHT = 500
    # WINDOW_SIZE = (WINDOW_WIDTH, WINDOW_HEIGHT)
    CLEARANCE = 5

    BACKROUND_COLOR = (0, 40, 255) # blue
    OBSTACLE_COLOR = (255, 30, 70)   # RED
    BLOATED_OBSTACLE_COLOR = (255, 255, 0) # yellow
    NODES_COLOR = (0, 100, 0) # green
    PATH_COLOR = (255, 255, 255) # white

    # Game Setup
    FPS = 60
    fpsClock = pygame.time.Clock()

    ##### FIRST RECTANGULAR OBSTACLE ON THE LEFT OF DISPLAY #######
    FIRST_LEFT_RECTANGLE_X = 100
    FIRST_LEFT_RECTANGLE_Y = 0
    FIRST_LEFT_RECTANGLE_HEIGHT = 400
    FIRST_LEFT_RECTANGLE_WIDTH = 75

    OBSTACLES = {'first_left_rectangle': ((FIRST_LEFT_RECTANGLE_X, FIRST_LEFT_RECTANGLE_Y),
                                          (FIRST_LEFT_RECTANGLE_WIDTH, FIRST_LEFT_RECTANGLE_HEIGHT))}

    BLOATED_FIRST_LEFT_RECTANGLE_X = FIRST_LEFT_RECTANGLE_X-CLEARANCE
    BLOATED_FIRST_LEFT_RECTANGLE_Y = FIRST_LEFT_RECTANGLE_Y
    BLOATED_FIRST_LEFT_RECTANGLE_HEIGHT= FIRST_LEFT_RECTANGLE_HEIGHT + CLEARANCE
    BLOATED_FIRST_LEFT_RECTANGLE_WIDTH = FIRST_LEFT_RECTANGLE_WIDTH + 2*CLEARANCE

    BLOATED_FIRST_RECTANGLE_LEFT_VERTICAL_EDGE = BLOATED_FIRST_LEFT_RECTANGLE_X
    BLOATED_FIRST_RECTANGLE_RIGHT_VERTICAL_EDGE = BLOATED_FIRST_LEFT_RECTANGLE_X + BLOATED_FIRST_LEFT_RECTANGLE_WIDTH
    BLOATED_FIRST_RECTANGLE_BOTTOM_HORIZONTAL_EDGE = BLOATED_FIRST_LEFT_RECTANGLE_HEIGHT
    BLOATED_OBSTACLES = {'first_left_rectangle': ((BLOATED_FIRST_LEFT_RECTANGLE_X, BLOATED_FIRST_LEFT_RECTANGLE_Y),
                                                  (BLOATED_FIRST_LEFT_RECTANGLE_WIDTH, BLOATED_FIRST_LEFT_RECTANGLE_HEIGHT),
                                                  (BLOATED_FIRST_RECTANGLE_LEFT_VERTICAL_EDGE,
                                          BLOATED_FIRST_RECTANGLE_RIGHT_VERTICAL_EDGE,
                                          BLOATED_FIRST_RECTANGLE_BOTTOM_HORIZONTAL_EDGE))}
    
    ############### SECOND RECTANGLE ON THE LEFT OF THE DISPLAY ##########################################
    SECOND_LEFT_RECTANGLE_HEIGHT = FIRST_LEFT_RECTANGLE_HEIGHT
    SECOND_LEFT_RECTANGLE_WIDTH = FIRST_LEFT_RECTANGLE_WIDTH
    SECOND_LEFT_RECTANGLE_X = FIRST_LEFT_RECTANGLE_X + FIRST_LEFT_RECTANGLE_WIDTH + 100
    SECOND_LEFT_RECTANGLE_Y = WINDOW_HEIGHT - FIRST_LEFT_RECTANGLE_HEIGHT
    OBSTACLES['second_left_rectangle'] = ((SECOND_LEFT_RECTANGLE_X, SECOND_LEFT_RECTANGLE_Y),
                                          (SECOND_LEFT_RECTANGLE_WIDTH, SECOND_LEFT_RECTANGLE_HEIGHT))
    BLOATED_SECOND_LEFT_RECTANGLE_X = SECOND_LEFT_RECTANGLE_X-CLEARANCE
    BLOATED_SECOND_LEFT_RECTANGLE_Y = SECOND_LEFT_RECTANGLE_Y-CLEARANCE
    BLOATED_SECOND_LEFT_RECTANGLE_HEIGHT= SECOND_LEFT_RECTANGLE_HEIGHT + CLEARANCE
    BLOATED_SECOND_LEFT_RECTANGLE_WIDTH = SECOND_LEFT_RECTANGLE_WIDTH + 2*CLEARANCE

    BLOATED_SECOND_RECTANGLE_LEFT_VERTICAL_EDGE = BLOATED_SECOND_LEFT_RECTANGLE_X
    BLOATED_SECOND_RECTANGLE_RIGHT_VERTICAL_EDGE = BLOATED_SECOND_LEFT_RECTANGLE_X + BLOATED_SECOND_LEFT_RECTANGLE_WIDTH
    BLOATED_SECOND_RECTANGLE_BOTTOM_HORIZONTAL_EDGE = BLOATED_SECOND_LEFT_RECTANGLE_Y
    BLOATED_OBSTACLES['second_left_rectangle'] = ((BLOATED_SECOND_LEFT_RECTANGLE_X, BLOATED_SECOND_LEFT_RECTANGLE_Y),
                                                  (BLOATED_SECOND_LEFT_RECTANGLE_WIDTH, BLOATED_SECOND_LEFT_RECTANGLE_HEIGHT),
                                                  (BLOATED_SECOND_RECTANGLE_LEFT_VERTICAL_EDGE,
                                          BLOATED_SECOND_RECTANGLE_RIGHT_VERTICAL_EDGE,
                                          BLOATED_SECOND_RECTANGLE_BOTTOM_HORIZONTAL_EDGE))
    ###### HEXAGON BLOATED_OBSTACLES ###### 

    HEXAGON_CENTER = (SECOND_LEFT_RECTANGLE_X+SECOND_LEFT_RECTANGLE_WIDTH+300, WINDOW_HEIGHT/2)
    HEXAGON_SIDE_LENGTH = 150
    BLOATED_HEXAGON_SIDE_LENGTH = HEXAGON_SIDE_LENGTH + 2 * CLEARANCE
    # Vertex A starts from top left vertical side, then go clockwise for the other vertices
    VERTEX_A = (HEXAGON_CENTER[0] - 0.5*HEXAGON_SIDE_LENGTH*math.sqrt(3),
                HEXAGON_CENTER[1]- HEXAGON_SIDE_LENGTH/2)
    VERTEX_B = (HEXAGON_CENTER[0], HEXAGON_CENTER[1]-HEXAGON_SIDE_LENGTH)
    VERTEX_C = (HEXAGON_CENTER[0]+0.5*HEXAGON_SIDE_LENGTH*math.sqrt(3), VERTEX_A[1])
    VERTEX_D = (VERTEX_C[0], VERTEX_C[1] +HEXAGON_SIDE_LENGTH)
    VERTEX_E = (HEXAGON_CENTER[0], HEXAGON_CENTER[1]+HEXAGON_SIDE_LENGTH)
    VERTEX_F = (VERTEX_A[0], VERTEX_A[1]+HEXAGON_SIDE_LENGTH)
    OBSTACLES['hexagon'] = (VERTEX_A, VERTEX_B, VERTEX_C,
                            VERTEX_D, VERTEX_E, VERTEX_F)

    BLOATED_VERTEX_A = (HEXAGON_CENTER[0] - 0.5*BLOATED_HEXAGON_SIDE_LENGTH*math.sqrt(3), 
                        HEXAGON_CENTER[1]- BLOATED_HEXAGON_SIDE_LENGTH/2)
    BLOATED_VERTEX_B = (HEXAGON_CENTER[0], HEXAGON_CENTER[1]-BLOATED_HEXAGON_SIDE_LENGTH)
    BLOATED_VERTEX_C = (HEXAGON_CENTER[0]+0.5*BLOATED_HEXAGON_SIDE_LENGTH*math.sqrt(3), BLOATED_VERTEX_A[1])
    BLOATED_VERTEX_D = (BLOATED_VERTEX_C[0], BLOATED_VERTEX_C[1] +BLOATED_HEXAGON_SIDE_LENGTH)
    BLOATED_VERTEX_E = (HEXAGON_CENTER[0], HEXAGON_CENTER[1]+BLOATED_HEXAGON_SIDE_LENGTH)
    BLOATED_VERTEX_F = (BLOATED_VERTEX_A[0], BLOATED_VERTEX_A[1]+BLOATED_HEXAGON_SIDE_LENGTH)

    # y = mx+b, with m=slope and b= y_intercept. m = (y2-y1)/(x2-x1)
    SLOPE_AB = (BLOATED_VERTEX_B[1]-BLOATED_VERTEX_A[1])/(BLOATED_VERTEX_B[0]-BLOATED_VERTEX_A[0])
    SLOPE_BC = (BLOATED_VERTEX_C[1]-BLOATED_VERTEX_B[1])/(BLOATED_VERTEX_C[0]-BLOATED_VERTEX_B[0])
    SLOPE_ED = SLOPE_AB # because AB is parallel to ED
    SLOPE_FE = SLOPE_BC # because BC IS parallel to FD
    # y= mx+b-> b = y-mx
    Y_INTERCEPT_AB = BLOATED_VERTEX_A[1] - SLOPE_AB * BLOATED_VERTEX_A[0]
    Y_INTERCEPT_BC = BLOATED_VERTEX_B[1] - SLOPE_BC * BLOATED_VERTEX_B[0]
    Y_INTERCEPT_ED = BLOATED_VERTEX_E[1] - SLOPE_ED * BLOATED_VERTEX_E[0]
    Y_INTERCEPT_FE = BLOATED_VERTEX_F[1] - SLOPE_FE * BLOATED_VERTEX_F[0]

    EDGE_AB = (SLOPE_AB, Y_INTERCEPT_AB)
    EDGE_BC = (SLOPE_BC, Y_INTERCEPT_BC)
    EDGE_ED = (SLOPE_ED, Y_INTERCEPT_ED)
    EDGE_FE = (SLOPE_FE, Y_INTERCEPT_FE)
    EDGE_AF = BLOATED_VERTEX_A[0] # (y=x) vertial edge on the left side of the hexagon
    EDGE_CD = BLOATED_VERTEX_C[0] # (y=x) vertical edge on the right side of the hexagon

    BLOATED_OBSTACLES['hexagon'] = ((BLOATED_VERTEX_A, BLOATED_VERTEX_B, BLOATED_VERTEX_C,
                                     BLOATED_VERTEX_D, BLOATED_VERTEX_E, BLOATED_VERTEX_F),
                                     (EDGE_AB, EDGE_BC, EDGE_ED, EDGE_FE, EDGE_AF, EDGE_CD))

    # Border rectangles to add 5 unit bloat to walls
    # Top Left Border
    TOP_LEFT_BLOATED_BORDER_X, TOP_LEFT_BLOATED_BORDER_Y = 0, 0
    TOP_LEFT_BLOATED_BORDER_WIDTH = BLOATED_FIRST_LEFT_RECTANGLE_X
    TOP_LEFT_BLOATED_BORDER_HEIGHT = CLEARANCE
    BLOATED_OBSTACLES['top_left_border'] = ((TOP_LEFT_BLOATED_BORDER_X, TOP_LEFT_BLOATED_BORDER_Y),
                                            (TOP_LEFT_BLOATED_BORDER_WIDTH, TOP_LEFT_BLOATED_BORDER_HEIGHT))
    # Rest of Top Border
    TOP_BLOATED_BORDER_X, TOP_BLOATED_BORDER_Y = FIRST_LEFT_RECTANGLE_X+FIRST_LEFT_RECTANGLE_WIDTH+CLEARANCE,0
    TOP_BLOATED_BORDER_WIDTH = WINDOW_WIDTH-TOP_BLOATED_BORDER_X
    TOP_BLOATED_BORDER_HEIGHT = CLEARANCE
    BLOATED_OBSTACLES['top_border'] = ((TOP_BLOATED_BORDER_X, TOP_BLOATED_BORDER_Y),
                                       (TOP_BLOATED_BORDER_WIDTH, TOP_BLOATED_BORDER_HEIGHT))
    # Left Vertical Border
    BLOATED_LEFT_VERTICAL_BORDER_X, BLOATED_LEFT_VERTICAL_BORDER_Y = 0, 0
    BLOATED_LEFT_VERTICAL_BORDER_WIDTH, BLOATED_LEFT_VERTICAL_BORDER_HEIGHT = CLEARANCE, WINDOW_HEIGHT
    BLOATED_OBSTACLES['left_vertical_border'] = ((BLOATED_LEFT_VERTICAL_BORDER_X, BLOATED_LEFT_VERTICAL_BORDER_Y),
                                                 (BLOATED_LEFT_VERTICAL_BORDER_WIDTH, BLOATED_LEFT_VERTICAL_BORDER_HEIGHT))
    # Bottom Border on the Left
    BLOATED_BOTTOM_LEFT_HORIZONTAL_BORDER_X = CLEARANCE
    BLOATED_BOTTOM_LEFT_HORIZONTAL_BORDER_Y = WINDOW_HEIGHT-CLEARANCE
    BLOATED_BOTTOM_LEFT_HORIZONTAL_BORDER_WIDTH = BLOATED_SECOND_LEFT_RECTANGLE_X-CLEARANCE
    BLOATED_BOTTOM_LEFT_HORIZONTAL_BORDER_HEIGHT = CLEARANCE
    BLOATED_OBSTACLES['bottom_left_horizontal_border'] = ((BLOATED_BOTTOM_LEFT_HORIZONTAL_BORDER_X, BLOATED_BOTTOM_LEFT_HORIZONTAL_BORDER_Y),
                                                          (BLOATED_BOTTOM_LEFT_HORIZONTAL_BORDER_WIDTH, BLOATED_BOTTOM_LEFT_HORIZONTAL_BORDER_HEIGHT))
    # Rest of Bottom Border
    BLOATED_BOTTOM_HORIZONTAL_BORDER_X = BLOATED_SECOND_LEFT_RECTANGLE_X+BLOATED_SECOND_LEFT_RECTANGLE_WIDTH
    BLOATED_BOTTOM_HORIZONTAL_BORDER_Y = WINDOW_HEIGHT-CLEARANCE
    BLOATED_HORIZONTAL_BORDER_WIDTH = WINDOW_WIDTH-BLOATED_BOTTOM_HORIZONTAL_BORDER_X
    BLOATED_HORIZONTAL_BORDER_HEIGHT = CLEARANCE
    BLOATED_OBSTACLES['bottom_horizontal_border'] = ((BLOATED_BOTTOM_HORIZONTAL_BORDER_X, BLOATED_BOTTOM_HORIZONTAL_BORDER_Y),
                                                     (BLOATED_HORIZONTAL_BORDER_WIDTH, BLOATED_HORIZONTAL_BORDER_HEIGHT))
    # Right Border
    BLOATED_RIGHT_VERTICAL_BORDER_X = WINDOW_WIDTH - CLEARANCE
    BLOATED_RIGHT_VERTICAL_BORDER_Y = CLEARANCE
    BLOATED_RIGHT_VERTICAL_BORDER_WIDTH = CLEARANCE
    BLOATED_RIGHT_VERTICAL_BORDER_HEIGHT = WINDOW_HEIGHT-2*CLEARANCE
    BLOATED_OBSTACLES['right_vertical_border'] = ((BLOATED_RIGHT_VERTICAL_BORDER_X, BLOATED_RIGHT_VERTICAL_BORDER_Y),
                                                  (BLOATED_RIGHT_VERTICAL_BORDER_WIDTH, BLOATED_RIGHT_VERTICAL_BORDER_HEIGHT))
    # Third Obstacle On right is divided into 2 horizontal rectangles & 1 vertical rectangle
    RIGHT_TOP_HORIZONTAL_RECTANGLE_X = WINDOW_WIDTH - 100 - 200
    RIGHT_TOP_HORIZONTAL_RECTANGLE_Y = 50
    RIGHT_TOP_HORIZONTAL_RECTANGLE_WIDTH = 200
    RIGHT_TOP_HORIZONTAL_RECTANGLE_HEIGHT = 75
    OBSTACLES['right_top_horizontal_rectangle']=((RIGHT_TOP_HORIZONTAL_RECTANGLE_X, RIGHT_TOP_HORIZONTAL_RECTANGLE_Y),
                                                 (RIGHT_TOP_HORIZONTAL_RECTANGLE_WIDTH, RIGHT_TOP_HORIZONTAL_RECTANGLE_HEIGHT))
    
    BLOATED_RIGHT_TOP_HORIZONTAL_RECTANGLE_X = RIGHT_TOP_HORIZONTAL_RECTANGLE_X-CLEARANCE
    BLOATED_RIGHT_TOP_HORIZONTAL_RECTANGLE_Y = RIGHT_TOP_HORIZONTAL_RECTANGLE_Y-CLEARANCE
    BLOATED_RIGHT_TOP_HORIZONTAL_RECTANGLE_WIDTH = RIGHT_TOP_HORIZONTAL_RECTANGLE_WIDTH + 2* CLEARANCE
    BLOATED_RIGHT_TOP_HORIZONTAL_RECTANGLE_HEIGHT = RIGHT_TOP_HORIZONTAL_RECTANGLE_HEIGHT + 2* CLEARANCE
    BLOATED_OBSTACLES['right_top_horizontal_rectangle'] = ((BLOATED_RIGHT_TOP_HORIZONTAL_RECTANGLE_X, BLOATED_RIGHT_TOP_HORIZONTAL_RECTANGLE_Y),
                                                           (BLOATED_RIGHT_TOP_HORIZONTAL_RECTANGLE_WIDTH, BLOATED_RIGHT_TOP_HORIZONTAL_RECTANGLE_HEIGHT))

    RIGHT_VERTICAL_RECTANGLE_X = RIGHT_TOP_HORIZONTAL_RECTANGLE_X + 120
    RIGHT_VERTICAL_RECTANGLE_Y = RIGHT_TOP_HORIZONTAL_RECTANGLE_Y + RIGHT_TOP_HORIZONTAL_RECTANGLE_HEIGHT
    RIGHT_VERTICAL_RECTANGLE_WIDTH = WINDOW_WIDTH - RIGHT_TOP_HORIZONTAL_RECTANGLE_X - 100 - 120
    RIGHT_VERTICAL_RECTANGLE_HEIGHT = 400 - 2* RIGHT_TOP_HORIZONTAL_RECTANGLE_HEIGHT
    OBSTACLES['right_vertical_rectangle']=((RIGHT_VERTICAL_RECTANGLE_X, RIGHT_VERTICAL_RECTANGLE_Y),
                                           (RIGHT_VERTICAL_RECTANGLE_WIDTH, RIGHT_VERTICAL_RECTANGLE_HEIGHT))

    BLOATED_RIGHT_VERTICAL_RECTANGLE_X = RIGHT_VERTICAL_RECTANGLE_X - CLEARANCE
    BLOATED_RIGHT_VERTICAL_RECTANGLE_Y = RIGHT_VERTICAL_RECTANGLE_Y
    BLOATED_RIGHT_VERTICAL_RECTANGLE_WIDTH = RIGHT_VERTICAL_RECTANGLE_WIDTH + 2* CLEARANCE
    BLOATED_RIGHT_VERTICAL_RECTANGLE_HEIGHT = RIGHT_VERTICAL_RECTANGLE_HEIGHT + CLEARANCE
    BLOATED_OBSTACLES['right_vertical_rectangle'] = ((BLOATED_RIGHT_VERTICAL_RECTANGLE_X, BLOATED_RIGHT_VERTICAL_RECTANGLE_Y),
                                                     (BLOATED_RIGHT_VERTICAL_RECTANGLE_WIDTH, BLOATED_RIGHT_VERTICAL_RECTANGLE_HEIGHT))

    RIGHT_BOTTOM_HORIZONTAL_RECTANGLE_X = RIGHT_TOP_HORIZONTAL_RECTANGLE_X
    RIGHT_BOTTOM_HORIZONTAL_RECTANGLE_Y = RIGHT_TOP_HORIZONTAL_RECTANGLE_Y+75+RIGHT_VERTICAL_RECTANGLE_HEIGHT
    RIGHT_BOTTOM_HORIZONTAL_RECTANGLE_WIDTH = RIGHT_TOP_HORIZONTAL_RECTANGLE_WIDTH
    RIGHT_BOTTOM_HORIZONTAL_RECTANGLE_HEIGHT = RIGHT_TOP_HORIZONTAL_RECTANGLE_HEIGHT
    OBSTACLES['right_bottom_horizontal_rectangle'] = ((RIGHT_BOTTOM_HORIZONTAL_RECTANGLE_X, RIGHT_BOTTOM_HORIZONTAL_RECTANGLE_Y),
                                                      (RIGHT_BOTTOM_HORIZONTAL_RECTANGLE_WIDTH, RIGHT_BOTTOM_HORIZONTAL_RECTANGLE_HEIGHT))

    BLOATED_RIGHT_BOTTOM_HORIZONTAL_RECTANGLE_X = BLOATED_RIGHT_TOP_HORIZONTAL_RECTANGLE_X
    BLOATED_RIGHT_BOTTOM_HORIZONTAL_RECTANGLE_Y = RIGHT_BOTTOM_HORIZONTAL_RECTANGLE_Y - CLEARANCE
    BLOATED_RIGHT_BOTTOM_HORIZONTAL_RECTANGLE_WIDTH = RIGHT_TOP_HORIZONTAL_RECTANGLE_WIDTH + 2* CLEARANCE
    BLOATED_RIGHT_BOTTOM_HORIZONTAL_RECTANGLE_HEIGHT = RIGHT_BOTTOM_HORIZONTAL_RECTANGLE_HEIGHT + 2 * CLEARANCE
    BLOATED_OBSTACLES['right_bottom_horizontal_rectangle'] = ((BLOATED_RIGHT_BOTTOM_HORIZONTAL_RECTANGLE_X, BLOATED_RIGHT_BOTTOM_HORIZONTAL_RECTANGLE_Y),
                                                              (BLOATED_RIGHT_BOTTOM_HORIZONTAL_RECTANGLE_WIDTH, BLOATED_RIGHT_BOTTOM_HORIZONTAL_RECTANGLE_HEIGHT))
    BLOATED_OBSTACLES['clearance'] = CLEARANCE

    start, goal = get_user_inputs(WINDOW_WIDTH, WINDOW_HEIGHT, BLOATED_OBSTACLES)

    parent_nodes_map, full_path, dijkstra_run_time = dijkstra_optimal_path(start, goal,\
                                                                           WINDOW_WIDTH,\
                                                                            WINDOW_HEIGHT,\
                                                                            BLOATED_OBSTACLES)

    print(f"Dijkstra Algorithm Execution Time: {round(dijkstra_run_time,2)} seconds")

    # If an optimal path is found, create animation
    if len(full_path) > 1:
        animation_strt_time = time.time()

        WINDOW = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption('DIJKSTRA!')
        draw_environment(WINDOW, BACKROUND_COLOR, OBSTACLE_COLOR,\
                         BLOATED_OBSTACLE_COLOR, OBSTACLES, BLOATED_OBSTACLES)
        animate_explored_nodes(WINDOW, NODES_COLOR, parent_nodes_map)
        animate_optimal_path(WINDOW, PATH_COLOR, full_path)

        animation_end_time = time.time()
        animation_run_time = animation_end_time - animation_strt_time
        print(f"Animation Execution Time: {round(animation_run_time,2)} seconds, \n")
        print(f'Total execution time of search algorithm & animation \
               {round(dijkstra_run_time+animation_run_time, 2)} seconds')
        # Main game loop for event handling
        while True:
            for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                    sys.exit()
            fpsClock.tick(FPS)
    else:
        print('The algorithm could not find an optimal path')

if __name__ == "__main__":
    main()
