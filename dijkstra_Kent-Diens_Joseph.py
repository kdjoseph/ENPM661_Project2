import heapq as hq
import sys
import math
import time
from collections import deque
import pygame
from pygame.locals import *
from numba import jit
pygame.init()


    

@jit(nopython=True)
def is_in_obstacle(x, y, window_width, window_height):
    """Checks if a point is in the obstacle space. 
    Takes the point's (x,y) as input and returns False 
    if it's in the obstacle space & True if it's not. """
    # Hexagon Obstacle
    if (-x/math.sqrt(3) + 90 + 650/math.sqrt(3))<= y <= (-x/math.sqrt(3) + 410 + 650/math.sqrt(3)) and \
        ((650 - 80*math.sqrt(3)) <= x <= (650 + 80*math.sqrt(3))) and \
        ((x/math.sqrt(3) + 90 -650/math.sqrt(3)) <= y <= (x/math.sqrt(3) + 410 -650/math.sqrt(3))):
        return True
    # First 2 rectangular obstacles
    if (95 <= x<= 180 and y <= 405) or (270 <= x <= 355 and y >= 95):
        return True
    # 4th Obstacle, on the right of display
    if ((1200-(205+100)) <= x <= (1200-100-85) and 45 <= y <= (85+45)) or \
        ((1200-100-85) <= x <= (1200-95) and 45 <= y <= (50+405)) or \
            ((1200-205-100) <= x <= (1200-100-85) and (450-80) <= y <= (450+5)):
        return True
    # Border region outside all obstacles
    if (x <=5 or x >= (window_width-5)) or (y <=5 or y >= (window_height-5)):
        return True
    else:               # not in obstacle space
        return False

def retrace_steps(vertex, parent_node_map, path):
    """ backtracks to create a path from the current node back to the initial node. 
    Returns printout saying whether a path was found or not"""
    # Keep backtracking until start node reached
    while vertex is not None:
        path.appendleft(vertex)  # Add the current node to the path
        vertex = parent_node_map.get(vertex)  # Move to the parent node to now search for its parent
    if len(path)>1:
        return print('Path found! \n')
    else:
        return print('Could not find a path \n')

def get_user_inputs(window_width, window_height):
    """ Asks the user to enter the start and goal points, and returns them"""

    while True:
        try:
            # worst case start wrt to coord at bottom-left corner:
            # start (6,6), goal (1194, 162) or goal (1194, 338)
            start_x, startb_y = float(input('Enter starting point x-cordinate : ')), \
                float(input('Enter starting point y-cordinate: '))
            # convert from coordinate w.r.t the lower-left corner of display to
            # upper left-corner coordinate (pygame coord)
            start_y = 500 - startb_y
            if is_in_obstacle(start_x, start_y, window_width, window_height):
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
            if is_in_obstacle(goal_x, goal_y, window_width, window_height):
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

def dijkstra_path(start, goal, window_size):
    """Searches for optimal path from a user-input starting point to a goal point """
    window_width, window_height = window_size
    open_list = []  # to be used for heapq for nodes in the open-list
    closed_list = set() # set to store the nodes without c2c for fast comparison with new nodes
    parent_node_map = {} # dict (key= visited-node, value=parent)
    lowest_c2c_map ={} # dict (key=node, val=c2c) to keep track of the nodes and their cost
    path = deque()     # deque used for backtracking

    start_x, start_y, goal_x, goal_y = start, goal
    dijkstra_strt_time = time.time() # start time of algorithm
    cost2c_start = 0.0 # starting point cost-to-come

    #Push first node to heap queue which also simultaneously heapifies the queue
    hq.heappush(open_list, (cost2c_start,(start_x, start_y)))
    # Update lowest cost and parent node maps with starting point info
    lowest_c2c_map[(start_x, start_y)] = cost2c_start
    parent_node_map[(start_x, start_y)] = None

    # Dijkstra while loop to generate new nodes
    while len(open_list) > 0:
        current_cost2come, (current_x, current_y) = hq.heappop(open_list)
        # Filter out nodes that had higher cost in the open-list compared to in the lowest cost map
        if lowest_c2c_map.get(current_x, current_y) is not None and \
            current_cost2come > lowest_c2c_map.get((current_x, current_y)):
            continue

        closed_list.add((current_x, current_y))  # add popped-node into closed set
        # Check if we've reached the goal,if yes, backtrack to find path
        if (current_x, current_y) == (goal_x, goal_y):
            print(f"\nGoal point {(current_x, window_height-current_y)} reached!")
            # Backtracking
            current_node = (current_x, current_y)
            retrace_steps(current_node, parent_node_map, path)
            dijkstra_end_time = time.time() # end time of algorithm
            return parent_node_map, path, dijkstra_end_time-dijkstra_strt_time

        # Explore neighbors with 8 possible actions (delta_x, delat_y, cost-to-come)
        for dx, dy, cost2come in ((0, 1, 1), (0, -1, 1), (-1, 0, 1), (1, 0, 1), \
                               (1, 1, 1.4), (-1, 1, 1.4), (-1, -1, 1.4), (1, -1, 1.4)):
            new_x, new_y = current_x + dx, current_y + dy
            # Ignore node if it's in the obstacle space.
            if is_in_obstacle(new_x,new_y, window_width, window_height):
                continue

            if (new_x, new_y) not in closed_list:
                new_cost2c = current_cost2come + cost2come
                # Only update the heapq & other dictionaries if new node not in lowest-cost map
                # or if now it's the lowest-cost node
                if (new_x, new_y) not in lowest_c2c_map or \
                    new_cost2c < lowest_c2c_map.get((new_x, new_y)):
                    parent_node_map[(new_x, new_y)] = (current_x, current_y)
                    lowest_c2c_map[(new_x,new_y)] = new_cost2c
                    hq.heappush(open_list, (new_cost2c, (new_x,new_y)))
        # Stop if the algorithm can't find a solution
        if len(open_list)== 0:
            print('No solution could be found')
            break

#### ANIMATION SECTION ####
# Colours (R, G, B)
BACKGROUND = (0, 40, 255) # blue
RED = (255, 30, 70)
YELLOW = (255, 255, 0)
GREEN = (0, 100, 0)
WHITE = (255, 255, 255)

# Game Setup
FPS = 60
fpsClock = pygame.time.Clock()

# Vertices for Hexagon Obstacle
vertxA = (650 - 75*math.sqrt(3), 175)
vertxB = (vertxA[0], vertxA[1]+150)
vertxC = (650, vertxB[1]+75)
vertxD = (650+75*math.sqrt(3), vertxB[1])
vertxE = (vertxD[0], vertxD[1]-150)
vertxF = (650, 100)

# Vertices for hexagon bloated by 5 mm to add clearance
# Point A starts on left verticle side at the top, then goes down counter clock-wise
bVertxA = (650-80*math.sqrt(3), 170)
bVertxB = (650, 90)
bVertxC = (650+80*math.sqrt(3), 170)
bVertxD = (650+80*math.sqrt(3), 330)
bVertxE = (650, 410)
bVertxF = (650-80*math.sqrt(3), 330)

# Border rectangles to add 5 unit bloat to walls
wrect1 = pygame.Rect(0,0, 95, 5)
wrect2 = pygame.Rect(0, 5, 5, 495 )
wrect3 = pygame.Rect(5,495, 275-5, 5)
wrect4 = pygame.Rect(100+75+5, 0, 1200-180, 5)
wrect5 = pygame.Rect(1200-5, 5, 5, 500-5)
wrect6 = pygame.Rect(100+75+100+75+5, 495, 1200-250, 5)

def draw_environment(WINDOW):
    """ Draws the obstacles and background """
    WINDOW.fill(BACKGROUND)
    # Highligh 5 unit bloat around obstacles
    pygame.draw.polygon(WINDOW, YELLOW, (bVertxA, bVertxB, bVertxC, bVertxD, bVertxE, bVertxF))
    pygame.draw.rect(WINDOW, YELLOW, (95, 0, 85, 405))
    pygame.draw.rect(WINDOW, YELLOW, (270, 95, 85, 405))
    pygame.draw.rect(WINDOW, YELLOW, (1200-(205+100), 45, 210, 85))
    pygame.draw.rect(WINDOW, YELLOW, (1200-(100+200-115), 50+75, 90, 400-75-75))
    pygame.draw.rect(WINDOW, YELLOW, (1200-(205+100), 50+75+(400-75*2)-5, 210, 85))
    # Drawing the 5 unit bloat for walls
    pygame.draw.rect(WINDOW, YELLOW, wrect1)
    pygame.draw.rect(WINDOW, YELLOW, wrect2)
    pygame.draw.rect(WINDOW, YELLOW, wrect3)
    pygame.draw.rect(WINDOW, YELLOW, wrect4)
    pygame.draw.rect(WINDOW, YELLOW, wrect5)
    pygame.draw.rect(WINDOW, YELLOW, wrect6)
    # Actual obstacles
    pygame.draw.polygon(WINDOW, RED, (vertxA, vertxB, vertxC, vertxD, vertxE, vertxF))
    pygame.draw.rect(WINDOW, RED, (100, 0, 75, 400))
    pygame.draw.rect(WINDOW, RED, (100+75+100, 500-400, 75, 400))
    pygame.draw.rect(WINDOW, RED, (1200-100-200, 50, 200, 75))
    pygame.draw.rect(WINDOW, RED, (1200-100-200+120, 50+75, 200-120, 400-75-75))
    pygame.draw.rect(WINDOW, RED, (1200-100-200, 50+75+400-75-75, 200, 75))
    pygame.display.update()

def animate_explored_nodes(WINDOW, parent_node_map, nodes_per_frame=10):
    """ Animates the explored nodes. Takes the display-window and 
    parent-node map dictionary as inputs"""
    node_list = list(parent_node_map.keys())
    for i in range(0, len(node_list), nodes_per_frame):
        for node in node_list[i:i+nodes_per_frame]:
            pygame.draw.circle(WINDOW, GREEN, (int(node[0]), int(node[1])), 1)
        pygame.display.update()
        pygame.time.delay(1)  # delay to adjust animation speed

def animate_optimal_path(WINDOW, path):
    """Function to animate moving from start to goal using the optimal path"""
    for node in path:
        pygame.draw.circle(WINDOW, WHITE, (int(node[0]), int(node[1])), 2)
        pygame.display.update()
        pygame.time.delay(4)  # delay to adjust animation speed

#### MAIN FUNCTION (start algorithm, then animates) ###############
def main():
    """ Main function to start and animate the algorithm search"""
    # Initialize Pygame window
    WINDOW_WIDTH = 1200
    WINDOW_HEIGHT = 500
    WINDOW_SIZE = (WINDOW_WIDTH, WINDOW_HEIGHT)
    CLEARANCE = 5

    BLUE_BACKROUND = (0, 40, 255) # blue
    RED_OBSTACLES = (255, 30, 70)   # RED
    YELLOW_CLEARANCE = (255, 255, 0)
    GREEN_NODES = (0, 100, 0)
    WHITE_PATH = (255, 255, 255)

    # Game Setup
    FPS = 60
    fpsClock = pygame.time.Clock()
    # x, y, width, height
    HEXAGON_CENTER = (650, WINDOW_HEIGHT/2)
    HEXAGON_SIDE_LENGTH = 150
    BLOATED_HEXAGON_SIDE_LENGTH = HEXAGON_SIDE_LENGTH + 2 * CLEARANCE
    # Vertex A starts from top left vertical side, then go clockwise for the other vertices
    VERTEX_A = (HEXAGON_CENTER[0] - 0.5*HEXAGON_SIDE_LENGTH*math.sqrt(3),
                HEXAGON_CENTER[1]-HEXAGON_SIDE_LENGTH+HEXAGON_SIDE_LENGTH/2)
    VERTEX_B = (HEXAGON_CENTER[0], HEXAGON_CENTER[1]-HEXAGON_SIDE_LENGTH)
    VERTEX_C = (HEXAGON_CENTER[0]+0.5*HEXAGON_SIDE_LENGTH*math.sqrt(3), VERTEX_A[1])
    VERTEX_D = (VERTEX_C[0], VERTEX_C +HEXAGON_SIDE_LENGTH)
    VERTEX_E = (HEXAGON_CENTER[0], HEXAGON_CENTER[1]+HEXAGON_SIDE_LENGTH)
    VERTEX_F = (VERTEX_A[0], VERTEX_A[1]+HEXAGON_SIDE_LENGTH)

    # Vertices for Hexagon Obstacle
    # vertxA = (650 - 75*math.sqrt(3), 175)
    # vertxB = (vertxA[0], vertxA[1]+150)
    # vertxC = (650, vertxB[1]+75)
    # vertxD = (650+75*math.sqrt(3), vertxB[1])
    # vertxE = (vertxD[0], vertxD[1]-150)
    # vertxF = (650, 100)

    # Vertices for hexagon bloated by 5 mm to add clearance
    # Point A starts on left verticle side at the top, then goes down counter clock-wise
    # bVertxA = (650-80*math.sqrt(3), 170)
    # bVertxB = (650, 90)
    # bVertxC = (650+80*math.sqrt(3), 170)
    # bVertxD = (650+80*math.sqrt(3), 330)
    # bVertxE = (650, 410)
    # bVertxF = (650-80*math.sqrt(3), 330)

    BLOATED_VERTEX_A = (HEXAGON_CENTER[0] - 0.5*BLOATED_HEXAGON_SIDE_LENGTH*math.sqrt(3), 
                        HEXAGON_CENTER[1]-BLOATED_HEXAGON_SIDE_LENGTH+BLOATED_HEXAGON_SIDE_LENGTH/2)
    BLOATED_VERTEX_B = (HEXAGON_CENTER[0], HEXAGON_CENTER[1]-BLOATED_HEXAGON_SIDE_LENGTH)
    BLOATED_VERTEX_C = (HEXAGON_CENTER[0]+0.5*BLOATED_HEXAGON_SIDE_LENGTH*math.sqrt(3), BLOATED_VERTEX_A[1])
    BLOATED_VERTEX_D = (BLOATED_VERTEX_C[0], BLOATED_VERTEX_C +BLOATED_HEXAGON_SIDE_LENGTH)
    BLOATED_VERTEX_E = (HEXAGON_CENTER[0], HEXAGON_CENTER[1]+BLOATED_HEXAGON_SIDE_LENGTH)
    BLOATED_VERTEX_F = (BLOATED_VERTEX_A[0], BLOATED_VERTEX_A[1]+BLOATED_HEXAGON_SIDE_LENGTH)

    # Border rectangles to add 5 unit bloat to walls
    wrect1 = pygame.Rect(0,0, 95, 5)
    wrect2 = pygame.Rect(0, 5, 5, 495 )
    wrect3 = pygame.Rect(5,495, 275-5, 5)
    wrect4 = pygame.Rect(100+75+5, 0, 1200-180, 5)
    wrect5 = pygame.Rect(1200-5, 5, 5, 500-5)
    wrect6 = pygame.Rect(100+75+100+75+5, 495, 1200-250, 5)

    start, goal = get_user_inputs(WINDOW_WIDTH, WINDOW_HEIGHT)

    parent_nodes_map, full_path, dijkstra_run_time = dijkstra_path(start, goal, WINDOW_SIZE)

    print(f"Dijkstra Algorithm Execution Time: {round(dijkstra_run_time,2)} seconds")

    # If an optimal path is found, create animation
    if len(full_path) > 1:
        animation_strt_time = time.time()

        WINDOW = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption('DIJKSTRA!')
        draw_environment(WINDOW)
        animate_explored_nodes(WINDOW, parent_nodes_map)
        animate_optimal_path(WINDOW, full_path)

        animation_end_time = time.time()
        animation_run_time = animation_end_time - animation_strt_time
        print(f"Animation Execution Time: {animation_run_time} seconds, \n")
        print(f'Total execution time of search algorithm & animation \
               {dijkstra_run_time+animation_run_time} seconds')
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
