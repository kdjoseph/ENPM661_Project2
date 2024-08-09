import pygame, sys, math, time
import heapq as hq
from collections import deque
from pygame.locals import *
pygame.init()

# Initialize Pygame window
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 500

def ObstacleCheck(x_cord, y_cord):
    """Checks if a point is in the obstacle space. Takes the point's (x,y) as input and returns False if it's in the obstacle space & True if it's not. """
    # Hexagon Obstacle
    if (y_cord >= (-x_cord/math.sqrt(3) + 90 + 650/math.sqrt(3))) and (y_cord <= (-x_cord/math.sqrt(3) + 410 + 650/math.sqrt(3))) and (x_cord >= (650 - 80*math.sqrt(3))) and (x_cord <= (650 + 80*math.sqrt(3))) and (y_cord <= (x_cord/math.sqrt(3) + 410 -650/math.sqrt(3))) and (y_cord >= (x_cord/math.sqrt(3) + 90 -650/math.sqrt(3))):
        return False
    # First 2 rectangular obstacles
    elif (x_cord >=95 and x_cord<= 180 and y_cord <= 405) or (x_cord >=270 and x_cord <= 355 and y_cord >= 95):
        return False
    # 4th Obstacle, on the right of display
    if (x_cord >= (1200-(205+100)) and x_cord <= (1200-100-85) and y_cord >= 45 and y_cord <= (85+45)) or (x_cord >= (1200-100-85) and x_cord <= (1200-95) and y_cord>=45 and y_cord <= (50+405)) or(x_cord >= (1200-205-100) and x_cord <= (1200-100-85) and y_cord >= (450-80) and y_cord <= (450+5)):
        return False
    # Border region outside all obstacles 
    elif (x_cord <=5 or x_cord >= (WINDOW_WIDTH-5)) or (y_cord <=5 or y_cord >= (WINDOW_HEIGHT-5)):
        return False
    else:               # not in obstacle space
        return True

open_list = []  # to be used for heapq for nodes in the open-list
close_list = set() # set to store the nodes without c2c for fast comparison with new nodes
prnt_node_map = {} # dictionary (key=node, val=parent) for all the nodes visited mapped to their parents to be used for backtracking
lowest_c2c_map ={} # dictionary (key=node, val=c2c) to keep track of the nodes and their cost, used to ensure only lowest cost in open-list
path = deque()     # deque used for backtracking

def retrace_steps(vertex):
    """ backtracks to create a path from the current node back to the initial node. Returns printout saying whether a path was found or not"""
    # Keep backtracking until start node reached
    while vertex is not None:
        path.appendleft(vertex)  # Add the current node to the path
        vertex = prnt_node_map.get(vertex)  # Move to the parent node to now search for its parent
    if len(path)>1:
        return print('Path found! \n')
    else:
        return print('Could not find a path \n')

def user_inputs():
    """ Asks the user to enter the start and goal points, and returns them"""
    start_pt_trigger = 1
    goal_pt_trigger =1
    while start_pt_trigger ==1:
        try:
            start_x, startb_y = float(input('Enter starting point x-cordinate : ')), float(input('Enter starting point y-cordinate: '))
            # worst case start wrt to coord at bottom-left corner: start (6,6), goal (1194, 162) or goal (1194, 338)
        except:
            print('you did not enter a number, please enter only numbers')
            continue
        start_y = 500 - startb_y # convert from coordinate wrt to lower-left corner of display to upper left-corner coordinate (pygame coord)
        if not ObstacleCheck(start_x, start_y):
            print('The chosen start point is in the obstacle space or too close to the border or out of the display dimensions, choose another one.')
        else:
            start_pt_trigger = 0
    while goal_pt_trigger == 1:
        try:
            goal_x, goalb_y = float(input('Enter goal point x- cordinate: ')), float(input('Enter goal point y-cordinate: '))
        except:
            print('you did not enter a number, please enter only numbers')
            continue
        goal_y = 500 - goalb_y  # convert from coordinate wrt to lower-left corner of display to upper left-corner coordinate (pygame coord)
        if not ObstacleCheck(goal_x, goal_y):
            print('The chosen goal point is in the obstacle space or too close to the border or out of the display dimensions, choose another one.')
            continue
        if (start_x,start_y) == (goal_x, goal_y):
            print('you chose the same starting and goal points, choose different starting and goal points')
        else:
            goal_pt_trigger = 0
    return start_x, start_y, goal_x, goal_y 

def DijkstraAlgo():
    """Searches for optimal path from a user-input starting point to a goal point """

    start_x, start_y, goal_x, goal_y = user_inputs()

    cost2c_start, parent_node = 0.0, None
    # creating tuple with cost to come and coordinate values (x,y) 
    n1 = (cost2c_start,(start_x, start_y))  

    #Push elements to heap queue which also simultaneously heapifies the queue
    hq.heappush(open_list, n1)
    # Update lowest cost and parent node maps with starting point info
    lowest_c2c_map[(start_x, start_y)] = cost2c_start
    prnt_node_map[(start_x, start_y)] = parent_node

    # Dijkstra while loop to generate new nodes
    while len(open_list) > 0:
        active_node = hq.heappop(open_list)
        curnt_cost2c, curnt_x, curnt_y = active_node[0], active_node[1][0], active_node[1][1]
        # Filter out nodes that had higher cost in the open-list compared to in the lowest cost map
        if lowest_c2c_map.get(curnt_x, curnt_y) is not None and curnt_cost2c > lowest_c2c_map.get((curnt_x, curnt_y)):
            continue

        close_list.add((curnt_x, curnt_y))  # add popped-node into closed list
        # Check if we've reached the goal,if yes, backtrack to find path
        if (curnt_x, curnt_y) == (goal_x, goal_y):
            print(f"\nGoal point {(curnt_x, WINDOW_HEIGHT-curnt_y)} reached!")
            # Backtracking
            curnt_node = (curnt_x, curnt_y)
            retrace_steps(curnt_node)
            break

        # Explore neighbors with 8 possible actions (delta_x, delat_y, cost-to-come)
        for dx, dy, cost2c in ((0, 1, 1), (0, -1, 1), (-1, 0, 1), (1, 0, 1), (1, 1, 1.4), (-1, 1, 1.4), (-1, -1, 1.4), (1, -1, 1.4)): 
            nx, ny = curnt_x + dx, curnt_y + dy
            # Ignore node if it's in the obstacle space.
            if not ObstacleCheck(nx,ny):
                continue

            if (nx, ny) not in close_list:
                new_cost2c = curnt_cost2c + cost2c
                # Only update the heapq & other dictionaries if new node not in lowest-cost map or if now it's the lowest-cost node  
                if (nx, ny) not in lowest_c2c_map or new_cost2c < lowest_c2c_map.get((nx, ny)):
                    prnt_node_map[(nx, ny)] = (curnt_x, curnt_y)
                    lowest_c2c_map[(nx,ny)] = new_cost2c
                    hq.heappush(open_list, (new_cost2c, (nx,ny)))
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

def animate_explored_nodes(WINDOW, prnt_node_map, nodes_per_frame=10):
    """ Animates the explored nodes. Takes the display-window and parent-node map dictionary as inputs"""
    node_list = list(prnt_node_map.keys())
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
    dijkstra_strt_time = time.time()
    DijkstraAlgo()
    dijkstra_end_time = time.time()
    dijkstra_run_time = dijkstra_end_time - dijkstra_strt_time  # calculates runtime of the search algorithm
    print(f"Dijkstra Algorithm Execution Time: {dijkstra_run_time} seconds")

    # If an optimal path is found, create animation
    if len(path) > 1: 
        animation_strt_time = time.time()  

        WINDOW = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption('DIJKSTRA!')
        draw_environment(WINDOW)
        animate_explored_nodes(WINDOW, prnt_node_map)
        animate_optimal_path(WINDOW, path)

        animation_end_time = time.time()
        animation_run_time = animation_end_time - animation_strt_time
        print(f"Animation Execution Time: {animation_run_time} seconds, \n")
        print(f'Total execution time of search algorithm & animation {dijkstra_run_time+animation_run_time} seconds')
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
