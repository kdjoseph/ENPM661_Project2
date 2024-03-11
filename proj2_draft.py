import pygame, sys, math
import heapq as hq
from collections import deque
from pygame.locals import *
pygame.init()

# Initialize Pygame window
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 500

####### DISJKSTRA ALGORITM ########################################################################################
def ObstacleCheck(x_cord, y_cord):
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
    else:
        return True

open_list = []  # to be used for heapq for nodes in the open-list
close_list = set() # set to store the nodes without c2c for fast comparison with new nodes
prnt_node_map = {} # dictionary (key=node, val=parent) for all the nodes visited mapped to their parents to be used for backtracking
lowest_c2c_map ={} # dictionary (key=node, val=c2c) to keep track of the nodes and their cost, used to ensure only lowest cost in open-list
path = deque()     # deque used for backtracking

def DijkstraAlgo():
    start_pt_trigger = 1
    goal_pt_trigger =1
    while start_pt_trigger ==1:
        try:
            start_x, startb_y = float(input('Enter starting point x-cordinate: ')), float(input('Enter starting point y-cordinate: '))
            # start_x, startb_y = 6, 6
        except:
            print('you did not enter a number, please enter only numbers')
            continue
        start_y = 500 - startb_y # convert from coordinate wrt to lower-left corner of display to upper left-corner coordinate
        if not ObstacleCheck(start_x, start_y):
            print('The chosen start point is in the obstacle space or too close to the border or out of the display dimensions, choose another one.')
        else:
            start_pt_trigger = 0
    while goal_pt_trigger == 1:
        try:
            goal_x, goalb_y = float(input('Enter goal point x- cordinate: ')), float(input('Enter goal point y-cordinate: '))
            # goal_x, goalb_y = 12, 12
        except:
            print('you did not enter a number, please enter only numbers')
            continue
        goal_y = 500 - goalb_y  # convert from coordinate wrt to lower-left corner of display to upper left-corner coordinate
        if not ObstacleCheck(goal_x, goal_y):
            print('The chosen goal point is in the obstacle space or too close to the border or out of the display dimensions, choose another one.')
            continue
        if (start_x,start_y) == (goal_x, goal_y):
            print('you chose the same starting and goal points, choose different starting and goal points')
        else:
            goal_pt_trigger = 0
    # worst case pts in coord at bottom left corner: (6, 6) end (1194, 162) or goal wrt to coord at top left corner -> (1194, 338)

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
            print("Goal reached!")
            # Backtracking
            curnt_node = (curnt_x, curnt_y)
            # Continue backtracking until start node reached
            while curnt_node is not None:
                path.appendleft(curnt_node)  # Add the current node to the path
                curnt_node = prnt_node_map.get(curnt_node)  # Move to the parent node to now search for its parent

            print('final x, final y:', curnt_x, curnt_y)
            print('goal x, goal y:', goal_x, goal_y, '\n')
            # print('while_loop counter =', while_loop_cntr)
            # print('for_loop_counter =', for_loop_cntr, '\n')
            # print(len(prnt_node_map), '\n')
            # print(f'size of the path deque {len(path)} \n')
            print('Path found!')
            break

        # Explore neighbors with 8 possible actions (delta_x, delat_y, cost-to-come)
        for dx, dy, cost2c in ((0, 1, 1), (0, -1, 1), (-1, 0, 1), (1, 0, 1), (1, 1, 1.4), (-1, 1, 1.4), (-1, -1, 1.4), (1, -1, 1.4)): 
            nx, ny = curnt_x + dx, curnt_y + dy

            if not ObstacleCheck(nx,ny):
                continue

            if (nx, ny) not in close_list:
                new_cost2c = curnt_cost2c + cost2c
                # Only updated the heapq & other dictionaries if new node not in lowest-cost map or if now it's the lowe-cost node  
                if (nx, ny) not in lowest_c2c_map or new_cost2c < lowest_c2c_map.get((nx, ny)):
                    prnt_node_map[(nx, ny)] = (curnt_x, curnt_y)
                    lowest_c2c_map[(nx,ny)] = new_cost2c
                    hq.heappush(open_list, (new_cost2c, (nx,ny)))
        # Stop if the algorithm can't find a solution
        if len(open_list)== 0:
            print('No solution could be found')
            break

# Colours
BACKGROUND = (0, 40, 255)
RED = (255, 30, 70)
YELLOW = (255, 255, 0)
GREEN = (153, 255, 153)
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
# A starts on left verticle side at the top, then goes down counter clock-wise
bVertxA = (650-80*math.sqrt(3), 170)
bVertxB = (650, 90)
bVertxC = (650+80*math.sqrt(3), 170)
bVertxD = (650+80*math.sqrt(3), 330)
bVertxE = (650, 410)
bVertxF = (650-80*math.sqrt(3), 330)

# Function to draw obstacles and environment
def draw_environment(WINDOW):
    WINDOW.fill(BACKGROUND)
    # Bloated obstacles for visualization
    pygame.draw.polygon(WINDOW, YELLOW, (bVertxA, bVertxB, bVertxC, bVertxD, bVertxE, bVertxF))
    pygame.draw.rect(WINDOW, YELLOW, (95, 0, 85, 405))
    pygame.draw.rect(WINDOW, YELLOW, (270, 95, 85, 405))
    pygame.draw.rect(WINDOW, YELLOW, (1200-(205+100), 45, 210, 85))
    pygame.draw.rect(WINDOW, YELLOW, (1200-(100+200-115), 50+75, 90, 400-75-75))
    pygame.draw.rect(WINDOW, YELLOW, (1200-(205+100), 50+75+(400-75*2)-5, 210, 85))
    # Actual obstacles
    pygame.draw.polygon(WINDOW, RED, (vertxA, vertxB, vertxC, vertxD, vertxE, vertxF))
    pygame.draw.rect(WINDOW, RED, (100, 0, 75, 400))
    pygame.draw.rect(WINDOW, RED, (100+75+100, 500-400, 75, 400))
    pygame.draw.rect(WINDOW, RED, (1200-100-200, 50, 200, 75))
    pygame.draw.rect(WINDOW, RED, (1200-100-200+120, 50+75, 200-120, 400-75-75))
    pygame.draw.rect(WINDOW, RED, (1200-100-200, 50+75+400-75-75, 200, 75))
    pygame.display.update()

def animate_explored_nodes(WINDOW, prnt_node_map, nodes_per_frame=10):
    node_list = list(prnt_node_map.keys())
    for i in range(0, len(node_list), nodes_per_frame):
        for node in node_list[i:i+nodes_per_frame]:
            pygame.draw.circle(WINDOW, GREEN, (int(node[0]), int(node[1])), 1)
        pygame.display.update()
        pygame.time.delay(1)

# Animate optimal path
def animate_optimal_path(WINDOW, path):
    for node in path:
        pygame.draw.circle(WINDOW, WHITE, (int(node[0]), int(node[1])), 2)
        pygame.display.update()
        pygame.time.delay(3) 

def main():

    DijkstraAlgo()
    WINDOW = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption('DIJKSTRA!')

    draw_environment(WINDOW)
    animate_explored_nodes(WINDOW, prnt_node_map)
    animate_optimal_path(WINDOW, path)

    # Main game loop for event handling
    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
        fpsClock.tick(FPS)

if __name__ == "__main__":
    main()
