import heapq as hq
import math
from collections import deque

# Initialize Pygame window
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 500
def ObstacleCheck(x_cord, y_cord):
    # Hexagon Obstacle
    if (y_cord >= (-x_cord/math.sqrt(3) + 90 + 650/math.sqrt(3))) and (y_cord <= (-x_cord/math.sqrt(3) + 410 + 650/math.sqrt(3))) and (x_cord >= (650 - 80*math.sqrt(3))) and (x_cord <= (650 + 80*math.sqrt(3))) and (y_cord <= (x_cord/math.sqrt(3) + 410 -650/math.sqrt(3))) and (y_cord >= (x_cord/math.sqrt(3) + 90 -650/math.sqrt(3))):
        return 'point is in hexagon obstacle space, please choose another one'
   
    # First 2 rectangular obstacles
    elif (x_cord >=95 and x_cord<= 180 and y_cord <= 405) or (x_cord >=270 and x_cord <= 355 and y_cord >= 95):
        return'point is in one of the rectugular obstacles on the left of display, please choose other point'

    # 4th Obstacle on the right of display
    if (x_cord >= (1200-205-100) and x_cord <= (1200-100-85) and y_cord >= 45 and y_cord <= (85+45)) or (x_cord >= (1200-100-85) and x_cord >= (1200-95) and y_cord>=45 and y_cord <= (500-455)) or(x_cord >= (1200-205-100) and x_cord <= (1200-100-85) and y_cord >= (450-80) and y_cord <= (500-455)):
        return 'point is in the 4th obstacle on the right of display, please choose another point'
    # Outside all obstacles 
    elif (x_cord <=5 or x_cord >= (WINDOW_WIDTH-5)) or (y_cord <=5 or y_cord >= (WINDOW_HEIGHT-5)):
        return 'point is in the border region, retry'
    else:
        return 'clear'

# Initialize the open list and closed lists (set)
open_list = []
close_list = set()
prnt_node_map = {}
# creating tuple with cost to come, parent node and coordinate values (x,y) 
start_x, start_y = 6, 6
goal_x, goal_y = 1194, 338 
# worst case pts in coord at bottom left corner: (6, 6) end (1194, 162) 
# or goal wrt to coord at top left corner -> (1194, 338)

# if (start_x,start_y) == (goal_x, goal_y):
#     print('you chose the same starting and goal points, choose different points')

cost2come_start, parent_node = 0.0, (None, None)

n1 = (cost2come_start,(start_x, start_y))  

#Push elements to heap queue which also simultaneously heapifies the queue
hq.heappush(open_list, n1)
# update dictionary. (might need to adjust key, value order depending on backtracking function)
prnt_node_map[(start_x, start_y)] = parent_node
# while_loop_cntr = 0
# for_loop_cntr = 0
while len(open_list) > 0:
    active_node = hq.heappop(open_list)
    curnt_cost2c, curnt_x, curnt_y = active_node[0], active_node[1][0], active_node[1][1]

    close_list.add((curnt_x, curnt_y))

    # Check if we've reached the goal
    if (curnt_x, curnt_y) == (goal_x, goal_y): 
        print("Goal reached!")
        # need to add backtracking
        path = deque()  # deque to store parent nodes found
        path.appendleft((curnt_x, curnt_y))
        j = 1
        while j == 1:
            for n in prnt_node_map:
                if (curnt_x, curnt_y) == n:
                    # if prnt ==(None, None):
                    if prnt_node_map[n] == (None, None):
                        j +=1
                        break
                    else:
                        path.appendleft(prnt_node_map[n])
                        (curnt_x, curnt_y) = prnt_node_map[n]
        print('final x, final y:', curnt_x, curnt_y)
        print('goal x, goal y:', goal_x, goal_y, '\n')
        # print('while_loop counter =', while_loop_cntr)
        # print('for_loop_counter =', for_loop_cntr, '\n')
        print(len(prnt_node_map), '\n')
        # print(path)       
        print('Path found!')
        break

    # Explore neighbors (delta_x, delat_y, cost-to-come)
    for dx, dy, cost2c in ((0, 1, 1), (0, -1, 1), (-1, 0, 1), (1, 0, 1), (1, 1, 1.4), (-1, 1, 1.4), (-1, -1, 1.4), (1, -1, 1.4)): # 8 possible actions with their costs
        nx, ny = curnt_x + dx, curnt_y + dy

        if ObstacleCheck(nx, ny) != 'clear':
            continue

        if (curnt_x, curnt_y) == (start_x, start_y):
            new_cost2c = curnt_cost2c + cost2c
            prnt_node_map[(nx, ny)] = (curnt_x, curnt_y)
            hq.heappush(open_list, (new_cost2c, (nx,ny)))
            continue

        if (nx, ny) not in close_list:
            new_cost2c = curnt_cost2c + cost2c  # Update this based on actual cost
            # prnt_node_map[(nx, ny)] = (curnt_x, curnt_y)
            if all((nx, ny) != nod[1] for nod in open_list):  # checks whether new node is not already in open list
                # new_prnt_node = (curnt_x, curnt_y)
                prnt_node_map[(nx, ny)] = (curnt_x, curnt_y)
                hq.heappush(open_list, (new_cost2c, (nx,ny)))
            else:
                for i in range(len(open_list)):
                    if open_list[i][1] == (nx, ny):
                        old_c2c = open_list[i][0]
                        if new_cost2c < old_c2c:
                            open_list[i] = (new_cost2c, (nx, ny))
                            hq.heapify(open_list)
                            prnt_node_map[(nx, ny)] = (curnt_x, curnt_y)
        # for_loop_cntr +=1                    
    if len(open_list)== 0:
        print('No solution could be found')
        break
    # while_loop_cntr+=1
                
