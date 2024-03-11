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
        return'point is in one of the rectangular obstacles on the left of display, please choose another one'

    # 4th Obstacle on the right of display
    if (x_cord >= (1200-205-100) and x_cord <= (1200-100-85) and y_cord >= 45 and y_cord <= (85+45)) or (x_cord >= (1200-100-85) and x_cord >= (1200-95) and y_cord>=45 and y_cord <= (500-455)) or(x_cord >= (1200-205-100) and x_cord <= (1200-100-85) and y_cord >= (450-80) and y_cord <= (500-455)):
        return 'point is in the 4th obstacle on the right of display, please choose another one'
    # Outside all obstacles 
    elif (x_cord <=5 or x_cord >= (WINDOW_WIDTH-5)) or (y_cord <=5 or y_cord >= (WINDOW_HEIGHT-5)):
        return 'point is in the border region, retry'
    else:
        return 'clear'

# Initialize the open list and closed lists (set)
open_list = []
close_list = set()
prnt_node_map = {}
lowest_c2c_map ={}
# creating tuple with cost to come, parent node and coordinate values (x,y) 
start_x, start_y = 6, 6
goal_x, goal_y = 1194, 338
# worst case pts in coord at bottom left corner: (6, 6) end (1194, 162) 
# or goal wrt to coord at top left corner -> (1194, 338)

# if (start_x,start_y) == (goal_x, goal_y):
#     print('you chose the same starting and goal points, choose different points')

cost2c_start, parent_node = 0.0, None

n1 = (cost2c_start,(start_x, start_y))  

#Push elements to heap queue which also simultaneously heapifies the queue
hq.heappush(open_list, n1)
lowest_c2c_map[(start_x, start_y)] = cost2c_start
prnt_node_map[(start_x, start_y)] = parent_node
# while_loop_cntr = 0
# for_loop_cntr = 0
while len(open_list) > 0:
    active_node = hq.heappop(open_list)
    curnt_cost2c, curnt_x, curnt_y = active_node[0], active_node[1][0], active_node[1][1]

    if lowest_c2c_map.get(curnt_x, curnt_y) is not None and curnt_cost2c > lowest_c2c_map.get((curnt_x, curnt_y)):
        continue

    close_list.add((curnt_x, curnt_y))

    # Check if we've reached the goal
    if (curnt_x, curnt_y) == (goal_x, goal_y):
        print("Goal reached!")
        # Backtracking
        curnt_node = (curnt_x, curnt_y)
        path = deque()
        # Continue backtracking until you reach the start node
        while curnt_node is not None:
            path.appendleft(curnt_node)  # Add the current node to the path
            curnt_node = prnt_node_map.get(curnt_node)  # Move to the parent node

        print('final x, final y:', curnt_x, curnt_y)
        print('goal x, goal y:', goal_x, goal_y, '\n')
        # print('while_loop counter =', while_loop_cntr)
        # print('for_loop_counter =', for_loop_cntr, '\n')
        # print(len(prnt_node_map), '\n')
        print(f'size of the path deque {len(path)} \n')
        print('Path found!')
        break

    # Explore neighbors with 8 possible actions (delta_x, delat_y, cost-to-come)
    for dx, dy, cost2c in ((0, 1, 1), (0, -1, 1), (-1, 0, 1), (1, 0, 1), (1, 1, 1.4), (-1, 1, 1.4), (-1, -1, 1.4), (1, -1, 1.4)): 
        nx, ny = curnt_x + dx, curnt_y + dy

        if ObstacleCheck(nx, ny) != 'clear':
            continue

        if (nx, ny) not in close_list:
            new_cost2c = curnt_cost2c + cost2c  
            if (nx, ny) not in lowest_c2c_map or new_cost2c < lowest_c2c_map.get((nx, ny)):
                prnt_node_map[(nx, ny)] = (curnt_x, curnt_y)
                lowest_c2c_map[(nx,ny)] = new_cost2c
                hq.heappush(open_list, (new_cost2c, (nx,ny)))

    if len(open_list)== 0:
        print('No solution could be found')
        break
    # while_loop_cntr+=1