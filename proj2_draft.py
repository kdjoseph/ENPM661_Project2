import heapq as hq
import math

# Initialize your Pygame window
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 500
def ObstacleCheck(inputzX, inputzY):
    # Hexagon Obstacle
    if (inputzY >= (-inputzX/math.sqrt(3) + 90 + 650/math.sqrt(3))) and (inputzY <= (-inputzX/math.sqrt(3) + 410 + 650/math.sqrt(3))) and (inputzX >= (650 - 80*math.sqrt(3))) and (inputzX <= (650 + 80*math.sqrt(3))) and (inputzY <= (inputzX/math.sqrt(3) + 410 -650/math.sqrt(3))) and (inputzY >= (inputzX/math.sqrt(3) + 90 -650/math.sqrt(3))):
        return 'point is in hexagon obstacle space, please choose another one'
   
    # First 2 rectangular obstacles
    elif (inputzX >=95 and inputzX<= 180 and inputzY <= 405) or (inputzX >=270 and inputzX <= 355 and inputzY >= 95):
        return'point is in one of the rectugular obstacles on the left of display, please choose other point'

    # 3rd Obstacle on the right of display
    if (inputzX >= (1200-205-100) and inputzX <= (1200-100-85) and inputzY >= 45 and inputzY <= (85+45)) or (inputzX >= (1200-100-85) and inputzX >= (1200-95) and inputzY>=45 and inputzY <= (500-455)) or(inputzX >= (1200-205-100) and inputzX <= (1200-100-85) and inputzY >= (450-80) and inputzY <= (500-455)):
        print('point is in the third obstacle on the right of display, please choose another point')
    # Outside all obstacles 
    elif (inputzX <=5 or inputzX >= (WINDOW_WIDTH-5)) or (inputzY <=5 or inputzY >= (WINDOW_HEIGHT-5)):
        return 'point is in the border region, retry'
    else:
        return 'clear'

# Initialize the open list and closed lists (set)
open_list = []
close_list = set()

# creating tuple with cost to come, parent node and coordinate values (x,y) 
start_x, start_y = 6, 6
goal_x, goal_y = 200, 400
cost2come_start, parent_node = 0.0, (None, None)

n1 = (cost2come_start, parent_node, (start_x, start_y))  

#Push elements to heap queue which also simultaneously heapifies the queue
hq.heappush(open_list, n1)



while len(open_list) > 0:
    active_node = hq.heappop(open_list)
    curnt_cost, prnt_node, curnt_x, curnt_y = active_node[0], active_node[1], active_node[2][0], active_node[2][1]

    close_list.add((curnt_x, curnt_y))

    # Check if we've reached the goal
    if (curnt_x, curnt_y) == (goal_x, goal_y): 
        print("Goal reached!")
        # need to add backtracking
        break

    # Explore neighbors
    for dx, dy, cost2c in ((0, 1, 1), (0, -1, 1), (-1, 0, 1), (1, 0, 1), (1, 1, 1.4), (-1, 1, 1.4), (-1, -1, 1.4), (1, -1, 1.4)): # 8 possible actions with their costs
        nx, ny = curnt_x + dx, curnt_y + dy

        if ObstacleCheck(nx, ny) != 'clear':
            continue

        if (nx, ny) not in close_list:
            new_cost2c = curnt_cost + cost2c  
            if all((nx, ny) != nod[2] for nod in open_list):  # checks whether new node is not already in open list
                new_prnt_node = (curnt_x, curnt_y)
                hq.heappush(open_list, (new_cost2c, new_prnt_node, (nx,ny)))
            else:
                for i in range(len(open_list)):
                    if open_list[i][2] == (nx, ny):
                        old_c2c = open_list[i][0]
                        if new_cost2c < old_c2c:
                            open_list[i] = (new_cost2c, (curnt_x, curnt_y), (nx, ny))
                            hq.heapify(open_list)
                
             
