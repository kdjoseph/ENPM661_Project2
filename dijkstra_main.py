import config
import obstacles
import pathfinding
import visualization
import pygame
import sys
import time

def get_user_inputs():
    """Get start and goal points from user input

    Returns:
        start, goal (tuple): combined tuple of the x,y coordinates of the start & goal points

    Raises:
        ValueError: if the user does not enter a number
    """
    while True:
        try:
            start_x = float(input('Enter starting point x-coordinate: '))
            start_y = float(input('Enter starting point y-coordinate: '))
            start_y = config.WINDOW_HEIGHT - start_y # convert to pygame coordinates
            
            if obstacles.is_collision(start_x, start_y):
                print('Invalid start point. The point is in an obstacle, or too close to '
                      'the borders or outside of the borders.')
                continue
            break
        except ValueError:
            print('Please enter numeric values only.')    

    while True:
        try:      
            goal_x = float(input('Enter goal point x-coordinate: '))
            goal_y = float(input('Enter goal point y-coordinate: '))
            goal_y = config.WINDOW_HEIGHT - goal_y # convert to pygame coordinates
            
            if obstacles.is_collision(goal_x, goal_y):
                print('Invalid goal point. The point is in an obstacle. or too close to '
                      'the borders or outside of the borders')
                continue
                
            if (start_x, start_y) == (goal_x, goal_y):
                print('Start and goal points cannot be the same.')
                continue
                
            return (start_x, start_y), (goal_x, goal_y)
            
        except ValueError:
            print('Please enter numeric values only.')

def main():

    # Get user inputs
    start, goal = get_user_inputs()
    
    # Find path
    result = pathfinding.find_path(start, goal)
    print(f"\nPathfinding time: {result['runtime']:.2f} seconds")
    
    if result['success']:
        # Initialize pygame and create visualization
        pygame.init()
        window = pygame.display.set_mode((config.WINDOW_WIDTH, config.WINDOW_HEIGHT))
        pygame.display.set_caption('Dijkstra Pathfinding')
        
        # Animate solution
        animation_start_time = time.time()
        visualization.draw_environment(window)
        visualization.animate_explored_nodes(window, result['visited_nodes'])
        visualization.animate_optimal_path(window, result['path'])
        animation_end_time = time.time()
        
        print(f"Pathfinding time: {result['runtime']:.2f} seconds")
        print(f"Animation time: {animation_end_time - animation_start_time:.2f} seconds")

        # Main game loop
        clock = pygame.time.Clock()
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
            clock.tick(config.FPS)
    else:
        print('\n',result['message'])

if __name__ == "__main__":
    main()