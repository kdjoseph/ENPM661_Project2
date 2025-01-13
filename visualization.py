import pygame
import config

def draw_environment(window):
    """Draw the environment including obstacles
    Args:
        window (pygame surface)
    """
    window.fill(config.BACKGROUND_COLOR)

    # Draw bloated obstacles
    pygame.draw.rect(window, config.BLOATED_OBSTACLE_COLOR, 
                    pygame.Rect(config.FIRST_LEFT_RECTANGLE['bloated']['x'],
                                config.FIRST_LEFT_RECTANGLE['bloated']['y'],
                                config.FIRST_LEFT_RECTANGLE['bloated']['width'],
                                config.FIRST_LEFT_RECTANGLE['bloated']['height']))
    
    pygame.draw.rect(window, config.BLOATED_OBSTACLE_COLOR, 
                    pygame.Rect(config.SECOND_LEFT_RECTANGLE['bloated']['x'],
                                config.SECOND_LEFT_RECTANGLE['bloated']['y'],
                                config.SECOND_LEFT_RECTANGLE['bloated']['width'],
                                config.SECOND_LEFT_RECTANGLE['bloated']['height']))
    
    pygame.draw.polygon(window, config.BLOATED_OBSTACLE_COLOR,
                        list(config.HEXAGON['bloated_vertices']['vertices'].values()))
    
    for rect in config.RIGHT_RECTANGLES.values():
        pygame.draw.rect(window, config.BLOATED_OBSTACLE_COLOR,
                        pygame.Rect(rect['bloated']['x'], rect['bloated']['y'],
                                    rect['bloated']['width'], rect['bloated']['height']))
    
    # Draw Bloated Border
    for rect in config.BLOATED_BORDERS.values():
        pygame.draw.rect(window, config.BLOATED_OBSTACLE_COLOR,
                         pygame.Rect(rect['x'], rect['y'], rect['width'], rect['height']))

    # Draw actual obstacles
    pygame.draw.rect(window, config.OBSTACLE_COLOR, 
                    pygame.Rect(config.FIRST_LEFT_RECTANGLE['x'],
                                config.FIRST_LEFT_RECTANGLE['y'],
                                config.FIRST_LEFT_RECTANGLE['width'],
                                config.FIRST_LEFT_RECTANGLE['height']))
    
    pygame.draw.rect(window, config.OBSTACLE_COLOR, 
                    pygame.Rect(config.SECOND_LEFT_RECTANGLE['x'],
                                config.SECOND_LEFT_RECTANGLE['y'],
                                config.SECOND_LEFT_RECTANGLE['width'],
                                config.SECOND_LEFT_RECTANGLE['height']))
    
    pygame.draw.polygon(window, config.OBSTACLE_COLOR,
                        list(config.HEXAGON['vertices']['vertices'].values()))
    
    for rect in config.RIGHT_RECTANGLES.values():
        pygame.draw.rect(window, config.OBSTACLE_COLOR,
                        pygame.Rect(rect['x'], rect['y'], rect['width'], rect['height']))

    pygame.display.update()

def animate_explored_nodes(window, nodes, nodes_per_frame=10):
    """Animate the node exploration process
    Args:
        window(pygame surface)
        nodes (list)
        nodes_per_frame (int)
    """
    for i in range(0, len(nodes), nodes_per_frame):
        for node in nodes[i:i + nodes_per_frame]:
            pygame.draw.circle(window, config.NODES_COLOR,
                            (int(node[0]), int(node[1])), 1)
        pygame.display.update()
        pygame.time.delay(1)  # Delay after each batch, not each node


def animate_optimal_path(window, path):
    """Animate the optimal path
    Args:
        window (pygame surface)
        path (deque)
    """
    for node in path:
        pygame.draw.circle(window, config.PATH_COLOR, 
                        (int(node[0]), int(node[1])), 2)
        pygame.display.update()
        pygame.time.delay(4)
