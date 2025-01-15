from typing import Tuple, Dict
import math

def _calculate_hexagon_vertices(center: Tuple[float, float], side_length: float) -> Dict:
    """Calculate vertices for regular hexagon"""
    x, y = center
    # Calling A the top point on the left-vertical side and the subsequent
    # points going clock-wise around the hexagon.
    vertices = {
        'A': (x - 0.5*side_length*math.sqrt(3), y - side_length/2),
        'B': (x, y - side_length),
        'C': (x + 0.5*side_length*math.sqrt(3), y - side_length/2),
        'D': (x + 0.5*side_length*math.sqrt(3), y + side_length/2),
        'E': (x, y + side_length),
        'F': (x - 0.5*side_length*math.sqrt(3), y + side_length/2)
    }
    
    # Calculate edge equations (y = mx + b)
    edges = {
        'AB': _calculate_line_equation(vertices['A'], vertices['B']),
        'BC': _calculate_line_equation(vertices['B'], vertices['C']),
        'ED': _calculate_line_equation(vertices['E'], vertices['D']),
        'FE': _calculate_line_equation(vertices['F'], vertices['E']),
        'AF': vertices['A'][0],  # vertical line x = constant
        'CD': vertices['C'][0]   # vertical line x = constant
    }
    
    return {'vertices': vertices, 'edges': edges}

def _calculate_line_equation(p1: Tuple[float, float], 
                            p2: Tuple[float, float]) -> Tuple[float, float]:
    """Calculate slope and y-intercept for line equation y = mx + b"""
    if p2[0] - p1[0] == 0:
        return (float('inf'), p1[0])
    m = (p2[1] - p1[1]) / (p2[0] - p1[0])
    b = p1[1] - m * p1[0]
    return (m, b)

def _init_right_rectangles(window_width: int, clearance: int) -> Dict:
    """Initialize dimensions for right side rectangle configuration"""
    top_rect = {
        'x': window_width - 300,
        'y': 50,
        'width': 200,
        'height': 75,
        'bloated': {
            'x': window_width - 300 - clearance,
            'y': 50 - clearance,
            'width': 200 + 2*clearance,
            'height': 75 + 2*clearance
        }
    }
    
    vertical_rect = {
        'x': top_rect['x'] + 120,
        'y': top_rect['y'] + top_rect['height'],
        'width': window_width - top_rect['x'] - 100 - 120,
        'height': 400 - 2*top_rect['height'],
        'bloated': {
            'x': top_rect['x'] + 120 - clearance,
            'y': top_rect['y'] + top_rect['height'],
            'width': (window_width - top_rect['x'] - 100 - 120) + 2*clearance,
            'height': (400 - 2*top_rect['height']) + clearance
        }
    }
    
    bottom_rect = {
        'x': top_rect['x'],
        'y': top_rect['y'] + 75 + vertical_rect['height'],
        'width': top_rect['width'],
        'height': top_rect['height'],
        'bloated': {
            'x': top_rect['x'] - clearance,
            'y': top_rect['y'] + 75 + vertical_rect['height'] - clearance,
            'width': top_rect['width'] + 2*clearance,
            'height': top_rect['height'] + 2*clearance
        }
    }
    
    return {
        'top': top_rect,
        'vertical': vertical_rect,
        'bottom': bottom_rect
    }

"""Configuration for display settings"""
WINDOW_WIDTH: int = 1200
WINDOW_HEIGHT: int = 500
FPS: int = 60
CLEARANCE: int = 5

# Colors (RGB tuples)
BACKGROUND_COLOR: Tuple[int, int, int] = (0, 40, 255) # blue
OBSTACLE_COLOR: Tuple[int, int, int] = (255, 30, 70) # red
BLOATED_OBSTACLE_COLOR: Tuple[int, int, int] = (255, 255, 0) # yellow
NODES_COLOR: Tuple[int, int, int] = (0, 100, 0) # green
PATH_COLOR: Tuple[int, int, int] = (255, 255, 255) # white

"""Configuration for obstacle dimensions and positions"""
# First left rectangle
FIRST_LEFT_RECTANGLE = {
    'x': 100,
    'y': 0,
    'height': 400,
    'width': 75,
    'bloated': {
        'x': 100 - CLEARANCE,
        'y': 0,
        'height': 400 + CLEARANCE,
        'width': 75 + 2*CLEARANCE
    }
}
    
# Second left rectangle
SECOND_LEFT_RECTANGLE = {
    'x': FIRST_LEFT_RECTANGLE['x'] + FIRST_LEFT_RECTANGLE['width'] + 100,
    'y': WINDOW_HEIGHT - FIRST_LEFT_RECTANGLE['height'],
    'height': FIRST_LEFT_RECTANGLE['height'],
    'width': FIRST_LEFT_RECTANGLE['width'],
    'bloated': {
        'x': FIRST_LEFT_RECTANGLE['x'] + FIRST_LEFT_RECTANGLE['width'] + 100 - CLEARANCE,
        'y': WINDOW_HEIGHT - FIRST_LEFT_RECTANGLE['height'] - CLEARANCE,
        'height': FIRST_LEFT_RECTANGLE['height'] + CLEARANCE,
        'width': FIRST_LEFT_RECTANGLE['width'] + 2*CLEARANCE
    }
}
    
# Hexagon configuration
HEX_CENTER = (SECOND_LEFT_RECTANGLE['x'] + SECOND_LEFT_RECTANGLE['width'] + 300, 
                WINDOW_HEIGHT/2)
HEX_SIDE = 150
BLOATED_HEX_SIDE = HEX_SIDE + 2 * CLEARANCE
    
HEXAGON = {
    'center': HEX_CENTER,
    'side_length': HEX_SIDE,
    'bloated_side_length': BLOATED_HEX_SIDE,
    'vertices': _calculate_hexagon_vertices(HEX_CENTER, HEX_SIDE),
    'bloated_vertices': _calculate_hexagon_vertices(HEX_CENTER, BLOATED_HEX_SIDE)
    }

# Right side rectangles
RIGHT_RECTANGLES = _init_right_rectangles(WINDOW_WIDTH, CLEARANCE)

# Bloated Border Rectangles (only for visualization)
BLOATED_BORDERS = {
    'left_vertical': {
        'x': 0,
        'y': 0,
        'height': WINDOW_HEIGHT,
        'width': CLEARANCE
    },
    'right_vertical': {
        'x': WINDOW_WIDTH-CLEARANCE,
        'y': 0,
        'height': WINDOW_HEIGHT,
        'width': CLEARANCE
    },
    'top_horizontal': {
        'x': 0,
        'y': 0,
        'height': CLEARANCE,
        'width': WINDOW_WIDTH
    },
    'bottom_horizontal': {
        'x': 0,
        'y': WINDOW_HEIGHT-CLEARANCE,
        'height': CLEARANCE,
        'width': WINDOW_WIDTH
    } 
}  