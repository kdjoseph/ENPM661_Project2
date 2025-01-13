from typing import Dict, Tuple, List
import config

"""Manages the obstacle space and collision detection"""
        
def is_collision(x: float, y: float) -> bool:
    """Check if point (x,y) collides with any obstacle
    Args:
        x (float): x coordinate of the point.
        y (float): y coordinate of a point.
    
    Returns:
        True: if point is in obstacle
        False: if point is not in obstacle
    """
    return (_check_hexagon_collision(x, y) or 
            _check_left_rectangles_collision(x, y) or
            _check_right_rectangles_collision(x, y) or
            _check_border_collision(x, y))

def _check_hexagon_collision(x: float, y: float) -> bool:
    """Check collision with hexagonal obstacle"""
    hex_data = config.HEXAGON['bloated_vertices']
    edges = hex_data['edges']
    
    # Check if point is within vertical bounds
    if x < edges['AF'] or x > edges['CD']:
        return False
        
    # Check if point is within angled edges
    ab_m, ab_b = edges['AB']
    bc_m, bc_b = edges['BC']
    ed_m, ed_b = edges['ED']
    fe_m, fe_b = edges['FE']
    
    if not (ab_m*x + ab_b <= y <= ed_m*x + ed_b):
        return False
        
    if not (bc_m*x + bc_b <= y <= fe_m*x + fe_b):
        return False
        
    return True

def _check_left_rectangles_collision(x: float, y: float) -> bool:
    """Check collision with left side rectangles"""
    first_rect = config.FIRST_LEFT_RECTANGLE['bloated']
    second_rect = config.SECOND_LEFT_RECTANGLE['bloated']
    
    # First rectangle
    if (first_rect['x'] <= x <= first_rect['x'] + first_rect['width'] and
        y <= first_rect['height']):
        return True
        
    # Second rectangle
    if (second_rect['x'] <= x <= second_rect['x'] + second_rect['width'] and
        y >= second_rect['y']):
        return True
        
    return False

def _check_right_rectangles_collision(x: float, y: float) -> bool:
    """Check collision with right side rectangles"""
    right_rects = config.RIGHT_RECTANGLES
    
    for rect in ('top', 'vertical', 'bottom'):
        bloated = right_rects[rect]['bloated']
        if (bloated['x'] <= x <= bloated['x'] + bloated['width'] and
            bloated['y'] <= y <= bloated['y'] + bloated['height']):
            return True
            
    return False

def _check_border_collision(x: float, y: float) -> bool:
    """Check if point is too close to borders"""
    return (x <= config.CLEARANCE or 
            x >= (config.WINDOW_WIDTH - config.CLEARANCE) or
            y <= config.CLEARANCE or 
            y >= (config.WINDOW_HEIGHT - config.CLEARANCE))