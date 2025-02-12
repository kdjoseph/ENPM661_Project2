# ENPM661_Project2 Dijkistra Algorithm Simulation

This code uses the Dijkstra algorithm to find the optimal path on an obstacle layout from a starting point to a goal point chosen by the user. Once the path is found, an animation showing the node exploration part of the search and the optimal path is displayed.

## Modules

The code consists of five modules:

-   `dijkstra_main.py`: Prompts the user for input values, then starts the Dijkstra search and the animation.
-   `config.py`: Contains key constants, such as the obstacle layout size and obstacle dimensions, the colors to use during the animation
-   `obstacles.py`: Contains a functions to check if a point is within the obstacle space.
-   `pathfinding.py`: Contains the Dijkstra algorithm implementation.
-   `visualization.py`: Contains the Pygame functions to display and create the animation of the search.

## Usage

To run the code:

1.  Ensure that the dependencies are installed.
2.  Download all modules into a single folder.
3.  Open the `dijkstra_main.py` module and run it.
4.  Input prompts will appear, asking for the starting and goal points
5.  The search for the optimal path will begin.
6.  Once an optimal path is found, a Pygame window will appear, showing the animation.

## Dependencies

### Software Packages

-   Python 3.8 or later

### Libraries

-   pygame
