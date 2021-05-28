import heapq
import math


def manhattan_distance(a, b):
    return abs(a.position[0] - b.position[0]) + abs(a.position[1] - b.position[1])

def euclidian_distance(a, b):
    return math.sqrt((a.position[0] - b.position[0])**2 + (a.position[1] - b.position[1])**2)


"""
Nom mal choisi !
Trop proche de ROS.
"""
class Node:
    """
    //A node class for A* Pathfinding
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position
    
    def __repr__(self):
        return f"{self.position} - g: {self.g} h: {self.h} f: {self.f}"

    # defining less than for purposes of heap queue
    def __lt__(self, other):
        return self.f < other.f
    
    # defining greater than for purposes of heap queue
    def __gt__(self, other):
        return self.f > other.f

"""
Code mort ?
"""

"""
Init grid.
0 = empty, 1 = starting point, 2 = ending point (arrival)
"""
#maze = [[0, 0, 2], [0, 0, 0], [0, 0, 1]]
maze = [[0, 1, 0, 0, 0],
        [0, 0, 0, -1, -1],
        [0, 0, 0, -1, 2],
        [0, 0, 0, 0, 0]]
#for i in maze :
    #print(i)


def return_path(current_node):
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]  # Return reversed path

"""
    print : Utilisation du système de log
    https://github.com/ros2/examples/blob/master/rclpy/services/minimal_client/examples_rclpy_minimal_client/client.py
    
    Faut avoir le noeud stocké quelques part (via node de la classe world ?)

    On a besoin de définir le noeud fin ?

    En lisant l'algorithme je le trouve verbeux. 
    Je pense qu'on n'a pas besoin de définir les adjacent squares
"""
def a_star(maze):
    for i in range(len(maze)):
        for j in range(len(maze[i])):
            if maze[i][j]==1:
                print(f"({i},{j}) est le noeud de départ")
                start_node = Node(None, (i,j))
                start_node.g = start_node.h = start_node.f = 0
            if maze[i][j]==2:
                print(f"({i},{j}) est le noeud d'arrivée")
                end_node = Node(None, (i,j))
                end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Heapify the open_list and Add the start node
    heapq.heapify(open_list) #on transforme la liste open en queue
    heapq.heappush(open_list, start_node)

    # what squares do we search
    adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1),) #liste des coordonnées des voisins

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = heapq.heappop(open_list)
        closed_list.append(current_node)

        # Found the goal ?
        if current_node == end_node:
            return return_path(current_node)

         # Generate children
        children = []
        
        for new_position in adjacent_squares: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range (check if within maze boundary)
            if (node_position[0] > (len(maze) - 1) or 
                node_position[0] < 0 or 
                node_position[1] > (len(maze[i]) -1) or 
                node_position[1] < 0) :
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] == -1:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        
        # Loop through children
        for child in children:
            # Child is on the closed list
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            "Distance type"
            child.h = euclidian_distance(end_node, child)
            child.f = child.g + child.h

            # Child is already in the open list
            if len([open_node for open_node in open_list if child.position == open_node.position and child.g > open_node.g]) > 0:
                continue

            # Add the child to the open list
            heapq.heappush(open_list, child)

    # Use logging system
    print("Couldn't get a path to destination")
    return None

# Non !
#path = a_star(maze)

# Non !
#print(path)
