import map
import sys 
import heapq
import math
from collections import deque
from HeapNode import HeapNode


# Generates the path from a map where the key a is node and the value 
# is the previous node
def getPath(prevMap, goal_cell):
    path = []
    p = goal_cell

    while p != None:
        path.append(p)
        p = prevMap[p]

    path.reverse()
    return path

# Helper function to check if a certain point is between 0 and the graphs dimensions
def checkPoint(x, y, dim):
    if x >= 0 and x < dim and y >= 0 and y < dim:
        return True
    return False

def aStar(graph, heuristicMethod):
    dim = len(graph)
    source_cell = (0,0)
    goal_cell = (dim-1, dim-1)
    
    # Dictionary where the key is the cell and the value is the 
    # cell that was previous. This is used to generate the actual path. 
    prevMap = {}

    # Create a visited array of same dimensions as the graph which will make sure
    # that A* considers only cells that have not been visited which will reduce 
    # the maximum fringe size and prevent cycles. Every position is initialized
    # to the maximum integer to make sure the correct path is found. 
    visited = [[sys.maxsize for p in range(dim)] for k in range(dim)]

    # Create a min-heap/priority queue to use for A*
    heap = []
    heapq.heapify(heap)

    # Start by appending the estimated distance from the source cell to the goal cell,
    # the source cell, the previous cell, and the distance from the source.
    # This heap will automatically use the first value in the tuple to sort the items
    # because of the __lt__ method in our HeapNode class.
    first_node = HeapNode(heuristicMethod(source_cell, goal_cell), source_cell, None, 0)
    
    prevMap[(0,0)] = None
    heapq.heappush(heap, first_node)
    visited[0][0] = heuristicMethod(source_cell, goal_cell)

    max_fringe = 0

    while len(heap) != 0:
        if(len(heap) > max_fringe):
            max_fringe = len(heap)
        node = heapq.heappop(heap)
        point = node.cell
        x = point[0]
        y = point[1]

        # If we have reached the goal cell, we can return the path associated with
        # that cell.
        if x == dim - 1 and y == dim - 1:
            count = 0
            for i in range(dim):
                for j in range(dim):
                    if(visited[i][j] != sys.maxsize):
                        count = count + 1
            return getPath(prevMap, goal_cell), count, max_fringe 
        else:
            # Generate a list of all possible neighboring points from the current point (x,y)
            points = [(x, y-1), (x,y+1), (x-1, y), (x+1, y)]
            for (i,j) in points:
                # The distance from the source to the current point (i,j)
                neighborToSource = node.distFromSource + 1

                # The estimated distance from the neighbor to the goal cell
                neighborPointHeuristic = heuristicMethod((i,j), goal_cell)

                totalDistanceToGoal = neighborToSource + neighborPointHeuristic

                # Only append points on the heap if the points are within the bounds
                # of the graph, the point is a 0, and the point has a smaller total distance
                # than visited[i][j].
                if checkPoint(i, j, dim) and graph[i][j] == 0 and visited[i][j] > totalDistanceToGoal:
                    visited[i][j] = totalDistanceToGoal
                    prevMap[(i,j)] = point
                    neighbor = HeapNode(totalDistanceToGoal, (i,j), point, neighborToSource)
                    heapq.heappush(heap, neighbor)
                    
    # If there is no path from source cell to goal cell than return the string below
    return "Failure: No Path", 0, 0

# Generates the euclidean distance between two points.
# Will be passed as the heuristicMethod for the aStar function
def euclideanH(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

# Generates the manhattan distance between two points.
# Will be passed as the heuristicMethod for the aStar function
def manhattanH(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)


#############################################################################################


# Generates a path from the source cell (0,0) to goal cell (dim - 1, dim - 1) using 
# Depth-First Search. 
def dfs(graph):
    max_fringe = 0
    # Create a stack to use for DFS
    stack = []
    dim = len(graph)
    goal_cell = (dim - 1, dim - 1)

    # Dictionary where the key is the cell and the value is the 
    # cell that was previous. This is used to generate the actual path. 
    prevMap = {}

    # Dictionary where the key is the cell and the value is the 
    # cell that was previous. This is used to generate the actual path. 
    prevMap = {}

    # Create a visited array of same dimensions as the graph to ensure that DFS
    # does not run forever.
    visited = [[False for p in range(dim)] for k in range(dim)]

    # Start by appending the source cell with the current path
    # The first element in the list is the x-value, second value is the 
    # y-value, and third value is a list of tuples which represents the
    # path DFS took to that cell.
    stack.append([0,0,(None)])
    visited[0][0] = True

    while len(stack) != 0:
        max_fringe = len(stack)
        point = stack.pop()
        x = point[0]
        y = point[1]
        prevMap[(x, y)] = point[2]
        
        # If we have reached the goal cell, we can return the path associated with
        # that cell. 
        if x == dim - 1 and y == dim - 1:
            return getPath(prevMap, goal_cell), max_fringe
        else:
            # Generate a list of all possible neighboring points from the current point (x,y)
            points = [(x, y-1), (x,y+1), (x-1, y), (x+1, y)]
            for (i,j) in points:
                # Only append points on the stack if the points are within the bounds
                # of the graph, the point is a 0, and the point has not been visited
                if checkPoint(i, j, dim) and graph[i][j] == 0 and visited[i][j] == False:
                    visited[i][j] = True
                    stack.append([i, j, (x, y)])

    # If there is no path from source cell to goal cell than return the string below
    return "Failure: No Path", 0

#####################################################################################    

#generate a hard maze based on path length using aStar euclidean
def generateHardMazePathLength():
    #counter for the len(path) over iterations
    max = 0
    original_path = None
    original_graph = None
    global_maze = None
    final_path = None
    #the termination condition to stop random restarting is that the path should be longer than 80
    while(max < 80):

        #generate a solvable map
        while True:
            g = map.generateMap(20, 0.3)
            if(dfs(g)[0] != "Failure: No Path"):
                break

        #using aStar euclidean to get the total number of nodes visited, max fringe size, and path size to determine difficulty
        #length of the path before making the map harder
        path, nodes_visited, max_fringe_size = aStar(g, euclideanH)
        max_difficulty = len(path)
        
        #print("First Maze Length: ")
        #print(max_difficulty)
        
        #generate the hard map by giving it the existing map, the current difficulty
        hard_graph = generateLocalHardMaze(g, max_difficulty)

        #used to get new path length and check results
        path2, nodes_visited2, max_fringe_size2 = aStar(hard_graph, euclideanH)
        max_difficulty2 = len(path2)
        #print("Second Maze Length:")
        #print(max_difficulty2)
        
        #used to save largest local max 
        if(max_difficulty2 > max):
            max = max_difficulty2
            global_maze = hard_graph
            final_path = path2
            original_path = path
            original_graph = g
           
    #display final result
    return final_path, global_maze, original_path, original_graph
    
#helper method to generate the local hard maze for a given graph 
# pass in the original graph and the original difficulty  
def generateLocalHardMaze(og_graph, difficulty):
    for i in range(len(og_graph)):
        for j in range(len(og_graph)):

            #save old bit value 
            old_value = og_graph[i][j]

            #flip bit in map
            if(old_value == 1):
                og_graph[i][j] = 0
            else:
                og_graph[i][j] = 1
            
            #runs aStar euclidean
            path, nodes_visited, max_fringe_size = aStar(og_graph, euclideanH)

            #if flipping the bit results in no possible path, do not flip the bit
            if(path == "Failure: No Path"):
                og_graph[i][j] = old_value
            
            #otherwise flip the bit only if the difficulty gets harder
            else:

                new_difficulty = len(path)

                #reset pointer back to the start node 
                if(new_difficulty > difficulty):
                    difficulty = new_difficulty
                    i = 0
                    j = 0
                else:
                    og_graph[i][j] = old_value
                    
    return og_graph
    
#generate a hard maze based on fringe size and DFS
def generateHardMazeFringeSize():
    #counter for the len(path) over iterations
    max = 0
    original_path = None
    original_graph = None
    global_maze = None
    final_path = None
    #the termination condition to stop random restarting is that the fringe size should be longer than 80
    while(max < 80):

        #generate a solvable map
        while True:
            g = map.generateMap(20, 0.3)
            if(dfs(g)[0] != "Failure: No Path"):
                break
        
        #find original fringe size
        path, fringe_size = dfs(g)
        #print(fringe_size)

        #generate hard map based on fringe size
        new_hard_maze = generateLocalHardMazeDFSFringe(g, fringe_size)
        path2, fringe_size2 = dfs(new_hard_maze)
        max_difficulty2 = fringe_size2
        #print(fringe_size2)

        #used to save largest local max 
        if(max_difficulty2 > max):
            max = max_difficulty2
            global_maze = new_hard_maze
            final_path = path2
            original_path = path
            original_graph = g

    #print out max fringe size 
    return final_path, global_maze, original_path, original_graph


#helper method to generate the local hard graph based on a given graph
#input is a graph and the original difficulty
def generateLocalHardMazeDFSFringe(og_graph, difficulty):
    for i in range(len(og_graph)):
        for j in range(len(og_graph)):

            #save old value
            old_value = og_graph[i][j]

            #flip value
            if(old_value == 1):
                og_graph[i][j] = 0
            else:
                og_graph[i][j] = 1

            path, max_fringe_size = dfs(og_graph)

            #if flipping value results in an unsolvable graph do not flip value
            if(path == "Failure: No Path"):
                og_graph[i][j] = old_value
            else:

                #otherwise, if flipping value results in harder maze to solve, flip 
                new_difficulty = max_fringe_size

                #reset pointer to start point if value is flipped
                if(new_difficulty > difficulty):
                    i = 0
                    j = 0
                    difficulty = new_difficulty
                else:
                    og_graph[i][j] = old_value
                    
    return og_graph

#generate a hard maze based on number of nodes and aStar manhattan
def generateHardMazeNumberOfNodes():
    #counter for the len(path) over iterations
    max = 0
    original_path = None
    original_graph = None
    global_maze = None
    final_path = None
    #the termination condition to stop random restarting is that the nodes visited should be longer than 280 
    while(max < 280):

        #generate a solvable map
        while True:
            g = map.generateMap(20, 0.3)
            if(dfs(g)[0] != "Failure: No Path"):
                break

        #find initial difficulty of the graph
        path, nodes_visited, max_fringe_size = aStar(g, manhattanH)
        max_difficulty = nodes_visited
        #print("First: ")
        #print(max_difficulty)

        #generate the hard graph based on the graph before
        hard_graph = generateLocalHardMazeManhattan(g, max_difficulty)
        path2, nodes_visited2, max_fringe_size2 = aStar(hard_graph, manhattanH)
        max_difficulty2 = nodes_visited2
        #print("Second:")
        #print(max_difficulty2)

        #used to save largest local max 
        if(max_difficulty2 > max):
            max = max_difficulty2
            global_maze = hard_graph
            final_path = path2
            original_path = path
            original_graph = g

    #print out max fringe size 
    return final_path, global_maze, original_path, original_graph 

#helper method that generates a hard maze based on the graph given 
#inputs are a graph and the original difficulty
def generateLocalHardMazeManhattan(og_graph, difficulty):
    for i in range(len(og_graph)):
        for j in range(len(og_graph)):
            old_value = og_graph[i][j]

            #flip value
            if(old_value == 1):
                og_graph[i][j] = 0
            else:
                og_graph[i][j] = 1

            path, nodes_visited, max_fringe_size = aStar(og_graph, manhattanH)

            #if flipping value results in an unsolvable graph do not flip
            if(path == "Failure: No Path"):
                og_graph[i][j] = old_value
            else:
                new_difficulty = nodes_visited

                #if flipping the value results in a harder map then flip and reset pointer to start
                if(new_difficulty > difficulty):
                    difficulty = new_difficulty
                    i = 0
                    j = 0
                else:
                    og_graph[i][j] = old_value
                    
    return og_graph