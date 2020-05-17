import sys 
import heapq
import math
from collections import deque
from HeapNode import HeapNode

# Generates a path from the source cell (0,0) to goal cell (dim - 1, dim - 1) using 
# Depth-First Search. 
def dfs(graph):
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
        point = stack.pop()
        x = point[0]
        y = point[1]
        prevMap[(x, y)] = point[2]
        
        # If we have reached the goal cell, we can return the path associated with
        # that cell. 
        if x == dim - 1 and y == dim - 1:
            return getPath(prevMap, goal_cell)
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
    return "Failure: No Path"

# Returns true when there is a path from the source cell (0,0) 
# to any goal cell using Depth-First Search. 
# Used by generateFireMap to see if there is a path from source 
# to fire starting cell.
def dfsForFireMap(graph, goal):
    # Create a stack to use for DFS
    stack = []
    dim = len(graph)

    # Create a visited array of same dimensions as the graph to ensure that DFS
    # does not run forever.
    visited = [[False for p in range(dim)] for k in range(dim)]

    # The first element in the list is the x-value, second value is the 
    # y-value.
    visited[0][0] = True
    stack.append([0,0])

    while len(stack) != 0:
        point = stack.pop()
        x = point[0]
        y = point[1]
        
        # If we have reached the goal cell, we can return True.
        if x == goal[0] and y == goal[1]:
            return True
        else:
            # Generate a list of all possible neighboring points from the current point (x,y)
            points = [(x, y-1), (x,y+1), (x-1, y), (x+1, y)]
            for (i,j) in points:
                # Only append points on the stack if the points are within the bounds
                # of the graph, the point is a 0, and the point has not been visited
                if checkPoint(i, j, dim) and graph[i][j] == 0 and visited[i][j] == False:
                    visited[i][j] = True
                    stack.append([i, j])

    # If there is no path from source cell to goal cell than return false
    return False

# Generates a path from the source cell (0,0) to goal cell (dim - 1, dim - 1) using 
# Breadth-First Search. 
def bfs(graph):
    dim = len(graph)
    queue = deque()

    # Create a visited array of same dimensions as the graph to ensure that BFS
    # will minimize the fringe size
    visited = [[False for p in range(dim)] for k in range(dim)]

    # Enqueue the starting position and mark it as visited
    queue.append([0,0,[(0,0)]])
    visited[0][0] = True

    # Keep looping while there are elements in the queue
    while len(queue) != 0:
        # Get the first item on the queue
        point = queue.popleft()
        x = point[0]
        y = point[1]
        path = point[2].copy()

        # Generate a list of all possible neighboring points from the current point (x,y)
        neighbors = [(x, y-1), (x,y+1), (x-1, y), (x+1, y)]
        for neighbor in neighbors:
            path = point[2].copy()
            i = neighbor[0]
            j = neighbor[1]

            # If we have reached the goal cell, we can return the path associated with
            # that cell.
            if (i == dim - 1) and (j ==  dim - 1):
                path.append(neighbor)
                return path

            # Only append points on the stack if the points are within the bounds
            # of the graph, the point is a 0, and the point has not been visited
            if checkPoint(i, j, dim) and graph[i][j] == 0 and visited[i][j] == False:
                path.append(neighbor)
                queue.append([i, j, path])
                visited[i][j] = True
    
    # If there is no path from source cell to goal cell than return the string below
    return "Failure: No Path"

# Generates a path from the source cell (0,0) to goal cell (dim - 1, dim - 1) using 
# bidirectional Breadth-First Search. 
def bidirectionalBfs(graph):
    dim = len(graph)
    # Queue from the starting point
    sQueue = deque()
    # Queue from the finishing point
    fQueue = deque()

    # Create a visited array of the same dimensions as the graph to find the 
    # intersection of the two BFSs where:
    # 0  --> Not visited by both BFSs
    # 1  --> Visited by starting point queue
    # -1 --> Visited by finishing point queue 
    visited = [[0 for p in range(dim)] for k in range(dim)]

    # Enqueue the starting position and mark it as visited
    sQueue.append([0,0,[(0,0)]])
    visited[0][0] = 1

    # Enqueue the finishing postion and mark it as visted
    fQueue.append([dim-1,dim-1,[(dim-1,dim-1)]])
    visited[dim-1][dim-1] = -1

    # Keep looping while there are elements in the queue
    while len(sQueue) != 0 and len(fQueue) != 0:
        # Get the first item on the starting queue
        sPoint = sQueue.popleft()
        sX = sPoint[0]
        sY = sPoint[1]

        # Generate a list of all possible neighboring points from the current point (x,y)
        neighbors = [(sX, sY-1), (sX,sY+1), (sX-1, sY), (sX+1, sY)]
        for neighbor in neighbors:
            path = sPoint[2].copy()
            i = neighbor[0]
            j = neighbor[1]

            # If we have reached a cell that has been visited by the finishing point 
            # queue (-1), than we have found an intersection and we can return the path
            # associated with that cell.
            if checkPoint(i, j, dim) and visited[i][j] == -1:
                while len(fQueue) != 0:
                    currPoint = fQueue.popleft()
                    currFX = currPoint[0]
                    currFY = currPoint[1]
                    if(i == currFX and j == currFY):
                        return path + currPoint[2][::-1]

            # Only append points on the stack if the points are within the bounds
            # of the graph, the point is a 0, and the point has not been visited
            if checkPoint(i, j, dim) and graph[i][j] == 0 and visited[i][j] == 0:
                path.append(neighbor)
                sQueue.append([i, j, path])
                visited[i][j] = 1
                
        # Get the first item on the finishing queue
        fPoint = fQueue.popleft()
        fX = fPoint[0]
        fY = fPoint[1]

        # Generate a list of all possible neighboring points from the current point (x,y)
        neighbors = [(fX, fY-1), (fX,fY+1), (fX-1, fY), (fX+1, fY)]
        for neighbor in neighbors:
            path = fPoint[2].copy()
            i = neighbor[0]
            j = neighbor[1]

            # If we have reached a cell that has been visited by the starting point 
            # queue (1), than we have found an intersection and we can return the path
            # associated with that cell.
            if checkPoint(i, j, dim) and visited[i][j] == 1:
                while len(sQueue) != 0:
                    currPoint = sQueue.popleft()
                    currSX = currPoint[0]
                    currSY = currPoint[1]
                    if(i == currSX and j == currSY):
                        return path + currPoint[2][::-1]

            # Only append points on the stack if the points are within the bounds
            # of the graph, the point is a 0, and the point has not been visited
            if checkPoint(i, j, dim) and graph[i][j] == 0 and visited[i][j] == 0:
                path.append(neighbor)
                fQueue.append([i, j, path])
                visited[i][j] = -1

    # If there is no path from source cell to goal cell than return the string below
    return "Failure: No Path"
    
# Generates a path from the source cell (0,0) to goal cell (dim - 1, dim - 1) using
# A* where heuristicMethod can be euclideanH or manhattanH
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

    while len(heap) != 0:
        node = heapq.heappop(heap)
        point = node.cell
        x = point[0]
        y = point[1]

        # If we have reached the goal cell, we can return the path associated with
        # that cell.
        if x == dim - 1 and y == dim - 1:
            return getPath(prevMap, goal_cell)
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
    return "Failure: No Path"

# Generates a path from any source cell to goal cell (dim - 1, dim - 1) using
# A* where heuristicMethod can be euclideanH or manhattanH
def aStarForStrategy2(source_cell, graph, heuristicMethod):
    dim = len(graph)
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
    
    prevMap[source_cell] = None
    heapq.heappush(heap, first_node)
    visited[source_cell[0]][source_cell[1]] = heuristicMethod(source_cell, goal_cell)

    while len(heap) != 0:
        node = heapq.heappop(heap)
        point = node.cell
        x = point[0]
        y = point[1]

        # If we have reached the goal cell, we can return the path associated with
        # that cell.
        if x == dim - 1 and y == dim - 1:
            return getPath(prevMap, goal_cell)
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
    return "Failure: No Path"

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

