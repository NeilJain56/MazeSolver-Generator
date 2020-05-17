import random
import matplotlib as mpl
from matplotlib import pyplot
import numpy as np
import search
import fire
from copy import deepcopy


# Generates a map given a dimension and a probability that a cell is blocked
# 1 --> Cell is Blocked, 0 --> Cell is Open
def generateMap(dim, p):
    graph = []

    # These two loops are used initialize the graph with the correct values
    for i in range(dim):
        row = []
        for j in range(dim):
            # Random.choices function is an inbuilt python function that takes 
            # a list of choices and respective weights that returns a choice.
            # In this case, our two choices are 1 for a cell being blocked and 
            # 0 for a cell being clear/open. 
            randomNum = random.choices(population=[1, 0], weights=[p, 1 - p], k=1)[0]
            row.append(randomNum)
        graph.append(row)

    # Ensures that the source cell (0,0) is open and the goal cell (dim - 1, dim - 1)
    # is open, as they may have been initialized to 1 in the previous for loops
    graph[0][0] = 0
    graph[dim-1][dim-1] = 0
    return graph

# Generates a map given a dimension and a probability that a cell is blocked
# 1 --> Cell is Blocked, 0 --> Cell is Open , 2 --> Cell is on Fire
# Additionally, adds a random starting point for the fire that has a path 
# from source to fire cell.
def generateFireMap(dim, p):
    goal_cell = (dim-1, dim-1)
    source_cell = (0,0)

    # Generate a map with a dim and p for walls 
    graph = generateMap(dim, p)

    # If there is no path from the source cell to goal cell
    # discard this map and generate a new one. 
    while search.dfsForFireMap(graph, goal_cell) == False:
        graph = generateMap(dim, p)


    # Generate a random coordinate between 0 and dim - 1
    fireX = random.randint(0, dim-1)
    fireY = random.randint(0, dim-1)
    fire_cell = (fireX, fireY)

    # Generate new fire cell starting points if fire starting cell:
    #   - is source cell
    #   - is goal cell
    #   - is a wall (1 in the graph means a wall)
    #   - has no path to source cell
    while fire_cell == source_cell or fire_cell == goal_cell or graph[fireX][fireY] == 1 or search.dfsForFireMap(graph, fire_cell) == False:
        fireX = random.randint(0, dim-1)
        fireY = random.randint(0, dim-1)
        fire_cell = (fireX, fireY)

    graph[fireX][fireY] = 2
    return graph

# Helper Function used to print graphs in a pretty way
def printMap(graph):
    for i in range(len(graph)):
        for j in range(len(graph)):
            print(graph[i][j], end=" ")
        print("")

# Helper function that takes in a list of points as a path and a graph.
# Prints out the graph with the points taken shown as "-"
def printPath(path, graph):
    if type(path) == str:
        print("No Valid Path")
        return
    p = set(path)
    for i in range(len(graph)):
        for j in range(len(graph)):
            if (i,j) in p:
                print("-", end=" ")
            else:
                print(graph[i][j], end=" ")
        print("")

# Helper function that takes in a list of points as a path and a graph.
# Shows graph in image form where: White --> Open Cell, Black --> Wall, Red --> Path Taken
def visualize(path, graph):
    pyplot.rcParams["image.composite_image"] = False
    if type(path) == str:
        print("No Valid Path")
        return

    copyOfGraph = deepcopy(graph)
    p = set(path)

    for i in range(len(copyOfGraph)):
        for j in range(len(copyOfGraph)):
            if (i,j) in p:
                copyOfGraph[i][j] = 2

    cmap = mpl.colors.ListedColormap(['white','black', 'red'])
    pyplot.figure()
    img = pyplot.imshow(copyOfGraph, cmap = cmap)
    pyplot.show()

# Helper function that takes in a list of points as a path and a graph.
# Shows graph in image form where: White --> Open Cell, Black --> Wall,
# Red --> Path Taken, Orange --> Fire in Final State
def visualizeFireMap(path, graph):
    pyplot.rcParams["image.composite_image"] = False
    if type(path) == str:
        print("No Valid Path")
        return
        
    copyOfGraph = deepcopy(graph)
    p = set(path)

    for i in range(len(copyOfGraph)):
        for j in range(len(copyOfGraph)):
            if (i,j) in p:
                copyOfGraph[i][j] = 3

    cmap = mpl.colors.ListedColormap(['white','black', 'orange', 'red'])
    pyplot.figure()
    img = pyplot.imshow(copyOfGraph, cmap = cmap)
    pyplot.show()
