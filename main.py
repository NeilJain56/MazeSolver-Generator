import map
from search import euclideanH, manhattanH, aStar, bfs, bidirectionalBfs, dfs
import fire
import GenerateHardMaze

def main():
    # Example graph generation with dimension and probability
    g = map.generateMap(150, 0.2)
    manhattanPath = aStar(g, manhattanH)
    euclideanPath = aStar(g, euclideanH)
    bfsPath = bfs(g)
    dfsPath = dfs(g)
    bidirectionalBFSPath = bidirectionalBfs(g) 

    # Example for visualizing path for graph g
    map.visualize(manhattanPath, g)
    map.visualize(euclideanPath, g)
    map.visualize(bfsPath, g)
    map.visualize(dfsPath, g)
    map.visualize(bidirectionalBFSPath,g)

    # Part 3 calls with visualization
    path, graph, og_path, og_graph = GenerateHardMaze.generateHardMazePathLength()
    map.visualize(og_path, og_graph)
    map.visualize(path, graph)

    path2, graph2, og_path2, og_graph2 = GenerateHardMaze.generateHardMazeFringeSize()
    map.visualize(og_path2, og_graph2)
    map.visualize(path2, graph2)

    path3, graph3, og_path3, og_graph3 = GenerateHardMaze.generateHardMazeNumberOfNodes()
    map.visualize(og_path3, og_graph3)
    map.visualize(path3, graph3)

    # Example fire graph generation with dimension and probability
    fireG = map.generateFireMap(100, 0.3)

    # Strategies with Flammability rate q
    q = 0.1
    result1 = fire.strategy1(fireG, q)
    result2 = fire.strategy2(fireG, q)

    map.printPath(result1[0], result1[1])
    print("")
    map.printPath(result2[0], result2[1])

    # Example for visualizing path for fire graph g where the first element 
    # in the result tuple is the path and second element is the final graph. 
    map.visualizeFireMap(result1[0], result1[1])
    map.visualizeFireMap(result2[0], result2[1])


if __name__ == "__main__":
    main()
