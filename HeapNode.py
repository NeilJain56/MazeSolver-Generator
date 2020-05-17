class HeapNode:
    def __init__(self, heuristicDist, cell, prev, distFromSource):
            self.heuristicDist = heuristicDist
            self.cell = cell
            self.prev = prev
            self.distFromSource = distFromSource

    def __lt__(self, other):
        return self.heuristicDist < other.heuristicDist
