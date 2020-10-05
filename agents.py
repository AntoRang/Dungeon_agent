# Sample code from https://www.redblobgames.com/pathfinding/a-star/
# Copyright 2014 Red Blob Games <redblobgames@gmail.com>
#
# Feel free to use this code in your own projects, including commercial projects
# License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>

from typing import Dict, List, Iterator, Tuple, TypeVar, Optional
from typing_extensions import Protocol
import time

T = TypeVar('T')
Location = TypeVar('Location')
GridLocation = Tuple[int, int]

class Graph(Protocol):
    def neighbors(self, id: Location) -> List[Location]: pass


class SimpleGraph:
    def __init__(self):
        self.edges: Dict[Location, List[Location]] = {}

    def neighbors(self, id: Location) -> List[Location]:
        return self.edges[id]

# Class Queue

class Queue:
    def __init__(self):
        self.elements = collections.deque()

    def empty(self) -> bool:
        return len(self.elements) == 0

    def put(self, x: T):
        self.elements.append(x)

    def get(self) -> T:
        return self.elements.popleft()

# Class Square Grid

class SquareGrid:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.walls: List[GridLocation] = []

    def in_bounds(self, id: GridLocation) -> bool:
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, id: GridLocation) -> bool:
        return id not in self.walls

    def neighbors(self, id: GridLocation) -> Iterator[GridLocation]:
        (x, y) = id
        neighbors = [(x + 1, y), (x, y - 1), (x - 1, y), (x, y + 1)]
        if (x + y) % 2 == 0: neighbors.reverse()  # aesthetics
        results = filter(self.in_bounds, neighbors)
        results = filter(self.passable, results)
        return results

# Class Weighted Graph

class WeightedGraph(Graph):
    def cost(self, from_id: Location, to_id: Location) -> float: pass


# Class Grid With Weights

class GridWithWeights(SquareGrid):
    def __init__(self, width: int, height: int):
        super().__init__(width, height)
        self.weights: Dict[GridLocation, float] = {}

    def cost(self, from_node: GridLocation, to_node: GridLocation) -> float:
        return self.weights.get(to_node, 1)

# Class Priority Queue

class PriorityQueue:
    def __init__(self):
        self.elements: Array[T] = []

    def empty(self) -> bool:
        return len(self.elements) == 0

    def put(self, item: T, priority: float):
        heapq.heappush(self.elements, (priority, item))

    def get(self) -> T:
        return heapq.heappop(self.elements)[1]

# Class Square Grid Neighbor Order

class SquareGridNeighborOrder(SquareGrid):
    NEIGHBOR_ORDER = [(+1, 0), (0, -1), (-1, 0), (0, +1)]
    def neighbors(self, id):
        (x, y) = id
        neighbors = [(x + dx, y + dy) for (dx, dy) in self.NEIGHBOR_ORDER]
        results = filter(self.in_bounds, neighbors)
        results = filter(self.passable, results)
        return list(results)


# utility functions for dealing with square grids
def from_id_width(id, width):
    return (id % width, id // width)


# Function Draw tile

def draw_tile(graph, id, style):
    r = "."
    coor = id
    if 'number' in style and id in style['number']: r = " %-2d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = ">"
        if x2 == x1 - 1: r = "<"
        if y2 == y1 + 1: r = "v"
        if y2 == y1 - 1: r = "^"
    if 'start' in style and id == style['start']: r, coor = "A", id
    if 'goal' in style and id == style['goal']:   r, coor = "Z", id
    if 'path' in style and id in style['path']:   r, coor = "@", id
    if id in graph.walls: r, coor = "#", id
    return (r, coor)


# Function Draw grid

def draw_grid(graph, **style):
    str = ""
    list_coor = []
    for y in range(graph.height):
        for x in range(graph.width):
            (tile, coor) = draw_tile(graph, (x, y), style)
            str += "%s" % tile
            if tile == "@":
                list_coor.append(coor)
        str+= "\n"
    return str, list_coor

# Function Dijkstra search

def dijkstra_search(graph: WeightedGraph, start: Location, goal: Location):
    time_start = time.time()
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from: Dict[Location, Optional[Location]] = {}
    cost_so_far: Dict[Location, float] = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current: Location = frontier.get()

        if current == goal:
            break

        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put(next, priority)
                came_from[next] = current
    end = time.time()
    print('Dijkstra search Time elapsed')
    print(end - time_start)
    return came_from, cost_so_far



# thanks to @m1sp <Jaiden Mispy> for this simpler version of
# reconstruct_path that doesn't have duplicate entries
def reconstruct_path(came_from: Dict[Location, Location],
                     start: Location, goal: Location) -> List[Location]:
    current: Location = goal
    path: List[Location] = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional
    return path


# Function Heuristic

def heuristic(a: GridLocation, b: GridLocation) -> float:
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)


# Function A Star

def a_star_search(graph: WeightedGraph, start: Location, goal: Location):
    time_start = time.time()
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from: Dict[Location, Optional[Location]] = {}
    cost_so_far: Dict[Location, float] = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current: Location = frontier.get()

        if current == goal:
            break

        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
    end = time.time()
    print('A Star  Time elapsed')
    print(end - time_start)
    return came_from, cost_so_far



# Function BFS

def breadth_first_search(graph: Graph, start: Location, goal: Location):
    frontier = Queue()
    frontier.put(start)
    came_from: Dict[Location, Location] = {}
    came_from[start] = None

    while not frontier.empty():
        current: Location = frontier.get()

        if current == goal:
            break

        for next in graph.neighbors(current):
            if next not in came_from:
                frontier.put(next)
                came_from[next] = current

    return came_from


import heapq

# Function Convert Path to the solution path
def convert_path(came_from, coordinates):
    coordinates.reverse()
    path_list = []
    for i in came_from:
        for j in coordinates:
            if i == j:
                path_list.append(i)
    return path_list

def read_map(mapfile):
    with open(mapfile, 'r') as f:
        world_map = f.readlines()
    world_map = [line.strip() for line in world_map]
    return (world_map)

# diagram4.weights = {loc: 5 for loc in [(3, 4), (3, 5), (4, 1), (4, 2),
#                                        (4, 3), (4, 4), (4, 5), (4, 6),
#                                        (4, 7), (4, 8), (5, 1), (5, 2),
#                                        (5, 3), (5, 4), (5, 5), (5, 6),
#                                        (5, 7), (5, 8), (6, 2), (6, 3),
#                                        (6, 4), (6, 5), (6, 6), (6, 7),
#                                        (7, 3), (7, 4), (7, 5)]}

# start, goal = (1, 9), (9, 6)
# came_from, cost_so_far = a_star_search(diagram4, start, goal)
# print("Path:")
# (diag, list_coor) = draw_grid(diagram4, path=reconstruct_path(came_from, start=start, goal=goal))
#
# print(diag)
# path = convert_path(came_from, list_coor)
# print(path)
#
# print("Cost:")
# (diag, cost) = draw_grid(diagram4, number=cost_so_far, start=start, goal=goal)
# print(diag)

# map = 'map1.txt'
# map = get_map(map)
# x, y = 0, 0
# diagrama = GridWithWeights(10,10)
# diagrama.walls = []
# for line in map:
#     x = 0
#     for idx in line:
#         if idx == "G":
#             goal = (x, y)
#         if idx == "S":
#             start = (x, y)
#         if idx == "#":
#             diagrama.walls.append((x, y))
#         x += 1
#     y+=1

# diagram4 = GridWithWeights(10, 10)
# diagram4.walls = [(1, 7), (1, 8), (2, 7), (2, 8), (3, 7), (3, 8), (3, 6), (3, 5), (3, 4), (3, 3), (3, 2), (3, 0),
#                   (3, 9), (9,8), (8,8), (7,8), (5,6), (6,6), (7,6), (8,6), (6,8), (4,4), (5,4), (6,4), (7,4), (7,3), (7,2)]
# start, goal = (1, 4), (9, 9)



# A star
# came_from_A, cost_so_far_A = a_star_search(diagrama, start, goal)
# # Dijkstra search
# came_from, cost_so_far = dijkstra_search(diagrama, start, goal)
#
# # Dijkstra search
# print('Dijkstra search')
# (diag, list_coor) = draw_grid(diagrama, path=reconstruct_path(came_from, start=start, goal=goal))
# print(diag)
#
# # A star seach
# print('A star search')
# (diag, list_coor) = draw_grid(diagrama, path=reconstruct_path(came_from_A, start=start, goal=goal))
# print(diag)
