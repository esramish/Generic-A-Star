# Usage: pytest -rP

import os, sys

from typing import List

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

from a_star import find_path, AStarableI

class Point2D(AStarableI):

    def __init__(self, x, y, grid_width, grid_height, obstacles):
        self.x = x
        self.y = y
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.obstacles = obstacles

    def __eq__(self, __o: object) -> bool:
        return self.x == __o.x and self.y == __o.y
    
    def __hash__(self):
        # return self.x + self.grid_width * self.y
        return hash(str(self))
    
    def __repr__(self):
        return f"({self.x}, {self.y})"
    
    def is_valid(self) -> bool:
        return 0 <= self.x < self.grid_width and \
               0 <= self.y < self.grid_height and \
               self not in self.obstacles

    def get_neighbors(self) -> List:
        '''Get all the neighbors of this location/node/payload. They don't have to be valid; the A* algorithm will query validity later.'''
        return [
            Point2D(self.x - 1, self.y, self.grid_width, self.grid_height, self.obstacles),
            Point2D(self.x + 1, self.y, self.grid_width, self.grid_height, self.obstacles),
            Point2D(self.x, self.y - 1, self.grid_width, self.grid_height, self.obstacles),
            Point2D(self.x, self.y + 1, self.grid_width, self.grid_height, self.obstacles)
        ]

    def est_dist_to(self, end) -> float:
        '''The heuristic for the A* algorithm to use. Must be admissible in order for A* to provide a definitively optimal path.'''
        return (self.x - end.x)**2 + (self.y - end.y)**2

def test_bad_start():
    obstacles = set()
    start = Point2D(1, 1, 10, 10, obstacles)
    end = Point2D(5, 5, 10, 10, obstacles)
    obstacles.add(start)
    result = find_path(start, end)
    print(f"From {start} to {end}, {len(obstacles)} obstacle(s):")
    print(result)
    assert result is None

def test_blocked_end():
    obstacles = set()
    start = Point2D(1, 1, 10, 10, obstacles)
    end = Point2D(5, 5, 10, 10, obstacles)
    for n in end.get_neighbors():
        obstacles.add(n)
    result = find_path(start, end)
    print(f"From {start} to {end}, {len(obstacles)} obstacle(s):")
    print(result)
    assert result is None

def test_external_end():
    obstacles = set()
    start = Point2D(1, 1, 10, 10, obstacles)
    end = Point2D(15, 5, 10, 10, obstacles)
    result = find_path(start, end)
    print(f"From {start} to {end}, {len(obstacles)} obstacle(s):")
    print(result)
    assert result is None

def test_start_is_end():
    obstacles = set()
    start = Point2D(0, 0, 1, 1, obstacles)
    end = Point2D(0, 0, 1, 1, obstacles)
    result = find_path(start, end)
    print(f"From {start} to {end}, {len(obstacles)} obstacle(s):")
    print(result)
    assert len(result) == 1

def test_start_is_end_bigger_grid():
    obstacles = set()
    start = Point2D(5, 5, 10, 10, obstacles)
    end = Point2D(5, 5, 10, 10, obstacles)
    result = find_path(start, end)
    print(f"From {start} to {end}, {len(obstacles)} obstacle(s):")
    print(result)
    assert len(result) == 1

def test_one_move_border():
    obstacles = set()
    start = Point2D(0, 0, 10, 10, obstacles)
    end = Point2D(0, 1, 10, 10, obstacles)
    result = find_path(start, end)
    print(f"From {start} to {end}, {len(obstacles)} obstacle(s):")
    print(result)
    assert len(result) == 2

def test_one_move_interior():
    obstacles = set()
    start = Point2D(4, 4, 10, 10, obstacles)
    end = Point2D(4, 5, 10, 10, obstacles)
    result = find_path(start, end)
    print(f"From {start} to {end}, {len(obstacles)} obstacle(s):")
    print(result)
    assert len(result) == 2

def test_many_moves_no_obstacles():
    obstacles = set()
    start = Point2D(0, 0, 10, 10, obstacles)
    end = Point2D(7, 9, 10, 10, obstacles)
    result = find_path(start, end)
    print(f"From {start} to {end}, {len(obstacles)} obstacle(s):")
    print(result)
    assert len(result) == 17

def test_many_moves_req_all_directions():
    obstacles = set()
    start = Point2D(0, 0, 10, 3, obstacles)
    end = Point2D(4, 2, 10, 3, obstacles)
    for x, y in [(1, 0), (1, 1), (3, 1), (3, 2), (4, 1)]:
        obstacles.add(Point2D(x, y, 10, 3, obstacles))
    result = find_path(start, end)
    print(f"From {start} to {end}, {len(obstacles)} obstacle(s):")
    print(result)
    assert len(result) == 13

def test_avoid_traps():
    obstacles = set()
    start = Point2D(0, 0, 5, 5, obstacles)
    end = Point2D(4, 4, 5, 5, obstacles)
    for x, y in [(1, 2), (2, 1), (2, 2), (3, 2), (4, 2), (3, 4)]:
        obstacles.add(Point2D(x, y, 5, 5, obstacles))
    result = find_path(start, end)
    print(f"From {start} to {end}, {len(obstacles)} obstacle(s):")
    print(result)
    assert len(result) == 9

def test_avoid_traps_reflected():
    obstacles = set()
    start = Point2D(0, 0, 5, 5, obstacles)
    end = Point2D(4, 4, 5, 5, obstacles)
    for x, y in [(1, 2), (2, 1), (2, 2), (2, 3), (2, 4), (4, 3)]:
        obstacles.add(Point2D(x, y, 5, 5, obstacles))
    result = find_path(start, end)
    print(f"From {start} to {end}, {len(obstacles)} obstacle(s):")
    print(result)
    assert len(result) == 9