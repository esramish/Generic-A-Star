import heapq
from typing import List

class AStarableI:
    def __eq__(self, __o: object) -> bool:
        pass

    def __hash__(self) -> int:
        pass
    
    def is_valid(self) -> bool:
        pass

    def get_neighbors(self) -> List:
        '''Get all the neighbors of this location/node/payload. They don't have to be valid; the A* algorithm will query validity later.'''
        pass

    def est_dist_to(self, end) -> float:
        '''The heuristic for the A* algorithm to use. Must be admissible in order for A* to provide a definitively optimal path.'''
        pass

class HeapEntry:
    def __init__(self, payload, dist_from_start, est_dist_from_end, prev=None):
        self.payload = payload
        self.dist_from_start = dist_from_start
        self.est_dist_from_end = est_dist_from_end
        self.prev = prev
    
    def est_total_dist(self):
        return self.dist_from_start + self.est_dist_from_end

    def __lt__(self, other):
        return self.est_total_dist() < other.est_total_dist()

def find_path(start, end):
    already_seen = set()
    queue = [HeapEntry(start, 0, start.est_dist_to(end))]
    while queue:
        # Pop
        curr_heap_entry: HeapEntry = heapq.heappop(queue)
        
        # Check if found solution
        payload: AStarableI = curr_heap_entry.payload
        if payload == end:
            break
        
        # Check if already came here before
        if payload in already_seen:
            continue

        # Note that we've been here
        already_seen.add(payload)

        # Check if valid
        if not payload.is_valid():
            continue

        # Add neighbors to the queue
        for neighbor in payload.get_neighbors():
            heapq.heappush(queue, HeapEntry(neighbor, curr_heap_entry.dist_from_start + 1, neighbor.est_dist_to(end), curr_heap_entry))
    else:
        # No solution
        return

    # Found a solution!

    # Construct the path, end to start
    path = [end]
    while(curr_heap_entry.prev):
        curr_heap_entry = curr_heap_entry.prev
        path.append(curr_heap_entry.payload)
    
    # Reverse the path so it's start to end
    path = path[::-1]

    # Return it!
    return path
    



