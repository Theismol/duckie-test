import heapq

class Node:
    def __init__(self, x, y, cost, parent=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent

    def __lt__(self, other):
        return self.cost < other.cost



def astar(grid, start, goal):
    open_set = []
    closed_set = set()

    start_node = Node(start[0], start[1], 0)
    heapq.heappush(open_set, start_node)

    while open_set:
        current_node = heapq.heappop(open_set)

        if (current_node.x, current_node.y) == goal:
            path = []
            while current_node is not None:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]

        closed_set.add((current_node.x, current_node.y))

        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            neighbor_x, neighbor_y = current_node.x + dx, current_node.y + dy
            if (0 <= neighbor_x < len(grid) and
                0 <= neighbor_y < len(grid[0]) and
                (neighbor_x, neighbor_y) not in closed_set):

                neighbor_cost = current_node.cost + grid[neighbor_x][neighbor_y]  # Consider the cost of the neighbor cell
                neighbor_node = Node(neighbor_x, neighbor_y, neighbor_cost, current_node)
                heapq.heappush(open_set, neighbor_node)

    return None  # No path found


# Example usage:
# grid = [
#     [1, 1, 2, 10, 1],
#     [1, 3, 1, 4, 1],
#     [1, 2.3, 1, 1, 1],
#     [1, 10, 3, 1, 1],
#     [1, 1, 1, 1, 1]
# ]

grid = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 30, 30, 30, 30, 0, 30, 30, 30, 30, 30, 30, 30, 30, 0],
    [0, 0, 0, 0, 30, 0, 30, 0, 0, 0, 0, 0, 30, 30, 0],
    [0, 30, 30, 30, 30, 0, 30, 0, 30, 30, 30, 0, 30, 0, 0],
    [0, 30, 0, 0, 0, 0, 30, 0, 30, 0, 30, 0, 30, 30, 0],
    [0, 30, 30, 30, 30, 0, 30, 0, 30, 0, 30, 0, 0, 0, 0],
    [0, 0, 0, 0, 30, 0, 30, 0, 30, 30, 30, 30, 30, 30, 0],
    [30, 30, 30, 0, 30, 0, 30, 0, 0, 0, 0, 0, 0, 30, 0],
    [0, 0, 30, 0, 30, 0, 30, 30, 30, 30, 30, 30, 0, 30, 0],
    [0, 30, 30, 0, 30, 0, 0, 0, 0, 0, 0, 30, 0, 30, 0],
    [0, 30, 0, 0, 30, 0, 30, 30, 30, 30, 0, 30, 0, 30, 30],
    [0, 30, 30, 30, 30, 0, 30, 0, 0, 30, 0, 30, 0, 30, 0],
    [0, 0, 0, 0, 0, 0, 30, 0, 30, 30, 0, 30, 0, 30, 0],
    [0, 30, 30, 30, 30, 30, 30, 0, 0, 0, 0, 30, 0, 30, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]


start = (0, 0)
goal = (14, 14)

path = astar(grid, start, goal)
if path:
    print("Path found:", path)
else:
    print("No path found")

