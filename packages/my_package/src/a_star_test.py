import heapq



def astar(start, goal, graph):
    open_set = {}
    closed_set = {}

    # Add the starting node to the open set
    open_set[start] = (0, [start], 0)  # Add a third element to track the total cost

    while open_set:
        # Get the node with the lowest total cost from the open set
        current_node = min(open_set, key=lambda node: open_set[node][2])
        current_cost, path, total_cost = open_set[current_node]
        

        # Remove the current node from the open set
        del open_set[current_node]

        # If we've reached the goal, return the path and its cost
        if current_node == goal:
            return path, total_cost

        # Add the current node to the closed set
        closed_set[current_node] = (current_cost, total_cost)

        # Check each neighbor of the current node
        for neighbor, weight in graph[current_node].items():
            # Calculate the tentative g score by adding the current cost and the edge weight
            tentative_g_score = current_cost + weight

            # Calculate the tentative f score
            h_score = heuristic(neighbor, goal)
            tentative_f_score = tentative_g_score + h_score
            
            # print(f"current_node: {current_node}, neighbor: {neighbor}, tentative_f_score: {tentative_f_score}, tentative_g_score: {tentative_g_score}, h_score: {h_score}, weight: {weight}, current_cost: {current_cost}, total_cost: {total_cost}")

            # Calculate the total cost by adding the current total cost and the edge weight
            tentative_total_cost = total_cost + weight

            if neighbor in closed_set and tentative_total_cost >= closed_set[neighbor][1]:
                continue

            if neighbor not in open_set or tentative_total_cost < open_set[neighbor][2]:
                open_set[neighbor] = (tentative_f_score, path + [neighbor], tentative_total_cost)

    # If we've exhausted all possible paths and haven't found the goal, return None
    return None, None

# Rest of your code remains the same


def heuristic(node, goal):
    # In this example, we'll use the Manhattan distance as our heuristic
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])


def get_directions(path):
    directions = []
    directions_numbers = []
    directions_dict = {}
    for i in range(len(path) - 1):
     current_node = path[i]
     next_node = path[i + 1]
     if next_node[0] > current_node[0]:
         directions.append('down')
     elif next_node[0] < current_node[0]:
         directions.append('up')
     elif next_node[1] > current_node[1]:
         directions.append('right')
     elif next_node[1] < current_node[1]:
         directions.append('left')
    current_direction = directions[0]
    for direction in directions:
        if direction == current_direction:
            directions_numbers.append("f")
        elif direction == "up" and current_direction == "right" or direction == "right" and current_direction == "down" or direction == "down" and current_direction == "left" or direction == "left" and current_direction == "up":
            directions_numbers.append("l")
            current_direction=direction
        elif direction == "up" and current_direction == "left" or direction == "left" and current_direction == "down" or direction == "down" and current_direction == "right" or direction == "right" and current_direction == "up":
            directions_numbers.append("r")
            current_direction=direction
    for i in range(len(path)-1):
        directions_dict[path[i]] = directions_numbers[i]
    return directions_dict

def find_intersections(path, graph):
    in_intersection_list = []
    start_intersection_list = []
    new_path = {}
    for node, weight in graph.items():
        if (len(graph[node].items()) > 1 or weight == 20) and node in path:
            in_intersection_list.append(node)
            print(node)
    for node in graph:
        if graph[node] and list(graph[node].keys())[0] in in_intersection_list and node not in in_intersection_list and node in path:
            print(node)
            start_intersection_list.append(node)
    for node, direction in path.items():
        if node in start_intersection_list or node in in_intersection_list:
            new_path[node] = direction
    return new_path


def get_instructions(path, graph):
    instructions = []
    for i in range(len(path) - 2):
        current_node = path[i]
        next_node = path[i + 1]
        next_next_node = path[i + 2]

        if next_next_node in graph[next_node] and graph[next_node][next_next_node] == 20:
            dx1 = next_node[0] - current_node[0]
            dy1 = next_node[1] - current_node[1]
            dx2 = next_next_node[0] - next_node[0]
            dy2 = next_next_node[1] - next_node[1]

            cross_product = dx1 * dy2 - dy1 * dx2

            if cross_product == 1:  # Right
                instructions.append('2')
            elif cross_product == -1:  # Left
                instructions.append('3')
            else:  # Forward
                instructions.append('1')

    return instructions




    
    



'''def calculate_cost_of_path(path, graph):
    cost = 0
    for i in range(len(path) - 1):
        cost += graph[path[i]][path[i + 1]]
    return cost
'''



graph = {
    (0, 0): {},
    (0, 1): {},
    (0, 2): {},
    (0, 3): {},
    (0, 4): {(1,4):1},
    (0, 5): {(0,4):1},
    (0, 6): {(0,5):1},
    (0, 7): {(0,6):1},
    (0, 8): {(0,7):1},
    (0, 9): {(0,8):1},
    (0, 10): {(0,9):1},
    (0, 11): {(0,10):1},
    (0, 12): {},
    (0, 13): {},
    (0, 14): {},
    (0, 15): {},
    (1, 0): {},
    (1, 1): {},
    (1, 2): {},
    (1, 3): {},
    (1, 4): {(2,4):1},
    (1, 5): {(1,6):1},
    (1, 6): {(1,7):1},
    (1, 7): {(1,8):1},
    (1, 8): {(1,9):1},
    (1, 9): {(1,10):1},
    (1, 10): {(2,10):1},
    (1, 11): {(0,11):1},
    (1, 12): {},
    (1, 13): {},
    (1, 14): {},
    (1, 15): {},
    (2, 0): {},
    (2, 1): {},
    (2, 2): {},
    (2, 3): {},
    (2, 4): {(3,4):1},
    (2, 5): {(1,5):1},
    (2,6): {},
    (2, 7): {},
    (2, 8): {},
    (2, 9): {},
    (2, 10): {(3,10):1},
    (2, 11): {(1,11):1},
    (2, 12): {},
    (2, 13): {},
    (2, 14): {},
    (2, 15): {},
    (3, 0): {},
    (3, 1): {},
    (3, 2): {},
    (3, 3): {},
    (3, 4): {(4,4):1},
    (3, 5): {(2,5):1},
    (3, 6): {},
    (3, 7): {},
    (3, 8): {},
    (3, 9): {},
    (3, 10): {(4,10):1},
    (3, 11): {(2,11):1},
    (3, 12): {},
    (3, 13): {},
    (3, 14): {},
    (3, 15): {},
    (4, 0): {},
    (4, 1): {},
    (4, 2): {(5,2):1},
    (4, 3): {(4,2):1},
    (4, 4): {(4,3):1,(5,4):20},
    (4, 5): {(4,4):20,(3,5):1},
    (4, 6): {},
    (4, 7): {},
    (4, 8): {(5,8):1},
    (4, 9): {(4,8):1},
    (4, 10): {(4,9):1,(5,10):20},
    (4, 11): {(4,10):20,(3,11):1},
    (4, 12): {(4,11):1},
    (4, 13): {(4,12):1},
    (4, 14): {},
    (4, 15): {},
    (5, 0): {},
    (5, 1): {},
    (5, 2): {(6,2):1},
    (5, 3): {(5,4):1},
    (5, 4): {(6,4):1,(5,5):20},
    (5, 5): {(4,5):20},
    (5, 6): {},
    (5, 7): {},
    (5, 8): {(6,8):1},
    (5, 9): {(5,10):1},
    (5, 10): {(5,11):20},
    (5, 11): {(4,11):20,(5,12):1},
    (5, 12): {(6,12):1},
    (5, 13): {(4,13):1},
    (5, 14): {},
    (5, 15): {},
    (6, 0): {},
    (6, 1): {(6,0):1},
    (6, 2): {(6,1):1,(7,2):20},
    (6, 3): {(6,2):20,(5,3):1},
    (6, 4): {(7,4):1},
    (6, 5): {(5,5):1},
    (6, 6): {},
    (6, 7): {},
    (6, 8): {(7,8):1},
    (6, 9): {(5,9):1},
    (6, 10): {},
    (6, 11): {},
    (6, 12): {(7,12):1},
    (6, 13): {(5,13):1},
    (6, 14): {},
    (6, 15): {},
    (7, 0): {(7,1):1},
    (7, 1): {(7,2):1},
    (7, 2): {(8,2):1,(7,3):20},
    (7, 3): {(6,3):20},
    (7, 4): {(8,4):1},
    (7, 5): {(6,5):1},
    (7, 6): {},
    (7, 7): {},
    (7, 8): {(8,8):1},
    (7, 9): {(6,9):1},
    (7, 10): {},
    (7, 11): {},
    (7, 12): {(8,12):1},
    (7, 13): {(6,13):1},
    (7, 14): {},
    (7, 15): {},
    (8, 0): {},
    (8, 1): {},
    (8, 2): {(9,2):1},
    (8, 3): {(7,3):1},
    (8, 4): {(9,4):1},
    (8, 5): {(7,5):1},
    (8, 6): {},
    (8, 7): {},
    (8, 8): {(9,8):1},
    (8, 9): {(7,9):1},
    (8, 10): {},
    (8, 11): {},
    (8, 12): {(9,12):1},
    (8, 13): {(7,13):1},
    (8, 14): {},
    (8, 15): {},
    (9, 0): {},
    (9, 1): {},
    (9, 2): {(10,2):1},
    (9, 3): {(8,3):1},
    (9, 4): {(10,4):1},
    (9, 5): {(8,5):1},
    (9, 6): {},
    (9, 7): {},
    (9, 8): {(10,8):1},
    (9, 9): {(8,9):1},
    (9, 10): {},
    (9, 11): {},
    (9, 12): {(10,12):1},
    (9, 13): {(8,13):1},
    (9, 14): {},
    (9, 15): {},
    (10, 0): {},
    (10, 1): {},
    (10, 2): {(11,2):1},
    (10, 3): {(9,3):1},
    (10, 4): {(10,3):1,(11,4):20},
    (10, 5): {(9,5):1,(10,4):20},
    (10, 6): {(10,5):1},
    (10, 7): {(10,6):1},
    (10, 8): {(10,7):1,(11,8):20},
    (10, 9): {(9,9):1,(10,8):20},
    (10, 10): {(10,9):1},
    (10, 11): {(10,10):1},
    (10, 12): {(10,11):1,(11,12):20},
    (10, 13): {(9,13):1,(10,12):20},
    (10, 14): {(10,13):1},
    (10, 15): {(10,14):1},
    (11, 0): {},
    (11, 1): {},
    (11, 2): {(11,3):1},
    (11, 3): {(11,4):1},
    (11, 4): {(11,5):20},
    (11, 5): {(11,6):1,(10,5):20},
    (11, 6): {(11,7):1},
    (11, 7): {(11,8):1},
    (11, 8): {(11,9):20},
    (11, 9): {(11,10):1,(10,9):20},
    (11, 10): {(11,11):1},
    (11, 11): {(11,12):1},
    (11, 12): {(11,13):20},
    (11, 13): {(11,14):1,(10,13):20},
    (11, 14): {(11,15):1},
    (11, 15): {}
}



# Example start and goal nodes
if __name__ == '__main__':
    start_node = (10, 6)
    goal_node = (1, 6)

    path, cost = astar(start_node, goal_node, graph)
    if path:
        print("Path found:", path)
        print("Total cost:", cost)
    else:
        print("No path found")
    # directions = get_directions(path)
    # print(directions)
    # print(find_intersections(directions, graph))

# Create an image to visualize the maze and the path