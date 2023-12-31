import heapq


def reconstruct_path(came_from, end):
    path = []
    while came_from[end]:
        end, action = came_from[end]
        print(action)
        path.append(action)
    path.reverse()
    return path


def uniform_cost_graph_search_from_psuedo_code(start, rows, columns):

    closed = set()
    fringe = [(0, start)]  # Priority queue: (cost, state)

    came_from = {start: None}
    cost_so_far = {start: 0}

    generated_nodes = 0
    expanded_nodes = 0

    while True:

        if not fringe:
            return None, float('inf'), expanded_nodes, generated_nodes 

        current_cost, current_state = heapq.heappop(fringe)
        expanded_nodes += 1

        if goal_test(current_state[1]):
            return reconstruct_path(came_from, current_state), current_cost, expanded_nodes, generated_nodes  # Return path and cost

        if current_state not in closed:
            closed.add(current_state)

            for next_state, action, action_cost in expand(current_state, rows, columns):

                new_cost = current_cost + action_cost
                if next_state not in cost_so_far or new_cost < cost_so_far[next_state]:
                    cost_so_far[next_state] = new_cost
                    priority = new_cost
                    generated_nodes += 1
                    heapq.heappush(fringe, (priority, next_state))
                    came_from[next_state] = (current_state, action)


def uniform_cost_tree_search(start, rows, columns):
    frontier = [(0, start)]  # Priority queue: (cost, state)
    came_from = {start: None}

    generated_nodes = 0
    expanded_nodes = 0

    while frontier:

        current_cost, current_state = heapq.heappop(frontier)
        expanded_nodes += 1

        dirty_room_count = count_dirty_rooms(current_state[1])

        if goal_test(current_state[1]):
            return reconstruct_path(came_from, current_state), current_cost, expanded_nodes, generated_nodes  # Return path and cost

        for next_state, action, action_cost in expand(current_state, rows, columns):
            new_cost = current_cost + action_cost
            priority = new_cost
            generated_nodes += 1
            heapq.heappush(frontier, (priority, next_state))
            came_from[next_state] = (current_state, action)

    return None, float('inf'), expanded_nodes, generated_nodes  # Path not found


def expand(state, rows, columns):
    (x, y), env = state
    successors = []

    # Convert env to a list of lists for manipulation
    env_list = [list(row) for row in env]

    # Check if we can move to adjacent rooms and get the associated costs
    if x > 0:
        successors.append((((x-1, y), env), 'Up', 0.8))
    if x < rows-1:
        successors.append((((x+1, y), env), 'Down', 0.7))
    if y > 0:
        successors.append((((x, y-1), env), 'Left', 1.0))
    if y < columns-1:
        successors.append((((x, y+1), env), 'Right', 0.9))

    # Check if the current room is dirty
    if env_list[x][y] == 'D':
        env_list[x][y] = 'C'
        new_env = tuple(tuple(row) for row in env_list)
        successors.append((((x, y), new_env), 'Suck', 0.6))

    return successors


def goal_test(env):
    return all(cell == 'C' for row in env for cell in row)

def count_dirty_rooms(env):
    return sum(row.count('D') for row in env)


def print_state(state):

    env = state[1]
    x, y = state[0]

    env = modify_2D_tuple(env, x, y, 'V')

    print_environment(env)

def print_environment(env):
    for row in env:
        print(' '.join(row))
    print("\n")  # Add a newline for better separation if multiple prints


def modify_2D_tuple(tup, x, y, value):

    env_list = [list(row) for row in tup]

    env_list[x][y] = 'V'

    return tuple(tuple(row) for row in env_list)


if __name__ == "__main__":

    initial_state_1 = ((1, 1), (
        ('C', 'D', 'C', 'C', 'C'),
        ('C', 'C', 'C', 'D', 'C'),
        ('C', 'C', 'C', 'C', 'D'),
        ('C', 'C', 'C', 'C', 'C')
    ))

    initial_state_2 = ((2, 1), (
        ('C', 'D', 'C', 'C', 'C'),
        ('D', 'C', 'C', 'D', 'C'),
        ('C', 'C', 'D', 'C', 'C'),
        ('C', 'C', 'C', 'C', 'C')
    ))

    state = initial_state_2

    rows = len(state[1])
    columns = len(state[1][0])

    path, total_cost, expanded, generated = uniform_cost_graph_search(state, rows, columns)
    print("Uniform cost Graph search")
    print(f"Actions to clean all rooms: {path}")
    print(f"Total cost: {total_cost}")
    print(f"Generated Nodes: {generated}")
    print(f"Expanded Nodes: {expanded}")


    path, total_cost, expanded, generated = uniform_cost_tree_search(state, rows, columns)
    print("\nUniform cost tree search")
    print(f"Actions to clean all rooms: {path}")
    print(f"Total cost: {total_cost}")
    print(f"Generated Nodes: {generated}")
    print(f"Expanded Nodes: {expanded}")
