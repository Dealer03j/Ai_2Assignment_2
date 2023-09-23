import heapq
from enum import Enum
import time

# --------------- MAIN TREE/GRAPH ALGOS ---------------------------------------

def uniform_cost_graph_search_from_psuedo_code(start, rows, columns):

    start_time = time.time()

    closed = set()
    fringe = [((0,0,0), start)]  # Priority queue: (cost, state)

    came_from = {start: None}
    cost_so_far = {start: 0}

    generated_nodes = 0
    expanded_nodes = 0

    while True:

        if not fringe:
            total_time = time.time() - start_time
            return None, float('inf'), expanded_nodes, generated_nodes, total_time

        current_cost, current_state = heapq.heappop(fringe)
        current_cost = current_cost[0]
        expanded_nodes += 1

        if goal_test(current_state[1]):
            total_time = time.time() - start_time
            return reconstruct_path(came_from, current_state), current_cost, expanded_nodes, generated_nodes, total_time  # Return path and cost

        if current_state not in closed:
            closed.add(current_state)

            for next_state, action, action_cost in expand(current_state, rows, columns):

                new_cost = current_cost + action_cost
                #TODO: Remove best cost optimization
                if next_state not in cost_so_far or new_cost < cost_so_far[next_state]:
                    cost_so_far[next_state] = new_cost
                    priority = (new_cost, next_state[0][0], next_state[0][1])
                    generated_nodes += 1

                    # push in node in priority order
                    heapq.heappush(fringe, (priority, next_state))
                    came_from[next_state] = (current_state, action)


def uniform_cost_tree_search(start, rows, columns):
    frontier = [((0, 0, 0), start)]  # Priority queue: (cost, state)
    came_from = {start: None}

    generated_nodes = 0
    expanded_nodes = 0

    while frontier:
        current_cost, current_state = heapq.heappop(frontier)
        current_cost = current_cost[0]
        expanded_nodes += 1

        if goal_test(current_state[1]):
            return reconstruct_path(came_from, current_state), current_cost, expanded_nodes, generated_nodes  # Return path and cost

        for next_state, action, action_cost in expand(current_state, rows, columns):
            new_cost = current_cost + action_cost
            priority = (new_cost, next_state[0][0], next_state[0][1])
            generated_nodes += 1

            # push in node in priority order
            heapq.heappush(frontier, (priority, next_state))
            came_from[next_state] = (current_state, action)

    return None, float('inf'), expanded_nodes, generated_nodes  # Path not found

# -------------------------------------------------------------------------------------


class Result(Enum):
    CUTOFF = 1
    FAILURE = 2
    PASS = 3

def iterative_deeping_tree_search(start, rows, columns):

    start_time = time.time()

    limit = 1
    while True:
        (path, total_cost, expanded_nodes, generated_nodes, result) = DLS(limit, start, rows, columns)
        limit += 1
        if result == Result.PASS:
            total_time = time.time() - start_time
            return path, total_cost, expanded_nodes, generated_nodes, total_time


def DLS(limit, start, rows, columns):
    came_from = {start: None}
    generated_nodes = 0
    expanded_nodes = 0
    actionList = []

    # TODO: Pass rows and columns to recursive_DLS
    return recursive_DLS(actionList, came_from, expanded_nodes, generated_nodes, 0, start, limit, 0, rows, columns)


def recursive_DLS(actionList, came_from, expanded_nodes, generated_nodes, current_cost, current_state, limit, level, rows, columns):
    expanded_nodes += 1
    cutoff = False

    if goal_test(current_state[1]):
        return actionList, current_cost, expanded_nodes, generated_nodes, Result.PASS  # Return path and cost
    elif level == limit:
        return actionList, current_cost, expanded_nodes, generated_nodes, Result.CUTOFF  # Return path and cost
    else:
        level += 1

        successsors = expand(current_state, rows, columns)        
        sorted_successors = sorted(successsors, key=lambda x: x[0][0])

        for next_state, action, action_cost in sorted_successors:
            new_cost = current_cost + action_cost
            generated_nodes += 1
            came_from[next_state] = (current_state, action)
            actionList.append(action)

            (actionList, total_cost, expanded_nodes, generated_nodes, result) = recursive_DLS(actionList, came_from, expanded_nodes, generated_nodes, new_cost, next_state, limit, level, rows, columns)
            if result == Result.CUTOFF:
                actionList.pop()
                cutoff = True
            else:
                return actionList, total_cost, expanded_nodes, generated_nodes, result

    if cutoff:
        return actionList, current_cost, expanded_nodes, generated_nodes, Result.CUTOFF
    else:
        return actionList, current_cost, expanded_nodes, generated_nodes, Result.FAILURE


# -------------------------------------------------------------------------------------

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

    env_list[x][y] = 'C'
    new_env = tuple(tuple(row) for row in env_list)
    successors.append((((x, y), new_env), 'Suck', 0.6))

    return successors

def reconstruct_path(came_from, end):
    path = []
    while came_from[end]:
        end, action = came_from[end]
        path.append(action)
    path.reverse()
    return path

def goal_test(env):
    return all(cell == 'C' for row in env for cell in row)


# ----------------------- helpers ---------------------------------------------

# def count_dirty_rooms(env):
#     return sum(row.count('D') for row in env)


# def modify_2D_tuple(tup, x, y, value):

#     env_list = [list(row) for row in tup]

#     env_list[x][y] = 'V'

#     return tuple(tuple(row) for row in env_list)


# ---------- pretty printers ------------------------------------------
# def print_state(state):

#     env = state[1]
#     x, y = state[0]

#     env = modify_2D_tuple(env, x, y, 'V')

#     print_environment(env)


# def print_environment(env):
#     for row in env:
#         print(' '.join(row))
#     print("\n")  # Add a newline for bettr separation if multiple prints


def print_results(expanded_count, generated_count, path, total_cost, processing_time):

    print(f"  Processing Time: {processing_time} seconds")
    print(f"  First 5 actions: {path[:5]}")
    print(f"  Total cost: {total_cost}")
    print(f"  Generated Nodes: {generated_count}")
    print(f"  Expanded Nodes: {expanded_count}")


def run_search_algorithms(starting_env):

    rows = len(starting_env[1])
    columns = len(starting_env[1][0])

    # print("********* Uniform Cost Tree Search **************")
    # print("Running for up to 1 hour...")
    # path, total_cost, expanded, generated = uniform_cost_tree_search(starting_env, rows, columns)
    # print("\n***** Results *****")
    # print_results(expanded, generated, path, total_cost, 1)
    # print("\n")

    print("********* Uniform Cost Graph Search **************")
    print("Running for up to 1 hour...")
    path, total_cost, expanded, generated, processing_time = uniform_cost_graph_search_from_psuedo_code(starting_env, rows, columns)
    print("\n***** Results *****")
    print_results(expanded, generated, path, total_cost, processing_time)
    print("\n")

    print("********* Iterative Deepening Tree Search **************")
    print("Running for up to 1 hour...")
    path, total_cost, expanded, generated, processing_time = iterative_deeping_tree_search(starting_env, rows, columns)
    print("\n***** Results *****")
    print_results(expanded, generated, path, total_cost, processing_time)
    print("\n")


if __name__ == "__main__":

    print("\n************************* State 1 Results *************************\n")
    state_1 = ((1, 1), (
        ('C', 'D', 'C', 'C', 'C'),
        ('C', 'C', 'C', 'D', 'C'),
        ('C', 'C', 'C', 'C', 'D'),
        ('C', 'C', 'C', 'C', 'C')
    ))

    run_search_algorithms(state_1)

    print("\n\n\n\n************************* State 2 Results *************************\n")
    state_2 = ((2, 1), (
        ('C', 'D', 'C', 'C', 'C'),
        ('D', 'C', 'C', 'D', 'C'),
        ('C', 'C', 'D', 'C', 'C'),
        ('C', 'C', 'C', 'C', 'C')
    ))

    run_search_algorithms(state_2)

    # state = initial_state_2

    # rows = len(state[1])
    # columns = len(state[1][0])

    # print("Iterative Deepening Tree Search")
    # path, total_cost, expanded, generated = iterative_deeping_tree_search(state, rows, columns)


    # print("Uniform Cost Graph Search")
    # path, total_cost, expanded, generated = uniform_cost_graph_search_from_psuedo_code(state, rows, columns)
    # #print("Uniform cost Graph search")
    # #print(f"Actions to clean all rooms: {path}")
    # #print(f"Total cost: {total_cost}")
    # #print(f"Generated Nodes: {generated}")
    # #print(f"Expanded Nodes: {expanded}")


    # print("Uniform Cost Tree Search")
    # path, total_cost, expanded, generated = uniform_cost_tree_search(state, rows, columns)
    # print("\nUniform cost tree search")
    # print(f"Actions to clean all rooms: {path}")
    # print(f"Total cost: {total_cost}")
    # print(f"Generated Nodes: {generated}")
    # print(f"Expanded Nodes: {expanded}")
