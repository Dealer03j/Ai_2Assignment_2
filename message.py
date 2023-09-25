"""
Soures Credit: 
    We used ChatGPT to generate the intial boilerplate code for the algorithms 
    and then refactored the code to match the assignments requirements
"""


import heapq
from enum import Enum
import time

# --------------- UNIFORM COST GRAPH SEARCH---------------------------------------

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

        #Check if this statie has already been done before, avoids cycles
        if current_state not in closed:
            closed.add(current_state)

            for next_state, action, action_cost in expand(current_state, rows, columns):

                new_cost = current_cost + action_cost
                if next_state not in cost_so_far:
                    cost_so_far[next_state] = new_cost

                    #priortize nodes based first on cost and then on their x position.
                    priority = (new_cost, next_state[0][0], next_state[0][1])
                    generated_nodes += 1

                    # push in node in priority order
                    heapq.heappush(fringe, (priority, next_state))
                    came_from[next_state] = (current_state, action)


# ----------------------- UNIFORM COST TREE SEARACH ----------------------------------

## lacks the check for repeard actions, states and therefore is tree search
def uniform_cost_tree_search(start, rows, columns):

    start_time = time.time()
    max_runtime = 3600

    frontier = [((0, 0, 0), start, [])]  # Priority queue: priority is (cost, row, column)
    came_from = {start: None}

    generated_nodes = 0
    expanded_nodes = 0

    while frontier:

        total_time = time.time() - start_time

        # Makes the loop stop running after an hour
        if(total_time > max_runtime):
            return None, float('inf'), expanded_nodes, generated_nodes, max_runtime

        current_cost, current_state, current_actions = heapq.heappop(frontier)
        current_cost = current_cost[0]
        expanded_nodes += 1

        # GOAL TEST
        if goal_test(current_state[1]):
            total_time = time.time() - start_time
            return current_actions, current_cost, expanded_nodes, generated_nodes, total_time  # Return path and cost

        for next_state, action, action_cost in expand(current_state, rows, columns):
            new_cost = current_cost + action_cost

            #priortize nodes based first on cost and then on their x position.
            priority = (new_cost, next_state[0][0], next_state[0][1])
            generated_nodes += 1

            # push in node in priority order with updated actions
            heapq.heappush(frontier, (priority, next_state, current_actions + [tuple((next_state, action))]))
            came_from[next_state] = current_state

    total_time = time.time() - start_time
    return None, float('inf'), expanded_nodes, generated_nodes, total_time  # Path not found

# -------------------------------------------------------------------------------------

# -------------------------- ITERATIVE DEEP TREE SEARCH --------------------------------


#enum used for recurrsion similar to the alorighims shown in class
class Result(Enum):
    CUTOFF = 1
    FAILURE = 2
    PASS = 3

def iterative_deeping_tree_search(start, rows, columns):

    start_time = time.time()

    limit = 1
    
    # steps through starting from level 1 to infiniti
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

    return recursive_DLS(actionList, came_from, expanded_nodes, generated_nodes, 0, start, limit, 0, rows, columns)


#where recursion occurs
def recursive_DLS(actionList, came_from, expanded_nodes, generated_nodes, current_cost, current_state, limit, level, rows, columns):
    expanded_nodes += 1
    cutoff = False

    if goal_test(current_state[1]):
        return actionList, current_cost, expanded_nodes, generated_nodes, Result.PASS
    elif level == limit:
        return actionList, current_cost, expanded_nodes, generated_nodes, Result.CUTOFF
    else:
        level += 1

        successsors = expand(current_state, rows, columns)

        #SORT SUCCESSORS SO THAT WE ALWAYAS CONSIDER STATES IN THE CORRECT ORDER
        sorted_successors = sorted(successsors, key=lambda x: x[0][0])

        for next_state, action, action_cost in sorted_successors:
            new_cost = current_cost + action_cost
            generated_nodes += 1
            came_from[next_state] = (current_state, action)
            actionList.append(tuple((next_state, action)))

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


# --------------------------------------- HELPER FUNCTIONS ---------------------------
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
    path_with_actions = []
    while came_from[end]:
        end, action = came_from[end]
        path_with_actions.append((end, action))
    path_with_actions.reverse()
    return path_with_actions


# Method to check if we've reached the goal
def goal_test(env):
    return all(cell == 'C' for row in env for cell in row)

def print_results(expanded_count, generated_count, path, total_cost, processing_time):

    print(f"  Processing Time: {processing_time} seconds")
    print(f"  Generated Nodes: {generated_count}")
    print(f"  Expanded Nodes: {expanded_count}")

    if path:

        print(f"  First 5 actions")
        for i in range(0, 5):
            print(f"    {path[i]}")

        print(f"  Total cost: {total_cost}")
        print(f"  Complete path: {[x[1] for x in path]}")
        print(f"  Path Length: {len(path)}")
    else:
        print("  First 5 actions: Not found")
        print("  Total cost: Uknown")
    


def run_search_algorithms(starting_env):

    rows = len(starting_env[1])
    columns = len(starting_env[1][0])

    print("********* Iterative Deepening Tree Search **************")
    print("Running...")
    path, total_cost, expanded, generated, processing_time = iterative_deeping_tree_search(starting_env, rows, columns)
    print("\n***** Results *****")
    print_results(expanded, generated, path, total_cost, processing_time)
    print("\n")

    print("********* Uniform Cost Graph Search **************")
    print("Running...")
    path, total_cost, expanded, generated, processing_time = uniform_cost_graph_search_from_psuedo_code(starting_env, rows, columns)
    print("\n***** Results *****")
    print_results(expanded, generated, path, total_cost, processing_time)
    print("\n")

    print("********* Uniform Cost Tree Search **************")
    print("Running...")
    path, total_cost, expanded, generated, processing_time = uniform_cost_tree_search(starting_env, rows, columns)
    print("\n***** Results *****")
    print_results(expanded, generated, path, total_cost, processing_time)
    print("\n")


if __name__ == "__main__":

    print("\n************************* State 1 Results ***********************************\n")
    state_1 = ((1, 1), (
        ('C', 'D', 'C', 'C', 'C'),
        ('C', 'C', 'C', 'D', 'C'),
        ('C', 'C', 'C', 'C', 'D'),
        ('C', 'C', 'C', 'C', 'C')
    ))

    run_search_algorithms(state_1)

    print("\n\n************************* State 2 Results ***********************************\n")
    state_2 = ((2, 1), (
        ('C', 'D', 'C', 'C', 'C'),
        ('D', 'C', 'C', 'D', 'C'),
        ('C', 'C', 'D', 'C', 'C'),
        ('C', 'C', 'C', 'C', 'C')
    ))

    run_search_algorithms(state_2)
