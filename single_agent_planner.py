import heapq

USE_ITERATIVE_DEEPENING = False

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0,0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    ve_constraints = dict()
    blockedLocation = dict()
    positive = dict()
    for c in constraints:
        time = c['timestep']
        loc = c['loc']
        if('positive' in c and c['positive']):
            if(c['agent'] == agent):
                positive[time] = loc
            else:
                add_negated_positive_constraints(ve_constraints, c)
            continue
        if(c['agent'] != agent):
            continue
        if('allFuture' in c and c['allFuture']):
            blockedLocation[loc[0]] = time
            continue
        add_normal_constraints(ve_constraints, c)

    c_table = {
        've': ve_constraints,
        'b': blockedLocation,
        'p': positive,
    }
    return c_table


def add_negated_positive_constraints(ve_table, c):
    loc = c['loc']
    t = c['timestep']
    if(len(loc) == 1):
        add_normal_constraints(ve_table, c)
    else:
        locs = [[loc[0]], [loc[1]], [loc[1], loc[0]]]
        time = [t-1, t, t]
        for i in range(3):
            new_c = {
                'loc': locs[i],
                'timestep': time[i]
            }
            add_normal_constraints(ve_table, new_c)


def add_normal_constraints(ve_table, c):
    time = c['timestep']
    loc = c['loc']
    if(time not in ve_table):
        ve_table[time] = [loc]
    else:
        ve_table[time].append(loc)


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    while(len(path) >= 2 and path[-1] == path[-2]):
        path.pop()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    ve_c = constraint_table['ve']
    b_c = constraint_table['b']
    p_c = constraint_table['p']
    if(next_time in ve_c and 
            ([next_loc] in ve_c[next_time] or [curr_loc, next_loc] in ve_c[next_time])):
        return True
    if(next_loc in b_c and next_time >= b_c[next_loc]):
        return True
    if(next_time in p_c):
        p_loc = p_c[next_time]
        if(len(p_loc)==1 and [next_loc] != p_loc):
            return True
        if(len(p_c)==2 and [curr_loc, next_loc] != p_loc):
            return True
    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.
    if(USE_ITERATIVE_DEEPENING):
        return iterative_deepening_a_star(my_map, start_loc, goal_loc, h_values, agent, constraints)
    else:
        return normal_a_star(my_map, start_loc, goal_loc, h_values, agent, constraints)
        

def iterative_deepening_a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    maxNodeCount = 0
    c_table = build_constraint_table(constraints, agent)
    root = {
        'loc': start_loc, 
        'g_val': 0, 
        'h_val': h_values[start_loc], 
        'timestep': 0,
        'parent': None
    }
    min_time = 0
    for c in constraints:
        if(c['agent']==agent and c['timestep']>min_time):
            min_time = c['timestep']
    time_limit = len(my_map)*len(my_map[0])//2
    for iterative_time_limit in range(1, time_limit):
        # print("start time limit = ", iterative_time_limit)
        open_list = []
        open_list.append(root)
        while(len(open_list)>0):
            curr = open_list.pop()
            maxNodeCount = max(maxNodeCount, len(open_list))
            if(curr['loc'] == goal_loc and curr['timestep'] >= min_time):
                return (maxNodeCount, get_path(curr))
            if(curr['timestep']>iterative_time_limit):
                continue
            child_nodes = []
            for dir in range(5):
                child_loc = move(curr['loc'], dir)
                if((not 0<=child_loc[0]<len(my_map)) or (not 0<= child_loc[1]<len(my_map[0]))):
                    continue
                if my_map[child_loc[0]][child_loc[1]]:
                    continue
                if(is_constrained(curr['loc'], child_loc, curr['timestep']+1, c_table)):
                    continue
                child = {
                    'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'timestep': curr['timestep']+1,
                    'parent': curr
                }
                child_nodes.append(child)
            child_nodes.sort(key=lambda node:-1*(node['g_val'] + node['h_val']))
            open_list.extend(child_nodes)


    return (maxNodeCount, None)

def normal_a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.
    maxNodeCount = 0

    open_list = []
    closed_list = dict()
    h_value = h_values[start_loc]
    c_table = build_constraint_table(constraints, agent)
    root = {
        'loc': start_loc, 
        'g_val': 0, 
        'h_val': h_value, 
        'timestep': 0,
        'parent': None
    }
    min_time = 0
    for c in constraints:
        if(c['agent']==agent and c['timestep']>min_time):
            min_time = c['timestep']
    push_node(open_list, root)
    closed_list[(root['loc'], root["timestep"])] = root
    time_limit = len(my_map)*len(my_map[0])
    while len(open_list) > 0:
        curr = pop_node(open_list)
        maxNodeCount = max(maxNodeCount, len(open_list)+len(closed_list))
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if(curr['loc'] == goal_loc and curr['timestep'] >= min_time):
            # print(curr['timestep'])
            return (maxNodeCount, get_path(curr))
        if(curr['timestep']>time_limit):
            continue
        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            if((not 0<=child_loc[0]<len(my_map)) or (not 0<= child_loc[1]<len(my_map[0]))):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            if(is_constrained(curr['loc'], child_loc, curr['timestep']+1, c_table)):
                continue
            child = {
                'loc': child_loc,
                'g_val': curr['g_val'] + 1,
                'h_val': h_values[child_loc],
                'timestep': curr['timestep']+1,
                'parent': curr
            }
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)

    return (maxNodeCount, None)  # Failed to find solutions
