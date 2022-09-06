import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost

USE_ITERATIVE_DEEPENING = False

def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    c1 = get_location(path1, 0)
    c2 = get_location(path2, 0)
    p1 = c1
    p2 = c2
    l = max(len(path1), len(path2))
    for t in range(1, l):
        c1 = get_location(path1, t)
        c2 = get_location(path2, t)
        if(c1==c2): ## vertex collision
            return {
                'timestep': t,
                'loc': [c1]
            }
        if(c1==p2 and c2==p1): ## edge collision
            return {
                'timestep': t,
                'loc': [p1, c1]
            }
        p1 = c1
        p2 = c2
    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    l = len(paths)
    collisions = []
    for i in range(l):
        for j in range(i+1,l):
            c = detect_collision(paths[i], paths[j])
            if(c != None):
                collisions.append(
                    {
                        'a1': i,
                        'a2': j,
                        'loc': c['loc'],
                        'timestep': c['timestep']
                    }
                )
    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    return {
        "agent": collision["a1"],
        "loc": collision["loc"],
        "timestep": collision["timestep"],
    }, {
        "agent": collision["a2"],
        "loc": list(reversed(collision["loc"])),
        "timestep": collision["timestep"],
    }


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    agentIndex = random.randint(0,1)
    agents = [collision['a1'], collision['a2']]
    c = collision['loc'] if agentIndex==0 else list(reversed(collision['loc']))
    t = collision['timestep']
    c_a = agents[agentIndex]
    return [
        {
            'agent': c_a,
            'loc': c,
            'timestep': t,
            'positive': True
        },
        {
            'agent': c_a,
            'loc': c,
            'timestep': t,
            'positive': False
        }
    ]


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0
        self.maxNodes = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        # print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        # print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def simple_add_nodes(self, nodes):
        nodes.sort(key=lambda node: -1*node['cost'])
        self.open_list.extend(nodes)
        # print("Generate {} node starting from {}".format(len(nodes),self.num_of_generated))
        self.num_of_generated += len(nodes)

    def simple_pop_node(self):
        node = self.open_list.pop()
        # print("Expand node {}".format(self.num_of_expanded))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        path = None
        if(USE_ITERATIVE_DEEPENING):
            (conflictCount, path) = self.IterativeDeepeningCBS(disjoint)
        else:
            (conflictCount, path) = self.normalCBS(disjoint)
        return (path, self.CPU_time, self.num_of_expanded, self.num_of_generated, self.maxNodes, conflictCount)

    def IterativeDeepeningCBS(self, disjoint=True):
        self.start_time = timer.time()
        ## to be implemented
        maxNodes = 0

        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            (maxNodes, path) = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        # print("collisions: {}".format(len(root['collisions'])))

        iteration_limit = 1000
        self.num_of_generated = 1
        self.num_of_expanded = 0
        for cost_limit in range(root['cost'], iteration_limit):
            self.open_list=[root]
            while(self.open_list):
                curr = self.simple_pop_node()
                if(len(curr['collisions']) == 0):
                    self.print_results(curr, maxNodes, len(root['collisions']))
                    return (len(root['collisions']), curr['paths'])
                collisions = curr['collisions']
                collision_sample = collisions[random.randint(0, len(collisions)-1)]
                if(disjoint):
                    new_constraints = disjoint_splitting(collision_sample)
                else:
                    new_constraints = standard_splitting(collision_sample)
                new_nodes = []
                for c in new_constraints:
                    new_node = {
                        'cost': 0,
                        'constraints': [c],
                        'paths': [],
                        'collisions': []
                    }
                    new_node['constraints'].extend(curr['constraints'])
                    new_node['paths'].extend(curr['paths'])
                    agents = [c['agent']]
                    if('positive' in c and c['positive']):
                        agents = paths_violate_constraint(c, new_node['paths'])
                    impossible_flag = False
                    maxSubNodes = 0
                    for agent in agents:
                        if(impossible_flag):
                            continue
                        (subNodeCount, path) = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                                        agent, new_node['constraints'])
                        maxSubNodes = max(maxSubNodes, subNodeCount)
                        if(path != None):
                            new_node['paths'][agent] = path
                        else:
                            impossible_flag = True
                    maxNodes = max(maxNodes, len(self.open_list)+maxSubNodes)
                    if(not impossible_flag):
                        new_node['collisions'] = detect_collisions(new_node['paths'])
                        new_node['cost'] = get_sum_of_cost(new_node['paths'])
                        if(new_node['cost']<=cost_limit):
                            new_nodes.append(new_node)
                self.simple_add_nodes(new_nodes)
        
        return None

    def normalCBS(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        maxNodes = 0
        
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            (maxNodes, path) = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        print("collisions: {}".format(len(root['collisions'])))
        self.push_node(root)

        # # Task 3.1: Testing
        # print(root['collisions'])

        # # Task 3.2: Testing
        # for collision in root['collisions']:
        #     print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        while(self.open_list):
            curr = self.pop_node()
            if(len(curr['collisions']) == 0):
                self.print_results(curr, maxNodes, len(root['collisions']))
                return (len(root['collisions']), curr['paths'])
            collisions = curr['collisions']
            collision_sample = collisions[random.randint(0, len(collisions)-1)]
            if(disjoint):
                new_constraints = disjoint_splitting(collision_sample)
            else:
                new_constraints = standard_splitting(collision_sample)
            for c in new_constraints:
                new_node = {
                    'cost': 0,
                    'constraints': [c],
                    'paths': [],
                    'collisions': []
                }
                new_node['constraints'].extend(curr['constraints'])
                new_node['paths'].extend(curr['paths'])
                agents = [c['agent']]
                if('positive' in c and c['positive']):
                    agents = paths_violate_constraint(c, new_node['paths'])
                impossible_flag = False
                maxSubNodes = 0
                for agent in agents:
                    if(impossible_flag):
                        continue
                    (subNodeCount, path) = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                                    agent, new_node['constraints'])
                    maxSubNodes = max(maxSubNodes, subNodeCount)
                    if(path != None):
                        new_node['paths'][agent] = path
                    else:
                        impossible_flag = True
                maxNodes = max(maxNodes, len(self.open_list)+maxSubNodes)
                if(not impossible_flag):
                    new_node['collisions'] = detect_collisions(new_node['paths'])
                    new_node['cost'] = get_sum_of_cost(new_node['paths'])
                    self.push_node(new_node)

        return None


    def print_results(self, node, maxNodes, collisionCount):
        print("\n Found a solution! \n")
        self.CPU_time = timer.time() - self.start_time
        self.maxNodes = maxNodes
        for path in node['paths']:
            print("\t",path)
        print("CPU time (s):        {:.10f}".format(self.CPU_time))
        print("Sum of costs:        {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:      {}".format(self.num_of_expanded))
        print("Generated nodes:     {}".format(self.num_of_generated))
        print("Maximum nodes:       {}".format(maxNodes))
        print("Initial Collision:   {}".format(collisionCount))
