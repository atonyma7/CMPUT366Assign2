from search.algorithms import State
from search.map import Map
import getopt
import sys
import heapq

def astar(start, goal, gridded_map):
    # open list is heap
    open_list = []
    node_count = 0

    # closed list is hashmap or dict in python
    closed_list = dict()
    heapq.heappush(open_list, start)
    start.set_cost(get_fcost(start, goal))
    closed_list[start.state_hash()] = start
    while open_list:
        n = heapq.heappop(open_list)
        node_count+=1
        if n == goal:
            return n.get_cost(), node_count
        for n_prime in gridded_map.successors(n):
            n_prime.set_cost(get_fcost(n_prime, goal))
            if n_prime.state_hash() not in closed_list:
                heapq.heappush(open_list, n_prime)
                closed_list[n_prime.state_hash()] = n_prime
            if n_prime.state_hash() in closed_list and n_prime.get_g() < closed_list[n_prime.state_hash()].get_g():
                heapq.heappush(open_list, n_prime)
                closed_list[n_prime.state_hash()] = n_prime
    return -1, node_count

def bi_astar(start, goal, gridded_map):
    open_forward = []
    open_back = []
    closed_forward = dict()
    closed_back = dict()
    u = float("inf")
    node_count = 0

    start.set_cost(get_fcost(start, goal))
    goal.set_cost(get_fcost(goal, start))

    heapq.heappush(open_forward, start)
    heapq.heappush(open_back, goal)

    closed_forward[start.state_hash()] = start
    closed_back[goal.state_hash()] = goal
    
    while open_forward and open_back:
        if u <= min(open_forward[0].get_cost(), open_back[0].get_cost()):
            return u, node_count
        if open_forward[0].get_cost() < open_back[0].get_cost():
            n = heapq.heappop(open_forward)
            node_count+=1
            for n_prime in gridded_map.successors(n):
                n_prime.set_cost(get_fcost(n_prime, goal))
                if n_prime.state_hash() in closed_back:
                    u = min(u, n_prime.get_g() + closed_back[n_prime.state_hash()].get_g())
                if n_prime.state_hash() not in closed_forward:
                    heapq.heappush(open_forward, n_prime)
                    closed_forward[n_prime.state_hash()] = n_prime
                if n_prime.state_hash() in closed_forward and n_prime.get_g() < closed_forward[n_prime.state_hash()].get_g():
                    closed_forward[n_prime.state_hash()].set_g(n_prime.get_g())
                    closed_forward[n_prime.state_hash()].set_cost(n_prime.get_cost())
                    heapq.heapify(open_forward)

        else:
            n = heapq.heappop(open_back)
            node_count+=1
            for n_prime in gridded_map.successors(n):
                n_prime.set_cost(get_fcost(n_prime, start))
                if n_prime.state_hash() in closed_forward:
                    u = min(u, n_prime.get_g() + closed_forward[n_prime.state_hash()].get_g())
                if n_prime.state_hash() not in closed_back:
                    heapq.heappush(open_back, n_prime)
                    closed_back[n_prime.state_hash()] = n_prime
                if n_prime.state_hash() in closed_back and n_prime.get_g() < closed_back[n_prime.state_hash()].get_g():
                    closed_back[n_prime.state_hash()].set_g(n_prime.get_g())
                    closed_back[n_prime.state_hash()].set_cost(n_prime.get_cost())
                    heapq.heapify(open_back)

    return -1, node_count


def mm(start, goal, gridded_map):
    open_forward = []
    open_back = []
    closed_forward = dict()
    closed_back = dict()
    u = float("inf")
    node_count = 0

    start.set_cost(get_pcost(start, goal))
    goal.set_cost(get_pcost(goal, start))

    heapq.heappush(open_forward, start)
    heapq.heappush(open_back, goal)

    closed_forward[start.state_hash()] = start
    closed_back[goal.state_hash()] = goal
    
    while open_forward and open_back:
        if u <= min(open_forward[0].get_cost(), open_back[0].get_cost()):
            return u, node_count
        if open_forward[0].get_cost() < open_back[0].get_cost():
            n = heapq.heappop(open_forward)
            node_count+=1
            for n_prime in gridded_map.successors(n):
                n_prime.set_cost(get_pcost(n_prime, goal))
                if n_prime.state_hash() in closed_back:
                    u = min(u, n_prime.get_g() + closed_back[n_prime.state_hash()].get_g())
                if n_prime.state_hash() not in closed_forward:
                    heapq.heappush(open_forward, n_prime)
                    closed_forward[n_prime.state_hash()] = n_prime
                if n_prime.state_hash() in closed_forward and n_prime.get_g() < closed_forward[n_prime.state_hash()].get_g():
                    closed_forward[n_prime.state_hash()].set_g(n_prime.get_g())
                    closed_forward[n_prime.state_hash()].set_cost(n_prime.get_cost())
                    heapq.heapify(open_forward)

        else:
            n = heapq.heappop(open_back)
            node_count+=1
            for n_prime in gridded_map.successors(n):
                n_prime.set_cost(get_pcost(n_prime, start))
                if n_prime.state_hash() in closed_forward:
                    u = min(u, n_prime.get_g() + closed_forward[n_prime.state_hash()].get_g())
                if n_prime.state_hash() not in closed_back:
                    heapq.heappush(open_back, n_prime)
                    closed_back[n_prime.state_hash()] = n_prime
                if n_prime.state_hash() in closed_back and n_prime.get_g() < closed_back[n_prime.state_hash()].get_g():
                    closed_back[n_prime.state_hash()].set_g(n_prime.get_g())
                    closed_back[n_prime.state_hash()].set_cost(n_prime.get_cost())
                    heapq.heapify(open_back)

    return -1, node_count

def get_fcost(node, goal):
    delta_x = abs(node.get_x() - goal.get_x())
    delta_y = abs(node.get_y() - goal.get_y())
    hcost = 1.5 * min(delta_x, delta_y) + abs(delta_x - delta_y)
    return node.get_g() + hcost

def get_pcost(node, goal):
    return max(get_fcost(node, goal), 2 * node.get_g())

def main():
    """
    Function for testing your implementation. Run it with a -help option to see the options available. 
    """
    optlist, _ = getopt.getopt(sys.argv[1:], 'h:m:r:', ['testinstances', 'plots', 'help'])

    plots = False
    for o, _ in optlist:
        if o in ("-help"):
            print("Examples of Usage:")
            print("Solve set of test instances: main.py --testinstances")
            print("Solve set of test instances and generate plots: main.py --testinstances --plots")
            exit()
        elif o in ("--plots"):
            plots = True
    test_instances = "test-instances/testinstances.txt"
    gridded_map = Map("dao-map/brc000d.map")
    
    nodes_expanded_biastar = []   
    nodes_expanded_astar = []   
    nodes_expanded_mm = []
    
    start_states = []
    goal_states = []
    solution_costs = []
       
    file = open(test_instances, "r")
    for instance_string in file:
        list_instance = instance_string.split(",")
        start_states.append(State(int(list_instance[0]), int(list_instance[1])))
        goal_states.append(State(int(list_instance[2]), int(list_instance[3])))
        
        solution_costs.append(float(list_instance[4]))
    file.close()
        
    for i in range(0, len(start_states)):   

        start = start_states[i]
        goal = goal_states[i]
    
        cost, expanded_astar = astar(start, goal, gridded_map) # Replace None, None with a call to your implementation of A*
        nodes_expanded_astar.append(expanded_astar)

        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by A* and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()

        cost, expanded_mm = mm(start, goal, gridded_map)
        nodes_expanded_mm.append(expanded_mm)
        
        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by MM and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()

        cost, expanded_biastar = bi_astar(start, goal, gridded_map) # Replace None, None with a call to your implementation of Bi-A*
        nodes_expanded_biastar.append(expanded_biastar)
        
        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by Bi-A* and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()
    
    print('Finished running all tests. The implementation of an algorithm is likely correct if you do not see mismatch messages for it.')

    if plots:
        from search.plot_results import PlotResults
        plotter = PlotResults()
        plotter.plot_results(nodes_expanded_mm, nodes_expanded_astar, "Nodes Expanded (MM)", "Nodes Expanded (A*)", "nodes_expanded_mm_astar")
        plotter.plot_results(nodes_expanded_mm, nodes_expanded_biastar, "Nodes Expanded (MM)", "Nodes Expanded (Bi-A*)", "nodes_expanded_mm_biastar")
        

if __name__ == "__main__":
    main()