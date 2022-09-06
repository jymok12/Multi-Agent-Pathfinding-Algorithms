#!/usr/bin/python
# standard command: python run_experiments.py --instance "instances/test_*" --batch --disjoint --repeat 25
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost
from numpy import median


def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals


def getInstanceAverage(instances):
    # result.append({
    #             "cost": cost,
    #             "runtime": runtime,
    #             "nodeExpanded": expanded,
    #             "nodeGenerated": generated,
    #             "memoryUsed": maxNode,
    #             "initialCollision": collisionCount
    #         })
    n = len(instances)
    runtimeSum = 0
    expandedNodeSum = 0
    generatedNodeSum = 0
    memorySum = 0
    collisionSum = 0
    for instance in instances:
        runtimeSum += instance["runtime"]
        expandedNodeSum += instance["nodeExpanded"]
        generatedNodeSum += instance["nodeGenerated"]
        memorySum += instance["memoryUsed"]
        collisionSum += instance["initialCollision"]

    return {
        "runtime": runtimeSum/n,
        "nodeExpanded": expandedNodeSum/n,
        "nodeGenerated": generatedNodeSum/n,
        "memoryUsed": memorySum/n,
        "initialCollision": collisionSum/n
    }

def getInstanceMedian(instances):
    # result.append({
    #             "cost": cost,
    #             "runtime": runtime,
    #             "nodeExpanded": expanded,
    #             "nodeGenerated": generated,
    #             "memoryUsed": maxNode,
    #             "initialCollision": collisionCount
    #         })
    n = len(instances)
    runtimes = []
    expandedNodes = []
    generatedNodes = []
    memorys = []
    collisions = []
    for instance in instances:
        runtimes.append(instance["runtime"])
        expandedNodes.append(instance["nodeExpanded"])
        generatedNodes.append(instance["nodeGenerated"])
        memorys.append(instance["memoryUsed"])
        collisions.append(instance["initialCollision"])

    return {
        "runtime": median(runtimes),
        "nodeExpanded": median(expandedNodes),
        "nodeGenerated": median(generatedNodes),
        "memoryUsed": median(memorys),
        "initialCollision": median(collisions)
    }

def writeInstanceToFile(fileWriter, instanceName, cost, instanceInfo, sampleSize):
    fileWriter.write("{},{},{},{},{},{},{},{}\n".format(
            instanceName,
            cost,
            instanceInfo["runtime"],
            instanceInfo["nodeExpanded"], 
            instanceInfo["nodeGenerated"], 
            instanceInfo["memoryUsed"],
            instanceInfo["initialCollision"],
            sampleSize
        ))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--repeat', type=int, default=1,
                        help='Repeat test case number of time, default 1.')

    args = parser.parse_args()


    average_file = open("resultAverage.csv", "w", buffering=1)
    average_file.write("{},{},{},{},{},{},{},{}\n".format(
        "File",
        "cost",
        "time",
        "expanded nodes", 
        "generated nodes", 
        "maximum nodes", 
        "Initial collision count",
        "sampleSize"))

    median_file = open("resultMedian.csv", "w", buffering=1)
    median_file.write("{},{},{},{},{},{},{},{}\n".format(
        "File",
        "cost",
        "time",
        "expanded nodes", 
        "generated nodes", 
        "maximum nodes", 
        "Initial collision count",
        "sampleSize"))

    result = []
    
    for file in sorted(glob.glob(args.instance)):
        result = []
        print("\n\n\n***Import instance {}***".format(file))
        my_map, starts, goals = import_mapf_instance(file)
        print_mapf_instance(my_map, starts, goals)
        # result_file.write("{},{},{},{},{},{},{}\n".format(file,"","","", "", "",""))
        for i in range(args.repeat):
            cbs = CBSSolver(my_map, starts, goals)
            (paths, runtime, expanded, generated, maxNode, collisionCount) = cbs.find_solution(args.disjoint)

            cost = get_sum_of_cost(paths)
            # result_file.write("{},{},{},{},{},{},{}\n".format("", cost, runtime, expanded, generated, maxNode, collisionCount))
            result.append({
                "cost": cost,
                "runtime": runtime,
                "nodeExpanded": expanded,
                "nodeGenerated": generated,
                "memoryUsed": maxNode,
                "initialCollision": collisionCount
            })
            
            if((not args.batch) and args.repeat==1 ):
                print("***Test paths on a simulation***")
                animation = Animation(my_map, starts, goals, paths)
                # animation.save("output.mp4", 1.0)
                animation.show()
        minCost = 1000
        for instance in result:
            if(instance["cost"]<minCost):
                minCost = instance["cost"]
        result = list(filter(lambda n: n["cost"] == minCost, result))
        averageResult = getInstanceAverage(result)
        medianResult = getInstanceMedian(result)
        writeInstanceToFile(average_file, file, minCost, averageResult, len(result))
        writeInstanceToFile(median_file, file, minCost, medianResult, len(result))

    average_file.close()
