import argparse
import glob
import random
# python mazeGen.py --instance 100 --agent 10 --size 6

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='generate maze')
    parser.add_argument('--instance', type=str, default=None,
                        help='The number of instance')
    parser.add_argument('--agent', type=int, default=10,
                        help='number of agents on the map')
    parser.add_argument('--size', type=int, default=8,
                        help='the size of the map')

    args = parser.parse_args()


    result_file = open("instances/test_"+args.instance+".txt", "w", buffering=1)
    mapSize = args.size
    maze_map = []
    start_map = []
    goal_map = []
    for i in range(mapSize):
        maze_map.append([0]*mapSize)
        start_map.append([0]*mapSize)
        goal_map.append([0]*mapSize)

    for i in range(mapSize):
        for j in range(mapSize):
            if(random.randint(0,99)<20):
                start_map[i][j] = 1
                goal_map[i][j] = 1
                maze_map[i][j] = 1
    
    result_file.write("{} {}\n".format(mapSize,mapSize))
    for i in range(mapSize):
        row_string = ""
        for j in range(mapSize):
            if(maze_map[i][j] == 0):
                row_string+=". "
            else:
                row_string += "@ "
            
        row_string += "\n"
        result_file.write(row_string)
    agentNum = args.agent
    result_file.write(str(agentNum)+"\n")

    
    agentNum -= 1    
    while(agentNum >= 0):
        valid = False
        startLoc_1 = 0
        startLoc_2 = 0
        while(not valid):
            startLoc_1 = random.randint(0,mapSize-1)
            startLoc_2 = random.randint(0,mapSize-1)
            if(start_map[startLoc_1][startLoc_2] == 0):
                start_map[startLoc_1][startLoc_2] = 1
                valid = True
        
        valid = False
        goalLoc_1 = 0
        goalLoc_2 = 0
        while(not valid):
            goalLoc_1 = random.randint(0,mapSize-1)
            goalLoc_2 = random.randint(0,mapSize-1)
            if(goal_map[goalLoc_1][goalLoc_2] == 0):
                goal_map[goalLoc_1][goalLoc_2] = 1
                valid = True
        result_file.write("{} {} {} {}\n".format(startLoc_1, startLoc_2, goalLoc_1, goalLoc_2))
        agentNum -= 1
        

    # result_file.write("{},{},{},{},{},{},{}\n".format("File","cost","time","expanded nodes", "generated nodes", "maximum nodes", "Initial collision count"))

    
    # for file in sorted(glob.glob(args.instance)):
    #     print("\n\n\n***Import instance {}***".format(file))
    #     my_map, starts, goals = import_mapf_instance(file)
    #     print_mapf_instance(my_map, starts, goals)
    #     result_file.write("{},{},{},{},{},{},{}\n".format(file,"","","", "", "",""))
    #     for i in range(args.repeat):
    #         cbs = CBSSolver(my_map, starts, goals)
    #         (paths, runtime, expanded, generated, maxNode, collisionCount) = cbs.find_solution(args.disjoint)

    #         cost = get_sum_of_cost(paths)
    #         result_file.write("{},{},{},{},{},{},{}\n".format("", cost, runtime, expanded, generated, maxNode, collisionCount))
            
    #         if((not args.batch) and args.repeat==1 ):
    #             print("***Test paths on a simulation***")
    #             animation = Animation(my_map, starts, goals, paths)
    #             # animation.save("output.mp4", 1.0)
    #             animation.show()
    result_file.close()
