import Queue

class AStarPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()


    def Plan(self, start_config, goal_config):

        plan = []


        if self.visualize:
            self.planning_env.InitializePlot(goal_config)

        # TODO: Here you will implement the depth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        queue = Queue.PriorityQueue()

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        d_env = self.planning_env.discrete_env
        start_id = d_env.ConfigurationToNodeId(start_config)
        goal_id = d_env.ConfigurationToNodeId(goal_config)

        costs = dict()
        costs[start_id] = 0

        queue.put([self.planning_env.ComputeHeuristicCost(start_id, goal_id), start_id])

        print("Goal id = "+str(goal_id))

        cur_id = start_id
        found_path = False

        num_expanded = 0

        cur_id = queue.get()[1]
        while cur_id != None and found_path == False:

            # print(d_env.NodeIdToConfiguration(cur_id))
            succ = self.planning_env.GetSuccessors(cur_id)
            for new_id in succ:
                num_expanded = num_expanded + 1
                if (costs.get(new_id) is None):
                    #print("Goal id:"+str(goal_id))
                    costs[new_id] = costs[cur_id] + d_env.resolution

                    h = self.planning_env.ComputeHeuristicCost(new_id, goal_id)

                    # print("Heuristic cost for config: " + str(d_env.NodeIdToConfiguration(new_id)) + " is: "+str(h))

                    expectedTotal = h + costs[new_id]

                    queueTuple = [expectedTotal, new_id]
                    queue.put(queueTuple)
                else:
                    if (costs[new_id] > costs[cur_id] + d_env.resolution):
                        costs[new_id] = costs[cur_id] + d_env.resolution

                        h = self.planning_env.ComputeHeuristicCost(new_id, goal_id)

                        # print("Heuristic cost is: "+str(h))

                        expectedTotal = h + costs[new_id]

                        queueTuple = [expectedTotal, new_id]
                        queue.put(queueTuple)


                # print("cur id "+str(d_env.NodeIdToConfiguration(new_id))+" form id "+str(d_env.NodeIdToConfiguration(cur_id)))
                if self.visualize:
                    self.planning_env.PlotEdge(d_env.NodeIdToConfiguration(new_id), d_env.NodeIdToConfiguration(cur_id))

                if (new_id == goal_id):
                    found_path = True
                    print("Found path")

            cur_id = queue.get()[1]

        plan.append(goal_config)
        cur_id = goal_id
        while cur_id != start_id:
            cost = costs[cur_id]
            successors = self.planning_env.GetSuccessors(cur_id)
            # print("successors: "+str(successors))
            for succ in successors:
                # print("succ: "+str(d_env.NodeIdToGridCoord(succ)))
                successorCost = costs.get(succ)
                # print("curr cost = "+str(cost)+", new cost = "+str(successorCost))
                if (successorCost < cost and successorCost != None):
                    # print("Better cost")
                    if self.visualize:
                        self.planning_env.PlotEdge(d_env.NodeIdToConfiguration(succ), d_env.NodeIdToConfiguration(cur_id), 'b')
                    cur_id = succ
                    cost = successorCost

            plan.append(d_env.NodeIdToConfiguration(cur_id))

            # print(d_env.NodeIdToConfiguration(cur_id))

        if self.visualize:
            self.planning_env.ForcePlot()

        plan.append(start_config)

        plan.reverse()

        # f = open('results_wam.txt', 'a')
        # f.write('Nodes Expanded = %d \n' % num_expanded)
        # f.close()

        return plan