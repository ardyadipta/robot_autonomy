import Queue

import IPython
import math, numpy

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
        if (self.planning_env.Collides(d_env.NodeIdToConfiguration(goal_id))):
            print("ERROR: goal is in collision. cannot plan")
            return []

        parents = dict()
        costs = dict()
        costs[start_id] = 0

        queue.put([self.planning_env.ComputeHeuristicCost(start_id, goal_id), start_id])

        print("start = "+str(d_env.NodeIdToGridCoord(start_id)) + " => "+str(start_id)+" => "+str(d_env.NodeIdToConfiguration(start_id)))
        print("end = "+str(d_env.NodeIdToGridCoord(goal_id)) + " => "+str(goal_id)+" => "+str(d_env.NodeIdToConfiguration(goal_id)))

        cur_id = start_id
        parents[start_id] = [None, None]
        found_path = False

        num_expanded = 0

        cur_id = queue.get()[1]
        while cur_id != None and found_path == False:
            print("currently at config: "+str(d_env.NodeIdToConfiguration(cur_id)))
            # print(d_env.NodeIdToConfiguration(cur_id))
            succ = self.planning_env.GetSuccessors(cur_id)
            # returns a list of actions
            for action in succ:
                startConfig = d_env.NodeIdToConfiguration(cur_id)
                lastFootprint = action.footprint[len(action.footprint)-1]
                actionEndPoint = [startConfig[0] + lastFootprint[0], startConfig[1] + lastFootprint[1], startConfig[2] + lastFootprint[2]]

                if actionEndPoint[2] > numpy.pi:
                    actionEndPoint[2] -= 2.*numpy.pi
                if actionEndPoint[2] < -numpy.pi:
                    actionEndPoint[2] += 2.*numpy.pi
                # print(lastFootprint)
                # print(actionEndPoint)
                # print("\n")
                # print("Starting at point: "+str(startConfig)+", ending at: "+str(actionEndPoint))
                new_id = d_env.ConfigurationToNodeId(actionEndPoint)
                if (new_id == cur_id):
                    continue

                startGrid = d_env.NodeIdToGridCoord(cur_id)
                endGrid = d_env.NodeIdToGridCoord(new_id)
                # difference = [endGrid[0] - startGrid[0], endGrid[1] - startGrid[1], endGrid[2] - startGrid[2]]
                costToFrom = self.planning_env.ComputeDistance(cur_id, new_id)

                num_expanded = num_expanded + 1
                if (costs.get(new_id) is None):
                    #print("Goal id:"+str(goal_id))
                    costs[new_id] = costs[cur_id] + costToFrom

                    h = self.planning_env.ComputeHeuristicCost(new_id, goal_id) * 50

                    # print("Heuristic cost for config: " + str(d_env.NodeIdToConfiguration(new_id)) + " is: "+str(h))

                    expectedTotal = h + costs[new_id]

                    queueTuple = [expectedTotal, new_id]
                    queue.put(queueTuple)

                    # update parent of new id
                    parents[new_id] = [cur_id, action]
                # else: #check to see if this is a better path to this node???
                #     if (costs[new_id] > costs[cur_id] + costToFrom):
                #         costs[new_id] = costs[cur_id] + costToFrom
                #         # costs[new_id] = costs[cur_id] + abs(d_env.resolution[0] * difference[0]) + abs(d_env.resolution[1] * difference[1])
                #         # costs[new_id] = costs[cur_id] + abs(d_env.resolution[0] * difference[0]) + abs(d_env.resolution[1] * difference[1]) + abs(d_env.resolution[2] * difference[2])

                #         h = self.planning_env.ComputeHeuristicCost(new_id, goal_id) * 50

                #         # print("Heuristic cost is: "+str(h))

                #         expectedTotal = h + costs[new_id]

                #         queueTuple = [expectedTotal, new_id]
                #         queue.put(queueTuple)

                #         # update parent of new id
                #         parents[new_id] = [cur_id, action]

                if (new_id == goal_id):
                    found_path = True
                    print("Found path: "+str(found_path))
                    break


                # print("doing work...")
            # print "moving on"
            if queue.empty() == False:
                cur_id = queue.get()[1]

        print("creating path from end to start")


        import IPython
        IPython.embed()
        plan.append(parents[goal_id][1])
        cur = parents[goal_id][0]
        while cur != start_id:
            #IPython.embed()
            # print("parent of node: "+str(cur) +" = "+str(parents[cur]))

            plan.append(parents[cur][1])
            cur = parents[cur][0]
            # print(d_env.NodeIdToConfiguration(cur))
            if cur == start_id:
                break

        # if self.visualize:
        #     self.planning_env.ForcePlot()

        plan.reverse()
        import IPython
        #IPython.embed()

        for action in plan:
            print(str(action))

        return plan
