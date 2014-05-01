import numpy, operator
from RRTPlanner import RRTTree
import time

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        initial_time = time.time()
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        length = len(start_config)

        if length == 7:
            cfreeF_config = numpy.array([ 5, 5,  0.00,  5,  0.00,  0.00,  0.00 ])
            cfreeR_config = numpy.array([ 10, 10,  0.00,  10,  0.00,  0.00,  0.00 ])

        else:
            cfreeF_config = numpy.array([-20.0, 20.0])
            cfreeR_config = numpy.array([-15.,15.])


        sub = cfreeF_config - cfreeR_config
        norm = numpy.linalg.norm(sub)


        while norm > 0.0001:

            # get the random configuration
            randomF_config = self.planning_env.GenerateRandomConfiguration()
            randomR_config = self.planning_env.GenerateRandomConfiguration()

            # get the nearest vertex and its id
            nearestF_id,nearestF_config = ftree.GetNearestVertex(randomF_config)
            nearestR_id,nearestR_config = rtree.GetNearestVertex(randomR_config)

            cfreeF_config = self.planning_env.Extend(nearestF_config, randomF_config)
            cfreeR_config = self.planning_env.Extend(nearestR_config, randomR_config)

            if cfreeF_config != None:
                addedF_id = ftree.AddVertex(cfreeF_config)
                ftree.AddEdge(nearestF_id,addedF_id)
                if length != 7:
                    self.planning_env.PlotEdge(nearestF_config,cfreeF_config)
        
            else:
                if length == 7:
                    cfreeF_config = numpy.array([ 5, 5,  0.00,  5,  0.00,  0.00,  0.00 ])
            
                else:
                    cfreeF_config = numpy.array([-20.0, 20.0])

                continue
                

            if cfreeR_config != None:
                # Add vertex
                addedR_id = rtree.AddVertex(cfreeR_config)
                rtree.AddEdge(nearestR_id,addedR_id)
                if length != 7:
                    self.planning_env.PlotEdge(nearestR_config,cfreeR_config)
            else:
                if length == 7:
                    cfreeR_config = numpy.array([ 5, 5,  0.00,  5,  0.00,  0.00,  0.00 ])
            
                else:
                    cfreeR_config = numpy.array([-20.0, 20.0])

                continue

            randomF_config  = cfreeR_config
            nearestF_id,nearestF_config = ftree.GetNearestVertex(randomF_config)
            cfreeF_config = self.planning_env.Extend(nearestF_config, randomF_config)

            if cfreeF_config != None:
                # Add vertex
                addedF_id = ftree.AddVertex(cfreeF_config)
                ftree.AddEdge(nearestF_id,addedF_id)
                if length != 7:
                    self.planning_env.PlotEdge(nearestF_config,cfreeF_config)
            else:
                if length == 7:
                    cfreeF_config = numpy.array([ 5, 5,  0.00,  5,  0.00,  0.00,  0.00 ])
            
                else:
                    cfreeF_config = numpy.array([-20.0, 20.0])

                continue

            sub = cfreeF_config - cfreeR_config
            norm = numpy.linalg.norm(sub)

                

        goalF_id = addedF_id
        start_id = 0 
        plan.append(cfreeR_config)
        while goalF_id != start_id:
        
            temp = ftree.edges[goalF_id]
            plan.append(ftree.vertices[temp])
            goalF_id = temp
        #plan.append(start_config)

        plan.reverse()
        #**********************************************
        goalR_id = addedR_id
        end_id = 0
        while goalR_id != end_id:
        
            temp = rtree.edges[goalR_id]
            plan.append(rtree.vertices[temp])
            goalR_id = temp
        plan.append(goal_config)
        #**********************************************     
        plan = self.planning_env.ShortenPath(plan)
        distance = self.planning_env.ComputePathLength(plan)
        num1 = ftree.VerticesLength()
        num2 = rtree.VerticesLength()
        num = num1 + num2
        get_time = time.time()
        plan_time = get_time - initial_time
        print "Plan Time:", plan_time
        print "Path length:", distance
        print "Total number of vertices in the tree:", num

        return plan



