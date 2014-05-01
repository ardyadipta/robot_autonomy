import numpy
from RRTTree import RRTTree
import time

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        initial_time = time.time()
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        # append the start configuration
        length = len(start_config)
        if length == 7:
            cfree_config = numpy.array([ 5, 5,  0.00,  5,  0.00,  0.00,  0.00 ])
        else:
            cfree_config = numpy.array([-20.0, 20.0])

        #cfree_config = numpy.array([-20.,20])
        count = 0 
        sub = cfree_config - goal_config
        norm = numpy.linalg.norm(sub)

        while norm > 0.0001:
            # get the random configuration
            if count != 5:
                random_config = self.planning_env.GenerateRandomConfiguration()
            else:
                random_config = goal_config
                count = 0       
            # get the nearest vertex and its id
            nearest_id,nearest_config = tree.GetNearestVertex(random_config)
            #fake_confing = np.array([-20.,20])
            # Extend from the nearest vertex to the random vertex
            cfree_config = self.planning_env.Extend(nearest_config, random_config)
            if cfree_config == None:
                if length == 7:
                    cfree_config = numpy.array([ 5, 5,  0.00,  5,  0.00,  0.00,  0.00 ])
                else:
                    free_config = numpy.array([-20.0, 20.0])
                #cfree_config = numpy.array([-20.,20])
                continue
                count = count +1 
            else:
                # Add vertex
                added_id = tree.AddVertex(cfree_config)
                tree.AddEdge(nearest_id,added_id)
                #Below line should be commented for the herb
                if length != 7:
                    self.planning_env.PlotEdge(nearest_config,cfree_config)
                count = count + 1

            sub = cfree_config - goal_config
            norm = numpy.linalg.norm(sub)


        goal_id = added_id
        start_id = 0 
        plan.append(goal_config)
        while goal_id != start_id:
        
            temp = tree.edges[goal_id]
            plan.append(tree.vertices[temp])
            goal_id = temp
        #plan.append(start_config)

        plan.reverse()
        
        # The algorithm for the path shortening

        plan = self.planning_env.ShortenPath(plan)
        distance = self.planning_env.ComputePathLength(plan)
        num = tree.VerticesLength()
        get_time = time.time()
        plan_time = get_time - initial_time
        print "Plan Time:", plan_time
        print "Path length:", distance
        print "Total number of vertices in the tree:", num

        return plan