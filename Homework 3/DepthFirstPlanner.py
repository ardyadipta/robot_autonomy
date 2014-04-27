import copy
from DiscreteEnvironment import DiscreteEnvironment
from SimpleEnvironment import SimpleEnvironment
import time
import numpy as np

class DepthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    def Plan(self, start_config, goal_config):

        class node(object):
            State = None
            ParentNode = None
            Operator = None
            Depth = None
            PathCost = None    

            def __init__(self,State,ParentNode,Operator,Depth,PathCost):
                self.State = State
                self.ParentNode = ParentNode
                self.Operator = Operator
                self.Depth = Depth
                self.PathCost = PathCost


        def ConvertToConfig(path):
            config = []
            for i in range(len(path)):
                x = self.planning_env.discrete_env.GridCoordToConfiguration(path[i])
                config.append(x)

            return config

        def Visulaize(plan):
            for i in range(len(plan)-1):
                self.planning_env.PlotEdge(plan[i],plan[i+1])

            return 0.0

        def PlanDetails(plan):
            distance = self.planning_env.ComputePathLength(plan)
            nodes_visited = len(visited)
            print "Path length:", distance
            print "Total number of visited nodes:", nodes_visited

        def give_path():

            print "The solution is found!"
            q_length = len(visited)
            path = []
            soln = node([],None,None,None,None)
            soln = visited[q_length-1]

            for elements in visited:            
                path.append(soln.State)
                soln = soln.ParentNode

                if soln.ParentNode == None:
                    return path
            return path

        def generate_child(v):

            possible_moves = []
            children=[]
            child_nodes = []



            if v[0]>=ll_grid[0]+2:

                left_move= [v[0]-1,v[1]]
                possible_moves.append('left')
                children.append(left_move)


            if v[1]<=ul_grid[1]-2:

                down_move = [v[0],v[1]+1]
                possible_moves.append('down')
                children.append(down_move)

            if v[0]<=ul_grid[0]-2:

                right_move = [v[0]+1,v[1]]
                possible_moves.append('right')
                children.append(right_move)

            if v[1]>=ll_grid[1]+2:
                up_move = [v[0],v[1]-1]
                possible_moves.append('up')
                children.append(up_move)

            child_nodes.append(children)
            child_nodes.append(possible_moves)
            return child_nodes

        plan = []
        visited = []
        queue = []
        visited_states =[]


        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)

        ll0 = self.planning_env.lower_limits[0]
        ll1 = self.planning_env.lower_limits[1]
        ul0 = self.planning_env.upper_limits[0]
        ul1 = self.planning_env.upper_limits[1]
        res = self.planning_env.discrete_env.resolution


        start_grid = self.planning_env.discrete_env.ConfigurationToGridCoord(start_config)
        goal_grid = self.planning_env.discrete_env.ConfigurationToGridCoord(goal_config)

        ll_grid = self.planning_env.discrete_env.ConfigurationToGridCoord([ll0,ll1])
        ul_grid = self.planning_env.discrete_env.ConfigurationToGridCoord([ul0,ul1])

        start_node = node(start_grid,None,None,0,0)

        queue.append(start_node)

        while len(queue)!=0:

            popped_class = queue.pop()
            child_nodes = generate_child(popped_class.State)
            children = child_nodes[0]
            moves = child_nodes[1]

            for b in range(len(children)):
                cond = 0
                if children[b] in visited_states:
                    cond = 1
                    continue
                          
                if cond == 0 and self.planning_env.Collides(self.planning_env.discrete_env.GridCoordToConfiguration(children[b]))== False:
                    #x = self.planning_env.discrete_env.GridCoordToConfiguration(children[b])
                    #self.planning_env.PlotEdge(x,x)
                    queue.append(node(children[b],popped_class,moves[b],(popped_class.Depth)+1,0))
                    visited.append(node(children[b],popped_class,moves[b],(popped_class.Depth)+1,0))
                    visited_states.append(children[b])
                if children[b] == goal_grid:
                    visited.append(node(children[b],popped_class,moves[b],(popped_class.Depth)+1,0))
                    path = give_path()
                    path.reverse()
                    plan = ConvertToConfig(path)
                    plan = np.array(plan)   
                    Visulaize(plan)
                    PlanDetails(plan)
                    return plan
        return plan


