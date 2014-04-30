import copy
import time
import numpy as np
from operator import attrgetter
class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()


    def Plan(self, start_config, goal_config):

        plan = []
        visited = []
        queue = []
        visited_states =[]

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)

        dim = len(start_config)

        if dim == 2:

            ll0 = self.planning_env.lower_limits[0]
            ll1 = self.planning_env.lower_limits[1]
            ul0 = self.planning_env.upper_limits[0]
            ul1 = self.planning_env.upper_limits[1]
            res = self.planning_env.discrete_env.resolution

        else:

            ll0 = self.planning_env.lower_limits[0]
            ll1 = self.planning_env.lower_limits[1]
            ll2 = self.planning_env.lower_limits[2]
            ll3 = self.planning_env.lower_limits[3]
            ll4 = self.planning_env.lower_limits[4]
            ll5 = self.planning_env.lower_limits[5]
            ll6 = self.planning_env.lower_limits[6]

            ul0 = self.planning_env.upper_limits[0]
            ul1 = self.planning_env.upper_limits[1]
            ul2 = self.planning_env.upper_limits[2]
            ul3 = self.planning_env.upper_limits[3]
            ul4 = self.planning_env.upper_limits[4]
            ul5 = self.planning_env.upper_limits[5]
            ul6 = self.planning_env.upper_limits[6]

            res = self.planning_env.discrete_env.resolution


        class node(object):
            State = None
            ParentNode = None
            Operator = None
            Depth = None
            PathCost = None 
            Hcost = None   

            def __init__(self,State,ParentNode,Operator,Depth,PathCost,Hcost):
                self.State = State
                self.ParentNode = ParentNode
                self.Operator = Operator
                self.Depth = Depth
                self.PathCost = PathCost
                self.Hcost = Hcost

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
            print "path length2: ",(len(plan)-1)*res 

        def give_path():

            print "The solution is found!"
            q_length = len(visited)
            path = []
            soln = node([],None,None,None,None,None)
            soln = copy.deepcopy(visited[q_length-1])

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

            if dim == 2:

                if v[1]>=ll_grid[1]+2:
                    up_move = [v[0],v[1]-1]
                    possible_moves.append('up')
                    children.append(up_move)

                if v[0]<=ul_grid[0]-2:

                    right_move = [v[0]+1,v[1]]
                    possible_moves.append('right')
                    children.append(right_move)

                if v[0]>=ll_grid[0]+2:

                    left_move= [v[0]-1,v[1]]
                    possible_moves.append('left')
                    children.append(left_move)

                if v[1]<=ul_grid[1]-2:

                    down_move = [v[0],v[1]+1]
                    possible_moves.append('down')
                    children.append(down_move)

                child_nodes.append(children)
                child_nodes.append(possible_moves)
                return child_nodes

            else:

                if v[0]>=ll_grid[0]+2:
                    move01 = [v[0]-1,v[1],v[2],v[3],v[4],v[5],v[6]]
                    possible_moves.append(0.1)
                    children.append(move01)

                if v[0]<=ul_grid[0]-2:
                    move02 = [v[0]+1,v[1],v[2],v[3],v[4],v[5],v[6]]
                    possible_moves.append(0.2)
                    children.append(move02)

                if v[1]>=ll_grid[1]+2:
                    move11 = [v[0],v[1]-1,v[2],v[3],v[4],v[5],v[6]]
                    possible_moves.append(1.1)
                    children.append(move11)

                if v[1]<=ul_grid[1]-2:
                    move12 = [v[0],v[1]+1,v[2],v[3],v[4],v[5],v[6]]
                    possible_moves.append(1.2)
                    children.append(move12)

                if v[2]>=ll_grid[2]+2:
                    move21 = [v[0],v[1],v[2]-1,v[3],v[4],v[5],v[6]]
                    possible_moves.append(2.1)
                    children.append(move21)

                if v[2]<=ul_grid[2]-2:
                    move22 = [v[0],v[1],v[2]+1,v[3],v[4],v[5],v[6]]
                    possible_moves.append(2.2)
                    children.append(move22)

                if v[3]>=ll_grid[3]+2:
                    move31 = [v[0],v[1],v[2],v[3]-1,v[4],v[5],v[6]]
                    possible_moves.append(3.1)
                    children.append(move31)

                if v[3]<=ul_grid[3]-2:
                    move32 = [v[0],v[1],v[2],v[3]+1,v[4],v[5],v[6]]
                    possible_moves.append(3.2)
                    children.append(move32)

                if v[4]>=ll_grid[4]+2:
                    move41 = [v[0],v[1],v[2],v[3],v[4]-1,v[5],v[6]]
                    possible_moves.append(4.1)
                    children.append(move41)

                if v[4]<=ul_grid[4]-2:
                    move42 = [v[0],v[1],v[2],v[3],v[4]+1,v[5],v[6]]
                    possible_moves.append(4.2)
                    children.append(move42)

                if v[5]>=ll_grid[5]+2:
                    move51 = [v[0],v[1],v[2],v[3],v[4],v[5]-1,v[6]]
                    possible_moves.append(5.1)
                    children.append(move51)

                if v[5]<=ul_grid[5]-2:
                    move52 = [v[0],v[1],v[2],v[3],v[4],v[5]+1,v[6]]
                    possible_moves.append(5.2)
                    children.append(move52)

                if v[6]>=ll_grid[6]+2:
                    move61 = [v[0],v[1],v[2],v[3],v[4],v[5],v[6]-1]
                    possible_moves.append(6.1)
                    children.append(move61)

                if v[6]<=ul_grid[6]-2:
                    move62 = [v[0],v[1],v[2],v[3],v[4],v[5],v[6]+1]
                    possible_moves.append(6.2)
                    children.append(move62)

                child_nodes.append(children)
                child_nodes.append(possible_moves)
                return child_nodes




        def compute_manhattan_distance(x,y):
            
            if dim == 2:
                h = abs(x[0]-y[0])+abs(x[1]-y[1])
                return h 
            else:
                h = abs(x[0]-y[0])+abs(x[1]-y[1])+abs(x[2]-y[2])+abs(x[3]-y[3])+abs(x[4]-y[4])+abs(x[5]-y[5])+abs(x[6]-y[6])
                return h 
            '''
            h = self.planning_env.ComputeDistance(start_config,end_config)
            return h
            '''


        start_grid = self.planning_env.discrete_env.ConfigurationToGridCoord(start_config)
        goal_grid = self.planning_env.discrete_env.ConfigurationToGridCoord(goal_config)

        if dim == 2:
            ll_grid = self.planning_env.discrete_env.ConfigurationToGridCoord([ll0,ll1])
            ul_grid = self.planning_env.discrete_env.ConfigurationToGridCoord([ul0,ul1])
        else:
            ll_grid = self.planning_env.discrete_env.ConfigurationToGridCoord([ll0,ll1,ll2,ll3,ll4,ll5,ll6])
            ul_grid = self.planning_env.discrete_env.ConfigurationToGridCoord([ul0,ul1,ul2,ul3,ul4,ul5,ul6])


        start_node = node(start_grid,None,None,0,0,0)
        print "start grid: ", start_grid
        print "goal grid: ", goal_grid
        print "start node: ", start_node
        start_node.PathCost = compute_manhattan_distance(start_grid,goal_grid)
        start_node.Hcost = compute_manhattan_distance(start_grid,goal_grid)
        queue.append(start_node)

        while len(queue)!=0:

            popped_class = queue.pop()
            print "current node: ", popped_class.State
            child_nodes = generate_child(popped_class.State)
            # print "child nodes: ", child_nodes
            children = child_nodes[0]
            print "children: ", children
            moves = child_nodes[1]
            # print "moves: ", moves

            for b in range(len(children)):
                cond = 0
 
                if children[b] in visited_states:
                   cond = 1
                   continue

                if self.planning_env.Collides(self.planning_env.discrete_env.GridCoordToConfiguration(children[b])):
                    print "children[b]: ", children[b], " Collides"
                                  
                if cond == 0 and self.planning_env.Collides(self.planning_env.discrete_env.GridCoordToConfiguration(children[b]))== False:
                    h = compute_manhattan_distance(children[b],goal_grid)
                    #g = compute_manhattan_distance(popped_class.State,children[b]) + popped_class.PathCost - compute_manhattan_distance(popped_class.State,goal_grid)
                    g = 1 + popped_class.PathCost - compute_manhattan_distance(popped_class.State,goal_grid)
                    #g = popped_class.Depth + 1
                    #x = self.planning_env.discrete_env.GridCoordToConfiguration(children[b])
                    #self.planning_env.PlotEdge(x,x)
                    queue.append(node(children[b],popped_class,moves[b],(popped_class.Depth)+1,(h+g),h))
                    visited.append(node(children[b],popped_class,moves[b],(popped_class.Depth)+1,(h+g),h))
                    visited_states.append(children[b])
                if children[b] == goal_grid:
                    visited.append(node(children[b],popped_class,moves[b],(popped_class.Depth)+1,(h+g),h))
                    path = give_path()
                    path.reverse()
                    plan = ConvertToConfig(path)
                    plan = np.array(plan)   
                    if dim ==2:
                        Visulaize(plan)
                    PlanDetails(plan)
                    print plan[len(plan)-1]
                    return plan
            queue_sorted = sorted(queue, key=attrgetter('Hcost','PathCost'),reverse=True)
            queue = copy.deepcopy(queue_sorted)





