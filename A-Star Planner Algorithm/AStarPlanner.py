import copy

class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()


    def Plan(self, start_config, goal_config):            

        plan = []
        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        #define current,startnode and goalnode as Node objects
        
        current_node    = Node()
        start_node      = Node()
        goal_node       = Node()
        neighbors       = []
        
        #define startnode ID and heuristic cost
        start_node.id= self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_node.id  = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        start_node.h = self.planning_env.ComputeHeuristicCost(start_node.id, goal_node.id)
        start_node.f = start_node.g + start_node.h

        print "Start Node is: " , start_node.id
        print "Goal Node is: ", goal_node.id 

        #define the goalnode id

        #define openset, closed as Hash Table with key: Node address and value: f score
        openset     = dict()
        closedset   = dict()

        #initialize openset with start node
        openset[start_node] = start_node.f
        

        #initialize tentative g_score, which is the g_score of the neighbors
        tentative_g_score = 0
        g_score = 0

        while openset: # check when the openset is not empty
            current_node    = copy.copy(find_key(openset, min(openset.values()))) # get the node with smallest value f
            # print "current node is: " , current_node.id
            plan.append(self.planning_env.discrete_env.NodeIdToConfiguration(current_node.id))
            key_to_delete   = find_key(openset, min(openset.values()))

            id_openset = [] #openset ID to check if the nodes ID is already in the openset, since the keys of the openset dictionary is the node itself
            for i in range(len(openset)):
                id_openset.append(openset.keys()[i].id)
            

            if (current_node.id == goal_node.id):
                plan[0] = start_config
                plan[-1] = goal_config
                print "Plan is: ", plan
                return plan

            else:
                del openset[key_to_delete]
                id_openset = [] #openset ID to check if the nodes ID is already in the openset, since the keys of the openset dictionary is the node itself
                for i in range(len(openset)):
                    id_openset.append(openset.keys()[i].id)


                
                closedset[current_node] = current_node.f
                id_closedset = [] #openset ID to check if the nodes ID is already in the openset, since the keys of the openset dictionary is the node itself
                for i in range(len(closedset)):
                    id_closedset.append(closedset.keys()[i].id)


                # print "openset is: ", openset
                # print "closedset is: ", closedset
                #fill in the nighbors with the function from SimpleEnvironment 
                neighbors = self.planning_env.GetSuccessors(current_node.id)

                #for each neighbor, check whether it is in closed set or not 
                for neighbor in neighbors:
                    if not(neighbor in id_closedset): 
                        # break
                        new_neighbor_node = Node()
                        new_neighbor_node.id = neighbor
                        new_neighbor_node.g = current_node.g + self.planning_env.ComputeDistance(self.planning_env.discrete_env.NodeIdToConfiguration(current_node.id), self.planning_env.discrete_env.NodeIdToConfiguration(new_neighbor_node.id))
                        new_neighbor_node.h = self.planning_env.ComputeHeuristicCost(new_neighbor_node.id, goal_node.id)

                    if not(neighbor in  id_openset) | (new_neighbor_node.g < current_node.g):
                        new_neighbor_node.f = new_neighbor_node.g + new_neighbor_node.h
                        if not (new_neighbor_node.id in id_openset):
                            openset[new_neighbor_node] = new_neighbor_node.f

        return False



class Node(object):
    def __init__(self):
        self.id = None
        self.f = 0 # f score = g + h
        self.g = 0
        self.h = 0 #heuristic distance to goal


# find a key in a dict
def find_key(input_dict, value):
    return next((k for k, v in input_dict.items() if v == value), None)
