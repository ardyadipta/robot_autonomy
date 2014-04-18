from DiscreteEnvironment import DiscreteEnvironment
from SimpleEnvironment import SimpleEnvironment
import math
import numpy
import copy

lower_limits = [-5., -5.]
upper_limits = [5., 5.]
resolution = 0.2
dimension = 2
discrete_env = DiscreteEnvironment(resolution, lower_limits, upper_limits)



def main():
    start_config = [3.1, 3.0]
    goal_config = [4.5, 4.5]

    print "Start Config is: ", start_config
    print "Goal Config is: ", goal_config
    print Plan(start_config, goal_config)

def Plan(start_config, goal_config):

    # plan = []
    # # TODO: Here you will implement the AStar planner
    # #  The return path should be a numpy array
    # #  of dimension k x n where k is the number of waypoints
    # #  and n is the dimension of the robots configuration space
    # #define current,startnode and goalnode as Node objects
    
    # current_node    = Node()
    # start_node      = Node()
    # goal_node       = Node()
    # neighbors       = []
    
    # #define startnode ID and heuristic cost
    # start_node.id= ConfigurationToNodeId(start_config)
    # goal_node.id  = ConfigurationToNodeId(goal_config)
    # start_node.h = ComputeHeuristicCost(start_node.id, goal_node.id)
    # start_node.f = start_node.g + start_node.h

    # print "Start Node is: " , start_node.id
    # print "Goal Node is: ", goal_node.id 

    # #define the goalnode id

    # #define openset, closed as Hash Table with key: Node address and value: f score
    # openset     = dict()
    # closedset   = dict()

    # #initialize openset with start node
    # openset[start_node] = start_node.f
    

    # #initialize tentative g_score, which is the g_score of the neighbors
    # tentative_g_score = 0
    # g_score = 0

    # while openset: # check when the openset is not empty
    #     current_node    = copy.copy(find_key(openset, min(openset.values()))) # get the node with smallest value f
    #     print "current node is: " , current_node.id
    #     plan.append(NodeIdToConfiguration(current_node.id))
    #     key_to_delete   = find_key(openset, min(openset.values()))

    #     id_openset = [] #openset ID to check if the nodes ID is already in the openset, since the keys of the openset dictionary is the node itself
    #     for i in range(len(openset)):
    #         id_openset.append(openset.keys()[i].id)
        

    #     if (current_node.id == goal_node.id):
    #         plan[0] = start_config
    #         plan[-1] = goal_config
    #         print "Plan is: ", plan
    #         return plan

    #     else:
    #         del openset[key_to_delete]
    #         id_openset = [] #openset ID to check if the nodes ID is already in the openset, since the keys of the openset dictionary is the node itself
    #         for i in range(len(openset)):
    #             id_openset.append(openset.keys()[i].id)


            
    #         closedset[current_node] = current_node.f
    #         id_closedset = [] #openset ID to check if the nodes ID is already in the openset, since the keys of the openset dictionary is the node itself
    #         for i in range(len(closedset)):
    #             id_closedset.append(closedset.keys()[i].id)


    #         # print "openset is: ", openset
    #         # print "closedset is: ", closedset
    #         #fill in the nighbors with the function from SimpleEnvironment 
    #         neighbors = GetSuccessors(current_node.id)

    #         #for each neighbor, check whether it is in closed set or not 
    #         for neighbor in neighbors:
    #             if not(neighbor in id_closedset): 
    #                 # break
    #                 new_neighbor_node = Node()
    #                 new_neighbor_node.id = neighbor
    #                 new_neighbor_node.g = current_node.g + ComputeDistance(NodeIdToConfiguration(current_node.id), NodeIdToConfiguration(new_neighbor_node.id))

    #             if not(neighbor in  id_openset) | (new_neighbor_node.g < current_node.g):
    #                 new_neighbor_node.f = new_neighbor_node.g + ComputeHeuristicCost(new_neighbor_node.id, goal_node.id)
    #                 if not (new_neighbor_node.id in id_openset):
    #                     openset[new_neighbor_node] = new_neighbor_node.f

    # return False

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
        start_node.id= ConfigurationToNodeId(start_config)
        goal_node.id  = ConfigurationToNodeId(goal_config)
        start_node.h = ComputeHeuristicCost(start_node.id, goal_node.id)
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
            print "current node is: " , current_node.id
            plan.append(NodeIdToConfiguration(current_node.id))
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
                neighbors = GetSuccessors(current_node.id)

                #for each neighbor, check whether it is in closed set or not 
                for neighbor in neighbors:
                    if not(neighbor in id_closedset): 
                        # break
                        new_neighbor_node = Node()
                        new_neighbor_node.id = neighbor
                        new_neighbor_node.g = current_node.g + ComputeDistance(NodeIdToConfiguration(current_node.id), NodeIdToConfiguration(new_neighbor_node.id))
                        new_neighbor_node.h = ComputeHeuristicCost(new_neighbor_node.id, goal_node.id)

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

def GetSuccessors(node_id):

    successors = []

    # TODO: Here you will implement a function that looks
    #  up the configuration associated with the particular node_id
    #  and return a list of node_ids that represent the neighboring
    #  nodes

    coord = NodeIdToGridCoord(node_id)
    
    #if there are only 2 dimensions, there will be 4 neighbors (up, down, right, left)
    #if there are 3 dimensions, there will be 6 neighbors (up, down, right, left, front, back)
    #hence, the number of neighbors is 2*dimensions
    for i in range(2* discrete_env.dimension):
        if i == 0: #choose left neighbor
            if coord[0] != 0: #is not the most left
                successors.append(node_id - 1)
        elif i == 1: #choose right neighbor
            if coord[0] != (discrete_env.num_cells[0]-1): #is not the most right
                successors.append(node_id + 1)
        elif i == 2: #choose up neighbor
            if (coord[1] != 0) & (node_id - discrete_env.num_cells[1] > 0 ): #is not the top
                successors.append(node_id - discrete_env.num_cells[1])
        elif i == 3: #choose down neighbor
            if coord[1] != (discrete_env.num_cells[1]-1):
                successors.append(node_id + discrete_env.num_cells[1])
    #TODO : For 3 dimensions            
    return successors

def ConfigurationToGridCoord(config):
    
    # TODO:
    # This function maps a configuration in the full configuration space
    # to a grid coordinate in discrete space
    #     
    coord = [0] * 2
    #coord[0], coord[1] = (-1*self.lower_limits[0])/self.resolution+math.floor(config[0]*(1/self.resolution)), (-1*self.lower_limits[1])/self.resolution+math.floor(config[1]*(1/self.resolution))
    for i in range(2):
        coord[i] = (-1*lower_limits[i])/resolution+math.floor(config[i]*(1/resolution))
    return coord

# find a key in a dict
def find_key(input_dict, value):
    return next((k for k, v in input_dict.items() if v == value), None)

def ComputeHeuristicCost(start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        cost = min(numpy.abs(goal_id - start_id)%((upper_limits[0]-lower_limits[0])/resolution), \
        numpy.abs(numpy.abs(goal_id - start_id)%((upper_limits[0]-lower_limits[0])/resolution) - \
        (upper_limits[0]-lower_limits[0])/resolution)) + \
        (numpy.abs(goal_id - start_id)/(int)((upper_limits[1]-lower_limits[1])/resolution))

        return cost

def ConfigurationToNodeId(config):
    
    # TODO:
    # This function maps a node configuration in full configuration
    # space to a node in discrete space
    #
    node_id = 0
    node_id = GridCoordToNodeId(ConfigurationToGridCoord(config))
    return node_id

def GridCoordToConfiguration(coord):
    
    # TODO:
    # This function smaps a grid coordinate in discrete space
    # to a configuration in the full configuration space
    #
    config = [0] * dimension
    #config[0], config[1] = round((coord[0]-((-1*self.lower_limits[0])/self.resolution))/(1/self.resolution) + self.resolution/2,4), round((coord[1]-((-1*self.lower_limits[1])/self.resolution))/(1/self.resolution) + self.resolution/2,4)
    for i in range(dimension):
        config[i] = round((coord[i]-((-1*lower_limits[i])/resolution))/(1/resolution) + resolution/2,4)
    return config

def GridCoordToNodeId(coord):
    
    # TODO:
    # This function maps a grid coordinate to the associated
    # node id 
    node_id = 0
    #node_id = coord[0]%((self.upper_limits[0]-self.lower_limits[0])/self.resolution) + coord[1]*((self.upper_limits[1]-self.lower_limits[1])/self.resolution)
    node_id = coord[0]
    for i in range(1,dimension):
        for j in range(i):
            coord[i] = coord[i]*((upper_limits[j]-lower_limits[j])/resolution)
        node_id = node_id + coord[i]    

    return node_id

def ComputeDistance(start_config, end_config):

    dist = 0

    # TODO: Here you will implement a function that 
    # computes the distance between the configurations given
    # by the two node ids
    x = start_config[0] - end_config[0]
    y = start_config[1] - end_config[1]
    return numpy.sqrt(x**2 + y**2)

    return dist

def NodeIdToConfiguration(nid):
    
    # TODO:
    # This function maps a node in discrete space to a configuraiton
    # in the full configuration space
    #
    config = [0] * dimension
    config = GridCoordToConfiguration(NodeIdToGridCoord(nid))
    return config

def ConfigurationToGridCoord(config):
    
    # TODO:
    # This function maps a configuration in the full configuration space
    # to a grid coordinate in discrete space
    #     
    coord = [0] * dimension
    #coord[0], coord[1] = (-1*self.lower_limits[0])/self.resolution+math.floor(config[0]*(1/self.resolution)), (-1*self.lower_limits[1])/self.resolution+math.floor(config[1]*(1/self.resolution))
    for i in range(dimension):
        coord[i] = (-1*lower_limits[i])/resolution+math.floor(config[i]*(1/resolution))
    return coord


def NodeIdToGridCoord(node_id):
    
    # TODO:
    # This function maps a node id to the associated
    # grid coordinate
    coord = [0] * dimension
    #coord[0], coord[1] = node_id%((self.upper_limits[0]-self.lower_limits[0])/self.resolution), math.floor(node_id/((self.upper_limits[1]-self.lower_limits[1])/self.resolution))
    for i in range(1,dimension):
        c = 1
        for j in range(i,dimension):
            c = c*((upper_limits[j]-lower_limits[j])/resolution)
        coord[dimension - i] = node_id/c
        node_id = node_id%c
    coord[0] = node_id
    return coord

if __name__ == '__main__':
    main()