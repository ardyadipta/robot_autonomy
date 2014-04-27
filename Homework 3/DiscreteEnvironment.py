import numpy
import math
import copy

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = numpy.ceil((upper_limits[idx] - lower_limits[idx])/resolution)


    def ConfigurationToNodeId(self, config):
        
        # TODO:
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        #
        node_id = 0
        node_id = self.GridCoordToNodeId(self.ConfigurationToGridCoord(config))
        return node_id

    def NodeIdToConfiguration(self, nid):
        
        # TODO:
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        config = [0] * self.dimension
        config = self.GridCoordToConfiguration(self.NodeIdToGridCoord(nid))
        return config
        
    def ConfigurationToGridCoord(self, config):
        
        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #     
        coord = [0] * self.dimension
        #coord[0], coord[1] = (-1*self.lower_limits[0])/self.resolution+math.floor(config[0]*(1/self.resolution)), (-1*self.lower_limits[1])/self.resolution+math.floor(config[1]*(1/self.resolution))
        for i in range(self.dimension):
        	coord[i] = round((-1*self.lower_limits[i])/self.resolution)+math.floor(config[i]*(1/self.resolution))
        return coord

    def GridCoordToConfiguration(self, coord):
        
        # TODO:
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        config = [0] * self.dimension
        #config[0], config[1] = round((coord[0]-((-1*self.lower_limits[0])/self.resolution))/(1/self.resolution) + self.resolution/2,4), round((coord[1]-((-1*self.lower_limits[1])/self.resolution))/(1/self.resolution) + self.resolution/2,4)
        for i in range(self.dimension):
        	config[i] = round((coord[i]-((-1*self.lower_limits[i])/self.resolution))/(1/self.resolution) + self.resolution/2,4)
        return config

    def GridCoordToNodeId(self,coord):
        
        # TODO:
        # This function maps a grid coordinate to the associated
        # node id 
        node_id = 0
        #node_id = coord[0]%((self.upper_limits[0]-self.lower_limits[0])/self.resolution) + coord[1]*((self.upper_limits[1]-self.lower_limits[1])/self.resolution)
        node_id = coord[0]
        for i in range(1,self.dimension):
        	for j in range(i):
        		coord[i] = coord[i]*((round(self.upper_limits[j])-round(self.lower_limits[j]))/self.resolution)
        	node_id = node_id + coord[i]	

        return (int)(node_id)

    def NodeIdToGridCoord(self, node_id):
        
        # TODO:
        # This function maps a node id to the associated
        # grid coordinate
        coord = [0] * self.dimension
        #coord[0], coord[1] = node_id%((self.upper_limits[0]-self.lower_limits[0])/self.resolution), math.floor(node_id/((self.upper_limits[1]-self.lower_limits[1])/self.resolution))
        for i in range(1,self.dimension):
        	c = 1
        	for j in range(i,self.dimension):
        		c = c*((self.upper_limits[j]-self.lower_limits[j])/self.resolution)
        	coord[self.dimension - i] = node_id/c
        	node_id = node_id%c
        coord[0] = node_id		


        return coord
        
        
        
