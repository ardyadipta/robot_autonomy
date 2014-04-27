import numpy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
import copy
import time


class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)
 
        self.env = self.robot.GetEnv()

        # add an obstacle
        self.table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(self.table)

        table_pose = numpy.array([[ 0, 0, -1, 1.5], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        self.table.SetTransform(table_pose)
        self.p = 0.05

    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        
        return successors

    def ComputeDistance(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        x1 = start_config[0]
        y1 = start_config[1]
        x2 = end_config[0]
        y2 = end_config[1]

        dist  = numpy.sqrt((x1-x2)**2 + (y1-y2)**2) 
        
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        cost = min(numpy.abs(goal_id - start_id)%((self.upper_limits[0]-self.lower_limits[0])/self.discrete_env.resolution), \
        numpy.abs(numpy.abs(goal_id - start_id)%((self.upper_limits[0]-self.lower_limits[0])/self.discrete_env.resolution) - \
        (self.upper_limits[0]-self.lower_limits[0])/self.discrete_env.resolution)) + \
        (numpy.abs(goal_id - start_id)/(int)((self.upper_limits[1]-self.lower_limits[1])/self.discrete_env.resolution))

        return cost

    def Collides(self, point):
        # Show all obstacles in environment
        #if(env.CheckCollision(robot1,robot2)
        x = point[0]
        y = point[1]
        transform = self.robot.GetTransform()
        transform[0][3] = x
        transform[1][3] = y
        self.robot.SetTransform(transform)
        if self.env.CheckCollision(self.robot,self.table) == True:
            return True
        else:
            return False


    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')              
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()
        

    def ComputePathLength(self, plan):      
        dist = 0 
        for i in range(0,len(plan)-1):
            x = self.ComputeDistance(plan[i],plan[i+1])
            dist = dist + x       
        return dist

    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.lower_limits, self.upper_limits
         #
         # TODO: Generate and return a random configuration
         #
        config = [lower_limits[0]+(upper_limits[0]-lower_limits[0])*numpy.random.random(),lower_limits[1]+(upper_limits[1]-lower_limits[1])*numpy.random.random()]
             
        return numpy.array(config)

    def Extend(self, start_config, end_config):
         #
         # TODO: Implement a function which attempts to extend from 
         #   a start configuration to a goal configuration
         #
        x1 = start_config[0]
        y1 = start_config[1]
        x2 = end_config[0]
        y2 = end_config[1]

        numsteps = 20.0
        xdiff = x2 - x1
        ydiff = y2 - y1

        xsteps = range(1, int(numsteps)+1)
        ysteps = range(1, int(numsteps)+1)

        xstep = [x * (xdiff / numsteps) + x1 for x in xsteps]
        ystep = [y * (ydiff / numsteps) + y1 for y in ysteps]

        startT = self.robot.GetTransform()
        T = copy.deepcopy(startT)

        env = self.robot.GetEnv()
        for i in range(int(numsteps)):
             # Transform robot to new position
            T[0, 3] = xstep[i]
            T[1, 3] = ystep[i]
             #print "New Transform: %r" % T

            self.robot.SetTransform(T)
            with self.robot.GetEnv():
                self.robot.SetTransform(T)
             

            for b in self.robot.GetEnv().GetBodies():
                if b.GetName() == self.robot.GetName():
                    continue
                 #print "Body Transform: %r" % b.GetTransform()
                 # Check each body for collision with the robot
                q = self.robot.GetEnv().CheckCollision(b, self.robot)
                if q:
                    self.robot.SetTransform(startT)
                    with self.robot.GetEnv():
                        self.robot.SetTransform(startT)
                    return None
         
        self.robot.SetTransform(T)         
        with self.robot.GetEnv():
            self.robot.SetTransform(T)         
        return end_config


    def ShortenPath(self, path, timeout=5.0):
         
         # 
         # TODO: Implement a function which performs path shortening
         #  on the given path.  Terminate the shortening after the 
         #  given timout (in seconds).
         #
        now = 0         
        while now < timeout:
            now = copy.deepcopy(time.clock() - now)        
            g = copy.deepcopy(path[-1])     
            i = 0 
            while path[i+1].all() != g.all():
                if len(path) > 3:
                    if self.Extend(path[i],path[i+2]) != None:
                        del path[i+1]
                    i = i + 1
                else:
                    break
                if len(path) - i < 2:
                    break    
            now = copy.deepcopy(time.clock() - now)                       
             
        return path    


        
