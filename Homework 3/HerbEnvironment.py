import numpy
from DiscreteEnvironment import DiscreteEnvironment
import copy
import time

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)
        self.env = self.robot.GetEnv()
        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        self.table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(self.table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        self.table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
        self.p = 0.5
    
    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        
        return successors



    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        
        return cost

    def ComputeDistance(self, start_config, end_config):
        
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        S0 = start_config[0]
        S1 = start_config[1]
        S2 = start_config[2]
        S3 = start_config[3]
        S4 = start_config[4]
        S5 = start_config[5]
        S6 = start_config[6]

        E0 = end_config[0]
        E1 = end_config[1]
        E2 = end_config[2]
        E3 = end_config[3]
        E4 = end_config[4]
        E5 = end_config[5]
        E6 = end_config[6]


        dist  = numpy.sqrt((S0-E0)**2 + (S1-E1)**2 + (S2-E2)**2 + (S3-E3)**2 + (S4-E4)**2 + (S5-E5)**2 + (S6-E6)**2) 
        
        return dist

    def Collides(self, point):
        # Show all obstacles in environment
        #if(env.CheckCollision(robot1,robot2)
        Deg0 = point[0]
        Deg1 = point[1]
        Deg2 = point[2]
        Deg3 = point[3]
        Deg4 = point[4]
        Deg5 = point[5]
        Deg6 = point[6]

        activeDOFs = self.robot.GetActiveDOFValues()
        activeDOFs[0] = Deg0
        activeDOFs[1] = Deg1
        activeDOFs[2] = Deg2
        activeDOFs[3] = Deg3
        activeDOFs[4] = Deg4
        activeDOFs[5] = Deg5
        activeDOFs[6] = Deg6
        
        self.robot.SetActiveDOFValues(activeDOFs)

        if self.env.CheckCollision(self.robot,self.table) == True or self.robot.CheckSelfCollision() == True:
            return True
        else:
            return False

    def ComputePathLength(self, plan):      
        dist = 0 
        for i in range(0,len(plan)-1):
            x = self.ComputeDistance(plan[i],plan[i+1])
            dist = dist + x       
        return dist

    def GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        for i in range(len(config)):
            config[i] = lower_limits[i]+(upper_limits[i]-lower_limits[i])*numpy.random.random()
        return numpy.array(config)
        
    def Extend(self, start_config, end_config):
        numsteps = 100.0
        dimensions = 7
        steps = numpy.zeros((dimensions, int(numsteps)))
        

        for i in range(dimensions):
            diff = end_config[i] - start_config[i]
            this_dim_steps = range(1, int(numsteps)+1)
            steps[i] = [x * (diff / numsteps) + start_config[i] for x in this_dim_steps]
        joints = self.robot.GetActiveDOFIndices()
        original_values = self.robot.GetDOFValues()
        # steps[i][j], refers to the i-th dimension and the j-th step
        env = self.robot.GetEnv()
        for j in range(int(numsteps)):
            values = steps[:,j]
            self.robot.SetActiveDOFValues(values)
            with self.robot.GetEnv():
                self.robot.SetActiveDOFValues(values)
            #print values    
            for b in self.robot.GetEnv().GetBodies():
                q = self.robot.GetEnv().CheckCollision(b, self.robot)
                #print q
                if q:
                    #print 'collision!'
                    self.robot.SetActiveDOFValues(original_values)
                    with self.robot.GetEnv():
                        self.robot.SetActiveDOFValues(original_values)
                    return None
        self.robot.SetActiveDOFValues(values)       
        with self.robot.GetEnv():
            self.robot.SetActiveDOFValues(values)       
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


