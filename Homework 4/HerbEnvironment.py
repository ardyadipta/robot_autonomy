import numpy
import IPython
import time
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):

    def __init__(self, herb, table, resolution):

        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        # self.table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')

        # self.robot.GetEnv().Add(self.table)

        # table_pose = numpy.array([[ 0, 0, -1, 0.7],
        #                           [-1, 0,  0, 0],
        #                           [ 0, 1,  0, 0],
        #                           [ 0, 0,  0, 1

        
        # self.table.SetTransform(table_pose)

        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)

        self.table = table
        self.p = 0

    def GetSuccessors(self, node_id):

        successors = []
        #  This function looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        env = self.discrete_env
        coords = env.NodeIdToGridCoord(node_id)
        # print("coordinates for node id ("+str(node_id)+") = "+str(coords))

        # Iterate over dimensions, adding and subtracting one grid per dimension
        for x in xrange(0,self.discrete_env.dimension):
            # Slice copy to not alter coords
            newCoord = coords[:]

            # Subtraction neighbor
            newCoord[x] = coords[x] - 1
            newID = env.GridCoordToNodeId(newCoord)
            if (self.CheckCollisions(newID) == False and self.InBounds(newCoord) == True):
                successors.append(newID)

            # Addition Neighbor
            newCoord[x] = coords[x] + 1
            newID = env.GridCoordToNodeId(newCoord)
            if (self.CheckCollisions(newID) == False and self.InBounds(newCoord) == True):
                successors.append(newID)

        return successors

    def CheckCollisions(self, node_id):

        config = self.discrete_env.NodeIdToConfiguration(node_id)

        # Put robot into configuration
        self.robot.SetActiveDOFValues(config)

        # Check collision with table
        if self.robot.GetEnv().CheckCollision(self.robot, self.table) == True or self.robot.CheckSelfCollision() == True:
            return True
        else:
            return False



    def InBounds(self, coord):

        config = self.discrete_env.GridCoordToConfiguration(coord)

        for x in xrange(0, self.discrete_env.dimension):
            # Check limits of space for each dimension
            if not(config[x] < self.upper_limits[x]-0.0005 and config[x] > self.lower_limits[x]+0.0005):
                return False
        return True


    def ComputeDistance(self, start_id, end_id):
        # computes the distance between the configurations given
        # by the two node ids

        start_grid = self.discrete_env.NodeIdToGridCoord(start_id)
        end_grid = self.discrete_env.NodeIdToGridCoord(end_id)

        dist = 0
        for x in xrange(0, self.discrete_env.dimension):
            dist = dist + (abs(end_grid[x] - start_grid[x]) * self.discrete_env.resolution[x])

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):

        cost = self.ComputeDistance(start_id, goal_id)

        return cost

    def ShortenPath(self, path, timeout=5.0):

        #
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the
        #  given timout (in seconds).
        #
        t0 = time.time()
        idx = 0
        print "Shortening Path (original length: %d" % len(path)
        while idx < len(path)-1 and time.time() - t0 < timeout:
            # Check backwrds from goal
            for ridx in xrange(len(path)-1, idx, -1):
                if self.Extend(path[idx], path[ridx]) != None:

                    dist_ab = self.ComputeSomeDistance(path[idx], path[ridx])
                    dist_path_slice = self.ComputePathSliceLength(path, idx, ridx)
                    # If distance between two points is less than distance along path, slice out inbetween
                    if dist_ab < dist_path_slice:
                        # Remove all inbetween if not next to each other
                        # And done
                        if (ridx - idx+1 > 0):
                            path[idx+1:ridx] = []
                            break
            idx += 1
        print "Shorter Path (length: %d" % len(path)

        return path

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p


    def GenerateRandomConfiguration(self):
        if (numpy.random.random() < self.p):
            return numpy.array(self.goal_config)

        config = [0] * len(self.robot.GetActiveDOFIndices())

        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()

        for i in xrange(len(config)):
            config[i] = lower_limits[i] + numpy.random.random()*(upper_limits[i] - lower_limits[i]);
        #
        # TODO: Generate and return a random configuration
        #
        return numpy.array(config)

    def ComputeSomeDistance(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        return self.getDistance(end_config-start_config)

    def getDistance(self, offset):
        return numpy.sqrt(sum(offset**2))


    def Extend(self, start_config, end_config):

        #
        # TODO: Implement a function which attempts to extend from
        #   a start configuration to a goal configuration
        #
        dist = self.ComputeSomeDistance(start_config, end_config)
        MAX_STEP_SIZE = 0.05

        # If two points are so close vs step size, it can make it
        if (dist < MAX_STEP_SIZE):
            return end_config

        num_steps = round(dist/MAX_STEP_SIZE)
        path = numpy.array(map(lambda x: numpy.linspace(x[0], x[1], num_steps), zip(start_config,end_config)))

        #IPython.embed()
        for p in path.transpose():
            # print(p)
            if self.checkConfigurationCollision(p):
                return None

        # Return last element in path
        return path.transpose()[-1]

    def ComputePathSliceLength(self, path, a_idx, b_idx):
        dist_path_slice = 0
        for x in xrange(a_idx,b_idx):
            dist_path_slice += self.ComputeSomeDistance(numpy.array(path[x]), numpy.array(path[x+1]))
        return dist_path_slice

    def checkConfigurationCollision(self, config):
        # This could be extended to N bodies check using loops, quadtrees etc.
        # But this has been solved in openrave etc. so I'm just hardcoding it here.
        # IPython.embed()
        self.robot.SetJointValues(config, self.robot.GetActiveDOFIndices())
        return self.robot.GetEnv().CheckCollision(self.robot, self.table) or self.robot.CheckSelfCollision()