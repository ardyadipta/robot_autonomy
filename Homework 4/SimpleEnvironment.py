import copy
import numpy, openravepy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
from openravepy import matrixFromAxisAngle

class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

class SimpleEnvironment(object):

    def __init__(self, herb, resolution):
        self.herb = herb
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5., -numpy.pi], [5., 5., numpy.pi]]
        lower_limits, upper_limits = self.boundary_limits
        self.discrete_env = DiscreteEnvironment(resolution, lower_limits, upper_limits)

        self.resolution = resolution
        self.ConstructActions()

        self.env = openravepy.Environment()

    def GenerateFootprintFromControl(self, start_config, control, stepsize=0.01):

        # Extract the elements of the control
        ul = control.ul
        ur = control.ur
        dt = control.dt

        # Initialize the footprint
        config = start_config.copy()
        footprint = [numpy.array([0., 0., config[2]])]
        timecount = 0.0
        while timecount < dt:
            # Generate the velocities based on the forward model
            xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.cos(config[2])
            ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.sin(config[2])
            tdot = self.herb.wheel_radius * (ul - ur) / self.herb.wheel_distance

            # Feed forward the velocities
            if timecount + stepsize > dt:
                stepsize = dt - timecount
            config = config + stepsize*numpy.array([xdot, ydot, tdot])
            if config[2] > numpy.pi:
                config[2] -= 2.*numpy.pi
            if config[2] < -numpy.pi:
                config[2] += 2.*numpy.pi

            footprint_config = config.copy()
            footprint_config[:2] -= start_config[:2]
            footprint.append(footprint_config)

            timecount += stepsize

        # Add one more config that snaps the last point in the footprint to the center of the cell
        nid = self.discrete_env.ConfigurationToNodeId(config)
        snapped_config = self.discrete_env.NodeIdToConfiguration(nid)
        snapped_config[:2] -= start_config[:2]
        footprint.append(snapped_config)

        return footprint

    def PlotActionFootprints(self, idx):

        actions = self.actions[idx]
        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])

        for action in actions:
            xpoints = [config[0] for config in action.footprint]
            ypoints = [config[1] for config in action.footprint]
            pl.plot(xpoints, ypoints, 'k')

        pl.ion()
        pl.show()


    def ConstructActions(self):
        # Actions is a dictionary that maps orientation of the robot to
        #  an action set
        self.actions = dict()

        wc = [0., 0., 0.]
        grid_coordinate = self.discrete_env.ConfigurationToGridCoord(wc)

        # Iterate through each possible starting orientation
        print("ConstructActions")
        for idx in range(int(self.discrete_env.num_cells[2])):
            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = numpy.array(self.discrete_env.GridCoordToConfiguration(grid_coordinate))

            addedStuff = dict()

            actionSet = list()
            for ul in numpy.arange(-1, 1, 0.25):
                for ur in numpy.arange(-1, 1, 0.25):
                    for dt in numpy.arange(0.5, 5, 0.5):
                        control = Control(ul, ur, dt)
                        footprint = self.GenerateFootprintFromControl(start_config, control)
                        # newID = self.discrete_env.ConfigurationToNodeId(footprint[len(footprint)-1])
                        # if (addedStuff)
                        actionSet.append(Action(control, footprint))

            self.actions[idx] = actionSet
            print("number of actions for config: "+str(start_config)+" = "+str(len(actionSet)))
            # TODO: Here you will construct a set of actions
            #  to be used during the planning process
            #
        return


    def GetSuccessors(self, node_id):

        successors = list()

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes
        grid_coord = self.discrete_env.NodeIdToGridCoord(node_id)
        startConfig = self.discrete_env.GridCoordToConfiguration(grid_coord)
        theta_cord = grid_coord[2]

        validTrajectory = True
        for x in self.actions[theta_cord]:
            # x = the action in the self.actions variable
            for idx in range(len(x.footprint)):
                currentPosition = startConfig + x.footprint[idx]

                if (self.Collides(currentPosition)):
                    validTrajectory = False
                    print("collision...");
                    break

            if validTrajectory == True:
                successors.append(x)

        return successors


    def Collides(self, config):
        x = config[0]
        y = config[1]
        theta = config[2]

        transform = matrixFromAxisAngle([0,0,theta])
        transform[0][3] = x
        transform[1][3] = y

        # Assigns Transform to Robot and Checks Collision
        with self.env:
            for body in self.env.GetBodies():
                self.robot.SetTransform(transform)
                if self.robot.GetEnv().CheckCollision(self.robot,body) == True:
                    return True
                else:
                    return False

    def ComputeDistance(self, start_id, end_id):

        # This is a function that
        # computes the distance between the configurations given
        # by the two node ids
        start = self.discrete_env.NodeIdToConfiguration(start_id)
        end = self.discrete_env.NodeIdToConfiguration(end_id)

        # print("Current id is: "+str(start_id)+" Goal id is: "+str(end_id))
        # print("Current is: "+str(start)+" Goal is: "+str(end))

        # Manhattan distance
        dist = abs(end[0] - start[0]) + abs(end[1] - start[1]) + abs(end[2] - start[2])
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        dist = self.ComputeDistance(start_id, goal_id)

        return dist

