import copy
import numpy
import openravepy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
from openravepy import matrixFromAxisAngle
import math
import time

class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

    def __str__(self):
        return "ul = "+str(self.ul)+" ur = "+str(self.ur) + " dt ="+str(self.dt)
    def __repr__(self):
        return "ul = "+str(self.ul)+" ur = "+str(self.ur) + " dt ="+str(self.dt)

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

    def __str__(self):
        return "control = "+str(self.control)+" : footprint = "+str(self.footprint[len(self.footprint)-1])
    def __repr__(self):
        return "control = "+str(self.control)+" : footprint = "+str(self.footprint[len(self.footprint)-1])

class SimpleEnvironment(object):

    def __init__(self, herb, table, resolution):
        self.herb = herb
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5., -numpy.pi], [5., 5., numpy.pi]]
        lower_limits, upper_limits = self.boundary_limits
        self.discrete_env = DiscreteEnvironment(resolution, lower_limits, upper_limits)

        self.table = table
        self.resolution = resolution
        self.ConstructActions()

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
            footprint_config[2] -= start_config[2]
            footprint.append(footprint_config)

            timecount += stepsize

        # Add one more config that snaps the last point in the footprint to the center of the cell
        nid = self.discrete_env.ConfigurationToNodeId(config)
        snapped_config = self.discrete_env.NodeIdToConfiguration(nid)
        snapped_config[:2] -= start_config[:2]
        footprint.append(snapped_config)

        start_id = self.discrete_env.ConfigurationToNodeId(start_config)
        if (nid == start_id):
            return None

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
        grid_coordinate = numpy.array(self.discrete_env.ConfigurationToGridCoord(wc))

        # Iterate through each possible starting orientation
        print("ConstructActions")
        for idx in range(int(self.discrete_env.num_cells[2])):
            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = numpy.array(self.discrete_env.GridCoordToConfiguration(grid_coordinate))

            # alreadyAdded = dict()

            actionSet = list()
            for ul in numpy.arange(-1, 1.01, 2):
                for ur in numpy.arange(-1, 1.01, 2):
                    for dt in numpy.arange(0, 2, 0.05):
                        control = Control(ul, ur, dt)
                        footprint = self.GenerateFootprintFromControl(start_config, control, stepsize = 0.05)
                        # newID = self.discrete_env.ConfigurationToNodeId(footprint[len(footprint)-1])
                        # if (addedStuff)
                        if footprint != None:
                            actionSet.append(Action(control, footprint))

            # if idx == 0:
            #     forward = numpy.array([-1, 1 ,0])
            #     back = numpy.array([1, -1, 0])
            #     left = numpy.array([0, 0, 3])
            #     right = numpy.array([0, 0, 1])
            # elif idx == 1:
            #     forward = numpy.array([1, 1 ,0 ])
            #     back =    numpy.array([-1, -1, 0])
            #     left =    numpy.array([0, 0 ,-1])
            #     right =   numpy.array([0, 0, 1])
            # elif idx == 2:
            #     forward = numpy.array([1, -1 ,0 ])
            #     back =    numpy.array([-1, 1, 0])
            #     left =    numpy.array([0, 0 ,-1])
            #     right =   numpy.array([0, 0, 1])
            # elif idx == 3:
            #     forward = numpy.array([-1, -1 ,0 ])
            #     back =    numpy.array([1, 1, 0])
            #     left =    numpy.array([0, 0 ,-1])
            #     right =   numpy.array([0, 0 ,-3])

            # fcontrol = Control(1,1,1)
            # bcontrol = Control(-1,-1,1)
            # rcontrol = Control(1,-1,1)
            # lcontrol = Control(-1,1,1)

            # # Add forward one...
            # self.actions[idx]
            # ffootprint = [self.discrete_env.GridCoordToConfiguration(forward + grid_coordinate)]
            # if ffootprint[0][2] > numpy.pi:
            #     ffootprint[0][2] -= 2.*numpy.pi
            # if ffootprint[0][2] < -numpy.pi:
            #     ffootprint[0][2] += 2.*numpy.pi

            # bfootprint = [self.discrete_env.GridCoordToConfiguration(back + grid_coordinate)]
            # if bfootprint[0][2] > numpy.pi:
            #     bfootprint[0][2] -= 2.*numpy.pi
            # if bfootprint[0][2] < -numpy.pi:
            #     bfootprint[0][2] += 2.*numpy.pi

            # lfootprint = [self.discrete_env.GridCoordToConfiguration(left + grid_coordinate)]
            # if lfootprint[0][2] > numpy.pi:
            #     lfootprint[0][2] -= 2.*numpy.pi
            # if lfootprint[0][2] < -numpy.pi:
            #     lfootprint[0][2] += 2.*numpy.pi

            # rfootprint = [self.discrete_env.GridCoordToConfiguration(right + grid_coordinate)]
            # if rfootprint[0][2] > numpy.pi:
            #     rfootprint[0][2] -= 2.*numpy.pi
            # if rfootprint[0][2] < -numpy.pi:
            #     rfootprint[0][2] += 2.*numpy.pi


            # self.actions[idx].append(Action(fcontrol,ffootprint))
            # self.actions[idx].append(Action(bcontrol,bfootprint))
            # self.actions[idx].append(Action(rcontrol,rfootprint))
            # self.actions[idx].append(Action(lcontrol,lfootprint))


            self.actions[idx] = actionSet
            print('actions['+str(idx)+" = "+str(self.actions[idx]))
            print("number of actions for config: "+str(start_config)+" = "+str(len(actionSet)))
            # print (str(start_config))
            # TODO: Here you will construct a set of actions
            #  to be used during the planning process

            self.PlotActionFootprints(idx)

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
        with self.herb.env:
            if (theta_cord <0 or theta_cord > 3):
                print("ERROR???")
                return []
            for x in self.actions[theta_cord]:
                # x = the action in the self.actions variable
                for idx in range(len(x.footprint)):
                    currentPosition = startConfig + x.footprint[idx]

                    if (self.Collides(currentPosition)):
                        validTrajectory = False
                        # print("collision...");
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
        with self.herb.env:
            self.robot.SetTransform(transform)
            # time.sleep(0.01)
            if self.herb.env.CheckCollision(self.robot,self.table) == True:
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
        # dist = math.sqrt((end[0] - start[0]) * (end[0] - start[0]) + (end[1] - start[1]) * (end[1] - start[1]) + (end[2] - start[2]) * (end[2] - start[2]))
        dist = math.sqrt((end[0] - start[0])*(end[0] - start[0]) * self.resolution[0]
            + (end[1] - start[1]) * (end[1] - start[1]) * self.resolution[1])
            # + (end[2] - start[2]) * (end[2] - start[2]) * self.resolution[2])
        # dist = abs(end[0] - start[0]) + abs(end[1] - start[1]) + abs(end[2] - start[2])

        return dist

    def ComputeHeuristicCost(self, start_id, end_id):
        # start = self.discrete_env.NodeIdToConfiguration(start_id)
        # end = self.discrete_env.NodeIdToConfiguration(goal_id)
        # dist = 0
        # dist = dist + (end[0] - start[0])
        # dist = dist + (end[1] - start[1])

        start = self.discrete_env.NodeIdToConfiguration(start_id)
        end = self.discrete_env.NodeIdToConfiguration(end_id)

        dist = math.sqrt((end[0] - start[0])*(end[0] - start[0]) * self.resolution[0] \
            + (end[1] - start[1]) * (end[1] - start[1]) * self.resolution[1])
            # + (end[2] - start[2]) * (end[2] - start[2]) * self.resolution[2] * 0.2)
        return dist
