import numpy, openravepy, time

class SimpleRobot(object):

    def __init__(self, env, robot):
        self.name = 'simple'
        self.robot = robot
        self.wheel_radius = 0.25
        self.wheel_distance = 0.5
        self.max_wheel_velocity = 1.0

        self.env = env

    def GetCurrentConfiguration(self):
        t = self.robot.GetTransform()
        aa = openravepy.axisAngleFromRotationMatrix(t)
        pose = [t[0,3], t[1,3], aa[2]]
        return numpy.array(pose)

    def SetCurrentConfiguration(self, config):

        transform = [[numpy.cos(config[2]), -numpy.sin(config[2]), 0, config[0]],
                     [numpy.sin(config[2]),  numpy.cos(config[2]), 0, config[1]],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]]
        self.robot.SetTransform(transform)

    def ConvertPlanToTrajectory(self, plan):
        # Create a trajectory and insert all points
        return plan

    def ExecuteTrajectory(self, traj, stepsize = 0.01):
        # print("Executing: "+str(traj))
        # Send the trajectory to the controller and wait for execution to complete
        offset = None
        count = 0
        for action in traj:
            print("Action: "+str(count))
            count = count + 1
            config = self.GetCurrentConfiguration()
            print(config)

            for fconfig in action.footprint:
                new_config = fconfig[:]
                new_config[:] += config[:]
                self.SetCurrentConfiguration(new_config)
                time.sleep(0.01)
