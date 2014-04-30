import logging, numpy, openravepy

from openravepy import *

class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner


    def GetBasePoseForObjectGrasp(self, obj):

        # Load grasp database
        gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)
        if not gmodel.load():
            gmodel.autogenerate()

        base_pose = None
        grasp_config = None

        ###################################################################
        # TODO: Here you will fill in the function to compute
        #  a base pose and associated grasp config for the
        #  grasping the bottle
        ###################################################################

        #alex stuff
        Tgrasp = gmodel.grasps[0]
        print("Tgrasp = "+str(Tgrasp))
        #todo: testGrasp() inside of 1st hw
        contacts, final_config, mindist, volume = gmodel.testGrasp(grasp=Tgrasp, translate = True, forceClosure=True)
        densityfn,samplerfn,bounds = robot.irmodel.computeBaseDistribution(final_config)

        # initialize sampling parameters
        goals = []
        numfailures = 0
        starttime = time.time()
        timeout = inf
        N = 1
        with self.robot:
            while len(goals) < N:
                poses, jointstate = samplerfn(N-len(goals))
                for pose in poses:
                    self.robot.SetTransform(pose)
                    self.robot.SetDOFValues(*jointstate)
                    # validate that base is not in collision
                    if not self.manip.CheckIndependentCollision(CollisionReport()):
                        q = self.manip.FindIKSolution(grasp,filteroptions=IkFilterOptions.CheckEnvCollisions)
                        if q is not None:
                            values = self.robot.GetDOFValues()
                            values[self.manip.GetArmIndices()] = q
                            goals.append((grasp,pose,values))
                        elif self.manip.FindIKSolution(grasp,0) is None:
                            numfailures += 1

        return base_pose, grasp_config

    def PlanToGrasp(self, obj):

        # Next select a pose for the base and an associated ik for the arm
        # base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)

        # if base_pose is None or grasp_config is None:
        #     print 'Failed to find solution'
        #     exit()

        # Now plan to the base pose
        start_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        base_pose = [-1,0,0]
        base_plan = self.base_planner.Plan(start_pose, base_pose)
        base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

        print 'Executing base trajectory'
        self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)

        # Now plan the arm to the grasp configuration
        start_config = numpy.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        arm_plan = self.arm_planner.Plan(start_config, grasp_config)
        arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

        print 'Executing arm trajectory'
        self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)

        # Grasp the bottle
        task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
        task_manipultion.CloseFingers()

