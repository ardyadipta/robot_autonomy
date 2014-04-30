import logging, numpy, openravepy
import IPython
import time

from openravepy import *

class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner



    #displays the grasp
    def show_grasp(self, grasp, delay=1.5):
        with openravepy.RobotStateSaver(self.gmodel.robot):
            with self.gmodel.GripperVisibility(self.gmodel.manip):
                time.sleep(0.1) # let viewer update?
                try:
                    with self.env:
                        contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=True)
                        #if mindist == 0:
                        #  print 'grasp is not in force closure!'
                        contactgraph = self.gmodel.drawContacts(contacts) if len(contacts) > 0 else None
                        self.gmodel.robot.GetController().Reset(0)
                        self.gmodel.robot.SetDOFValues(finalconfig[0])
                        self.gmodel.robot.SetTransform(finalconfig[1])
                        self.env.UpdatePublishedBodies()
                        time.sleep(delay)
                except openravepy.planning_error,e:
                    print 'bad grasp!',e

    def GetBasePoseForObjectGrasp(self, obj):

        # Load grasp database
        self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)
        if not self.gmodel.load():
            self.gmodel.autogenerate()

        base_pose = None
        grasp_config = None

        ###################################################################
        # TODO: Here you will fill in the function to compute
        #  a base pose and associated grasp config for the
        #  grasping the bottle
        ###################################################################

        
        print("Got a valid grasp")
        # self.show_grasp(Tgrasp,5)

        counter = 0
        # initialize sampling parameters
        goals = []
        numfailures = 0
        starttime = time.time()
        #timeout = inf
        N = 100
        with self.robot:
            #alex stuff
            Tgrasp = self.gmodel.grasps[counter]
            print("Tgrasp = "+str(Tgrasp))
            #todo: testGrasp() inside of 1st hw
            gotGrasp = False

            while gotGrasp == False:
                try:
                    contacts, final_config, mindist, volume = self.gmodel.testGrasp(grasp=Tgrasp, translate = True, forceclosure=True)
                    print(str(final_config))
                    densityfn,samplerfn,bounds = self.robot.irmodel.computeBaseDistribution(final_config[1])
                    gotGrasp = True
                except openravepy.planning_error,e:
                    print("Exception caught")
                    counter += 1
                    Tgrasp = self.gmodel.grasps[counter]
                    print("Tgrasp = "+str(Tgrasp))

            while len(goals) < N:
                poses, jointstate = samplerfn(N-len(goals))
                for pose in poses:
                    self.robot.SetTransform(pose)
                    self.robot.SetDOFValues(*jointstate)

                    theta = 0.
                    robot_pose = numpy.array([[numpy.cos(theta), -numpy.sin(theta), 0, -0.9],
                                              [numpy.sin(theta),  numpy.cos(theta), 0,  0],
                                              [0.              ,  0.              , 1,  0.  ],
                                              [0.              ,  0.              , 0,  1.  ]])
                    self.robot.SetTransform(robot_pose)

                    # validate that base is not in collision
                    if not self.robot.GetManipulator('left_wam').CheckIndependentCollision(CollisionReport()):
                        q = self.robot.GetManipulator('left_wam').FindIKSolution(final_config[1],filteroptions=IkFilterOptions.CheckEnvCollisions)
                        print('q = '+str(q))
                        if q is None:
                            q = self.robot.GetManipulator('left_wam').FindIKSolution(final_config[1],0)
                            print q
                        if q is not None:
                            print('if statement taken:')
                            values = self.robot.GetDOFValues()
                            values[self.robot.GetManipulator('left_wam').GetArmIndices()] = q
                            goals.append((final_config[1],pose,values))
                        elif self.robot.GetManipulator('left_wam').FindIKSolution(final_config[1],0) is None:
                            print('elif statement taken:')
                            numfailures += 1
                    else:
                        print ("didn't get nothin")
            IPython.embed()

        return base_pose, grasp_config

    def PlanToGrasp(self, obj):

        # Next select a pose for the base and an associated ik for the arm
        base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)
        print base_pose

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

