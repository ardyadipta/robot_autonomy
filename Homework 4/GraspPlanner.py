import logging, numpy, openravepy
import IPython
import time
import numpy as np
from numpy import inf
#from hw1_grasp import RoboHandler

from HerbRobot import HerbRobot
from HerbEnvironmentRRT import HerbEnvironmentRRT
from HerbEnvironment import HerbEnvironment
from SimpleRobot import SimpleRobot
from SimpleEnvironment import SimpleEnvironment
from RRTConnectPlanner import RRTConnectPlanner
from RRTPlanner import RRTPlanner
from AStarPlanner import AStarPlanner



class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner
        self.env = self.robot.GetEnv()
        self.ikmodel = openravepy.databases.inversereachability.InverseReachabilityModel(robot)
        self.loaded = self.ikmodel.load()





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
		self.graspindices = self.gmodel.graspindices
		self.grasps = self.gmodel.grasps
		if not self.gmodel.load():
			self.gmodel.autogenerate()

		self.grasps=self.gmodel.grasps
		grasps_ordered = self.order_grasps()
		self.irodel=self.ikmodel

		base_pose = None
		grasp_config = None

		IPython.embed()
        ###################################################################
        # TODO: Here you will fill in the function to compute
        #  a base pose and associated grasp config for the
        #  grasping the bottle
        ###################################################################


		timeout = inf
		print("Got a valid grasp")
        # self.show_grasp(Tgrasp,5)

		counter = 0
        # initialize sampling parameters
        
        #timeout = inf
		N = 5
		with self.robot:
            # #alex stuff
            # print("Tgrasp = "+str(Tgrasp))
            # #todo: testGrasp() inside of 1st hw
			gotGrasp = False

			while gotGrasp == False:
				Tgrasp = self.gmodel.getGlobalGraspTransform(self.grasps_ordered[counter],collisionfree=True)

				densityfn,samplerfn,bounds = self.robot.irmodel.computeBaseDistribution(Tgrasp)


				goals = []
				numfailures = 0
				starttime = time.time()

				while len(goals) < N:
					if time.time()-starttime > timeout:
						break
					poses, jointstate = samplerfn(N-len(goals))
					for pose in poses:
						self.robot.SetTransform(pose)
						self.robot.SetDOFValues(*jointstate)

						# theta = 0
						# robot_pose = numpy.array([[numpy.cos(theta), -numpy.sin(theta), 0, -1],
	     #                                          [numpy.sin(theta),  numpy.cos(theta), 0,  0],
	     #                                          [0.              ,  0.              , 1,  0.  ],
	     #                                          [0.              ,  0.              , 0,  1.  ]])
						# self.robot.SetTransform(robot_pose)

	                    #validate that base is not in collision
						if not self.robot.GetManipulator('left_wam').CheckIndependentCollision(openravepy.CollisionReport()):
							q = self.robot.GetManipulator('left_wam').FindIKSolution(Tgrasp,filteroptions=openravepy.IkFilterOptions.CheckEnvCollisions)
							print('q = '+str(q))
							if q is None:
								q = self.robot.GetManipulator('left_wam').FindIKSolution(Tgrasp,0)
								print q
							if q is not None:
								print('if statement taken:')
								values = self.robot.GetDOFValues()
								values_location = self.robot.GetTransform()
								values[self.robot.GetManipulator('left_wam').GetArmIndices()] = q
								goals.append((Tgrasp,values_location,q,pose))
								self.robot.SetActiveDOFValues(q)
								gotGrasp = True
							elif self.robot.GetManipulator('left_wam').FindIKSolution(Tgrasp,0) is None:
								print('elif statement taken:')
								numfailures += 1
						else:
							print ("didn't get nothin")
			base_pose = values_location
			grasp_config = q
			IPython.embed()
		return base_pose, grasp_config



    def PlanToGrasp(self, obj):

        # Next select a pose for the base and an associated ik for the arm
        base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)
        print base_pose
        print 'PlanToGrasp'
        if base_pose is None or grasp_config is None:
            print 'Failed to find solution'
            exit()
        

        # # Now plan to the base pose
        start_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        #base_pose = [-1,0,0]
        IPython.embed()
        base_plan = self.base_planner.Plan(start_pose, base_pose)
        
        base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

        print 'Executing base trajectory'
        self.robot.ExecuteTrajectory(base_traj)
        IPython.embed()
        # Now plan the arm to the grasp configuration
        
        self.robot.SetTransform(base_pose)
        IPython.embed()


        start_config = self.robot.GetActiveDOFValues()
        self.robot.SetActiveDOFValues(grasp_config)

        IPython.embed()
        arm_plan = self.arm_planner.Plan(start_config, grasp_config)
        IPython.embed()

        # Create a trajectory
        traj = openravepy.RaveCreateTrajectory(self.robot.GetEnv(), 'GenericTrajectory')
        config_spec = self.robot.GetActiveConfigurationSpecification()
        traj.Init(config_spec)

        idx = 0
        for pt in arm_plan:
            traj.Insert(idx, pt)
            idx = idx + 1

        openravepy.planningutils.RetimeActiveDOFTrajectory(traj, self.robot, maxvelmult=1, maxaccelmult=1, hastimestamps=False, plannername='ParabolicTrajectoryRetimer')

        arm_traj = traj

        IPython.embed()
        print 'Executing arm trajectory'
        # Send the trajectory to the controller and wait for execution to complete
        self.robot.GetController().SetPath(arm_traj)
        self.robot.WaitForController(0)

        IPython.embed()

        # Grasp the bottle
        task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
        task_manipulation.CloseFingers()




    def order_grasps(self):
		self.grasps_ordered = self.grasps.copy() #you should change the order of self.grasps_ordered
		score_set = np.zeros((len(self.grasps_ordered),3))
		#initialize a martix to hold the scores

		for grasp_no in range(len(self.grasps_ordered)):
			score_set[grasp_no] = self.eval_grasps(self.grasps_ordered[grasp_no])   
      	#find the scores of all the grasps by call the eval_grasp function 
			score_normalized = np.divide(score_set,np.sum(np.transpose(score_set),1))
    	#is same as dividing each value by sum of its columns , is the L1 norm

	   	# score_normalized = np.divide(score_set,np.amax(np.transpose(score_set),1))
	   	# score_normalized = np.divide(score_set,np.maximum(score_set))
		weights_array    = np.array([0.2, 0.6, 0.2])
	    #weights to each score.given more importance to the second one
		self.scoree      = score_normalized
	    #stored the data in a variable to verify . remove while submission

		for grasp_no in range(len(self.grasps_ordered)):
			self.grasps_ordered[grasp_no][self.graspindices.get('performance')] =np.sum(np.multiply(score_normalized[grasp_no],weights_array))
		#    take the bitwise multiplication of the scores with weigths, and add them up. resultant should be one value

	    #set the score to each grasp 
	    # sort!
		order = np.argsort(self.grasps_ordered[:,self.graspindices.get('performance')[0]])
		order = order[::-1]
		self.grasps_ordered = self.grasps_ordered[order]




    def eval_grasps(self,grasp):
		with self.robot:

			#contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
			try:
				contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)

				obj_position = self.gmodel.target.GetTransform()[0:3,3]
		        # for each contact
				G = np.zeros(shape=(len(contacts),6)) #the wrench matrix
				for c in range(len(contacts)):
					pos = contacts[c,0:3] - obj_position
					dire = -contacts[c,3:] #this is already a unit vector
					G[c] = np.array(np.append(dire,np.cross(pos,dire)))

				try:
					eigen_array = np.linalg.svd(G)[1] #could go for one liner, but it means computing svd twice
	      #use a  try staement to handle the svd error that we get in the very few grasps that go bad

					return np.array([np.min(eigen_array[np.nonzero(eigen_array)])/np.max(eigen_array[np.nonzero(eigen_array)]),  np.min(eigen_array[np.nonzero(eigen_array)]), np.prod(eigen_array[np.nonzero(eigen_array)])])  

				except np.linalg.linalg.LinAlgError:
					return np.array([0,0,0])
           #return the zeros if you have an exception
			except openravepy.planning_error,e:
	        #you get here if there is a failure in planning
	        #example: if the hand is already intersecting the object at the initial position/orientation
				return  np.array([0,0,0]) 