#!/usr/bin/env python

PACKAGE_NAME = 'hw1'

# Standard Python Imports
import os
import copy
import time
import math
import numpy as np
np.random.seed(0)
import scipy

# OpenRAVE
import openravepy
#openravepy.RaveInitialize(True, openravepy.DebugLevel.Debug)


curr_path = os.getcwd() #getting the current working directory and store it to curr_path
relative_ordata = '/models' #where the models data of the robot and the object is 
ordata_path_thispack = curr_path + relative_ordata #complete path of the directory of the robot models 


#this sets up the OPENRAVE_DATA environment variable to include the files we're using
openrave_data_path = os.getenv('OPENRAVE_DATA', '') #return the value of the variable if exists. this only returns '' ????
openrave_data_paths = openrave_data_path.split(':') # split it for ?????
if ordata_path_thispack not in openrave_data_paths: # if the path of the models not in the environment, 
  if openrave_data_path == '': #if same with ''. then put 
      os.environ['OPENRAVE_DATA'] = ordata_path_thispack
  else:
      datastr = str('%s:%s'%(ordata_path_thispack, openrave_data_path))
      os.environ['OPENRAVE_DATA'] = datastr

#set database file to be in this folder only
relative_ordatabase = '/database'
ordatabase_path_thispack = curr_path + relative_ordatabase
os.environ['OPENRAVE_DATABASE'] = ordatabase_path_thispack

#get rid of warnings
openravepy.RaveInitialize(True, openravepy.DebugLevel.Fatal)
openravepy.misc.InitOpenRAVELogging()



class RoboHandler:
  def __init__(self):
    self.openrave_init()
    self.problem_init()

    #order grasps based on your own scoring metric
    self.order_grasps()

    #order grasps with noise
    self.order_grasps_noisy()


  # the usual initialization for openrave
  def openrave_init(self):
    self.env = openravepy.Environment()
    self.env.SetViewer('qtcoin')
    self.env.GetViewer().SetName('HW1 Viewer')
    self.env.Load('models/%s.env.xml' %PACKAGE_NAME)
    time.sleep(3) # wait for viewer to initialize. May be helpful to uncomment
    self.robot = self.env.GetRobots()[0]
    self.manip = self.robot.GetActiveManipulator()
    self.end_effector = self.manip.GetEndEffector()

  # problem specific initialization - load target and grasp module
  def problem_init(self):
    self.target_kinbody = self.env.ReadKinBodyURI('models/objects/champagne.iv')
    #self.target_kinbody = self.env.ReadKinBodyURI('models/objects/winegoblet.iv')
    #self.target_kinbody = self.env.ReadKinBodyURI('models/objects/black_plastic_mug.iv')

    #change the location so it's not under the robot
    T = self.target_kinbody.GetTransform()
    T[0:3,3] += np.array([0.5, 0.5, 0.5])
    self.target_kinbody.SetTransform(T)
    self.env.AddKinBody(self.target_kinbody)

    # create a grasping module
    self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, self.target_kinbody)
    
    # if you want to set options, e.g. friction
    options = openravepy.options
    options.friction = 0.1
    if not self.gmodel.load():
      self.gmodel.autogenerate(options)

    self.graspindices = self.gmodel.graspindices
    self.grasps = self.gmodel.grasps

  
  # order the grasps - call eval grasp on each, set the 'performance' index, and sort
  def order_grasps(self):
    self.grasps_ordered = self.grasps.copy() #you should change the order of self.grasps_ordered

    idx = 0
    s1Set = np.zeros(len(self.grasps_ordered))
    s2Set = np.zeros(len(self.grasps_ordered))
    s3Set = np.zeros(len(self.grasps_ordered))

    for grasp in self.grasps_ordered:
      (score1, score2, score3) = self.eval_grasp(grasp)
      s1Set[idx] = score1
      s2Set[idx] = score2
      s3Set[idx] = score3
      idx = idx + 1

    s1Set = s1Set/np.sum(s1Set)
    s2Set = s2Set/np.sum(s2Set)
    s3Set = s3Set/np.sum(s3Set)

    w1 = 1./2.
    w2 = 1./4.
    w3 = 1./4.
    score = w1*s1Set + w2*s2Set + w3*s3Set

    idx = 0
    for grasp in self.grasps_ordered:
      grasp[self.graspindices.get('performance')] = score[idx]
      idx = idx + 1
    
    # sort!
    order = np.argsort(self.grasps_ordered[:,self.graspindices.get('performance')[0]])
    order = order[::-1]
    self.grasps_ordered = self.grasps_ordered[order]

  
  # order the grasps - but instead of evaluating the grasp, evaluate random perturbations of the grasp 
  def order_grasps_noisy(self):
    self.grasps_ordered_noisy = self.grasps_ordered.copy() #you should change the order of self.grasps_ordered_noisy
    #TODO set the score with your evaluation function (over random samples) and sort


  # function to evaluate grasps
  # returns a score, which is some metric of the grasp
  # higher score should be a better grasp
  def eval_grasp(self, grasp):
    with self.robot:
      #contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
      try:
        contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)

        obj_position = self.gmodel.target.GetTransform()[0:3,3]
        # for each contact
        #G = np.array([]) #the wrench matrix
	G = np.zeros((contacts.shape[1],contacts.shape[0]))

	idx = 0;
        for c in contacts:
          pos = c[0:3] - obj_position
          dir = -c[3:] #this is already a unit vector

          #TODO fill G
	  G[:,idx] = np.r_[dir, np.cross(pos,dir)]
	  idx = idx + 1	
        
        #TODO use G to compute scrores as discussed in class
	s= np.linalg.svd(G, full_matrices=True)[1]

	# Use minimum sigma as the score
	# Use min sigma as the score
	score1 = np.min(s[np.nonzero(s)])
	# Use the ratio of min and max sigma as score (dist to 1)
	score2 = math.fabs(np.min(s[np.nonzero(s)])/np.max(s[np.nonzero(s)])-1)
	# Use volume as the score
        score3 = 1;
        for sigma in s[np.nonzero(s)]:
                score3 = score3*sigma 	 

        return (score1, score2, score3) 

      except openravepy.planning_error,e:
        #you get here if there is a failure in planning
        #example: if the hand is already intersecting the object at the initial position/orientation
        return  -1.00 # TODO you may want to change this
      
      #heres an interface in case you want to manipulate things more specifically
      #NOTE for this assignment, your solutions cannot make use of graspingnoise
#      self.robot.SetTransform(np.eye(4)) # have to reset transform in order to remove randomness
#      self.robot.SetDOFValues(grasp[self.graspindices.get('igrasppreshape')], self.manip.GetGripperIndices())
#      self.robot.SetActiveDOFs(self.manip.GetGripperIndices(), self.robot.DOFAffine.X + self.robot.DOFAffine.Y + self.robot.DOFAffine.Z)
#      self.gmodel.grasper = openravepy.interfaces.Grasper(self.robot, friction=self.gmodel.grasper.friction, avoidlinks=[], plannername=None)
#      contacts, finalconfig, mindist, volume = self.gmodel.grasper.Grasp( \
#            direction             = grasp[self.graspindices.get('igraspdir')], \
#            roll                  = grasp[self.graspindices.get('igrasproll')], \
#            position              = grasp[self.graspindices.get('igrasppos')], \
#            standoff              = grasp[self.graspindices.get('igraspstandoff')], \
#            manipulatordirection  = grasp[self.graspindices.get('imanipulatordirection')], \
#            target                = self.target_kinbody, \
#            graspingnoise         = 0.0, \
#            forceclosure          = True, \
#            execute               = False, \
#            outputfinal           = True, \
#            translationstepmult   = None, \
#            finestep              = None )



  # given grasp_in, create a new grasp which is altered randomly
  # you can see the current position and direction of the grasp by:
  # grasp[self.graspindices.get('igrasppos')]
  # grasp[self.graspindices.get('igraspdir')]
  def sample_random_grasp(self, grasp_in):
    grasp = grasp_in.copy()

    #sample random position
    RAND_DIST_SIGMA = 0.01 #TODO you may want to change this
    pos_orig = grasp[self.graspindices['igrasppos']]
    #TODO set a random position
    pos_orig = np.random.normal(pos_orig, RAND_DIST_SIGMA, 3)
    grasp[self.graspindices['igrasppos']] = pos_orig  #return back the value

    #sample random orientation
    RAND_ANGLE_SIGMA = np.pi/24 #TODO you may want to change this
    dir_orig = grasp[self.graspindices['igraspdir']]
    roll_orig = grasp[self.graspindices['igrasproll']]
    #TODO set the direction and roll to be random
    dir_orig = np.random.normal(dir_orig, RAND_DIST_SIGMA, 3) 
    roll_orig = np.random.normal(roll_orig,RAND_DIST_SIGMA, 1)
    grasp[self.graspindices['igraspdir']] = dir_orig  #return back the value of direction of the grasp
    grasp[self.graspindices['igrasproll']] = roll_orig  #return back the value


    return grasp


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

if __name__ == '__main__':
  robo = RoboHandler()
  robo.show_grasp(robo.sample_random_grasp(robo.grasps_ordered[0]))
  #time.sleep(10000) #to keep the openrave window open
  import IPython
  IPython.embed()

  # spin forever
  while True:
    time.sleep(1)
  
