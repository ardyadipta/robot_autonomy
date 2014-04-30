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

curr_path = os.getcwd()
relative_ordata = '/models'
ordata_path_thispack = curr_path + relative_ordata

#this sets up the OPENRAVE_DATA environment variable to include the files we're using
openrave_data_path = os.getenv('OPENRAVE_DATA', '')
openrave_data_paths = openrave_data_path.split(':')
if ordata_path_thispack not in openrave_data_paths:
  if openrave_data_path == '':
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
    # time.sleep(3) # wait for viewer to initialize. May be helpful to uncomment
    self.robot = self.env.GetRobots()[0]
    self.manip = self.robot.GetActiveManipulator()
    self.end_effector = self.manip.GetEndEffector()

  # problem specific initialization - load target and grasp module
  def problem_init(self):
   # self.target_kinbody = self.env.ReadKinBodyURI('models/objects/champagne.iv')
   # self.target_kinbody = self.env.ReadKinBodyURI('models/objects/winegoblet.iv')
    self.target_kinbody = self.env.ReadKinBodyURI('models/objects/black_plastic_mug.iv')

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
    score_set = np.zeros((len(self.grasps_ordered),3))
    #initialize a martix to hold the scores

    for grasp_no in range(len(self.grasps_ordered)):
      score_set[grasp_no] = self.eval_grasp(self.grasps_ordered[grasp_no])   
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

  
  # order the grasps - but instead of evaluating the grasp, evaluate random perturbations of the grasp 
  def order_grasps_noisy(self):
    self.grasps_ordered_noisy = self.grasps_ordered.copy() #you should change the order of self.grasps_ordered_noisy

    score_set = np.zeros((len(self.grasps_ordered_noisy),3))
    weight_orig = 1/2.
    weight_rand = 1/8.
    #assigne weights to original grasp and the mean of random sample grasps 
    for grasp_no in range(len(self.grasps_ordered)):
      score_set[grasp_no] = np.multiply(self.eval_grasp(self.grasps_ordered_noisy[grasp_no]),weight_orig)   
      #same as in above case
      for i in range(4):
      #take 4 samples from gaussian distribution
        rand_grasp = self.sample_random_grasp(self.grasps_ordered[grasp_no])
        score_rand = self.eval_grasp(rand_grasp)
        score_set[grasp_no] += np.multiply(score_rand,weight_rand)
        #give weights to them
    score_normalized = np.divide(score_set,np.sum(np.transpose(score_set),1))
    #normalize as above. 

  #  score_normalized = np.divide(score_set,np.amax(np.transpose(score_set),1))
#    score_normalized = np.divide(score_set,np.max(score_set))
    weights_array    = np.array([0.2, 0.6,0.2])
    self.scoree_noisy= score_normalized

    for grasp_no in range(len(self.grasps_ordered_noisy)):
      self.grasps_ordered_noisy[grasp_no][self.graspindices.get('performance')] =np.sum(np.multiply(score_normalized[grasp_no],weights_array))
    #similar to the ordered grasp case
    order = np.argsort(self.grasps_ordered_noisy[:,self.graspindices.get('performance')[0]])
    order = order[::-1]
    self.grasps_ordered_noisy = self.grasps_ordered_noisy[order]
   # print order

  def eval_grasp(self, grasp):
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
        #return value based on one of the metrics, which is sqrt 
      
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
#[-10 -10 -10]                                
      
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
    grasp[self.graspindices['igrasppos']] = pos_orig  #return b
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
  import IPython
  IPython.embed()
  #time.sleep(10000) #to keep the openrave window open

  
