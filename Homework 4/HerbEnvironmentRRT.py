import numpy
import random
import time

class HerbEnvironmentRRT(object):
    
    def __init__(self, herb, table):
        self.robot = herb.robot

        # add a table and move the robot into place
        # self.table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.env = self.robot.GetEnv()
        self.table = table
        self.robot.GetEnv().Add(self.table)

        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
        
        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        

    def GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())

        # TODO: Generate and return a random configuration
        #
        lower_limits,upper_limits = self.robot.GetActiveDOFLimits()

        Deg0 = random.uniform(lower_limits[0], upper_limits[0])
        Deg1 = random.uniform(lower_limits[1], upper_limits[1])
        Deg2 = random.uniform(lower_limits[2], upper_limits[2])
        Deg3 = random.uniform(lower_limits[3], upper_limits[3])
        Deg4 = random.uniform(lower_limits[4], upper_limits[4])
        Deg5 = random.uniform(lower_limits[5], upper_limits[5])
        Deg6 = random.uniform(lower_limits[6], upper_limits[6])

        detection = True 
        while (detection == True):
            Deg0 = random.uniform(lower_limits[0], upper_limits[0])
            Deg1 = random.uniform(lower_limits[1], upper_limits[1])
            Deg2 = random.uniform(lower_limits[2], upper_limits[2])
            Deg3 = random.uniform(lower_limits[3], upper_limits[3])
            Deg4 = random.uniform(lower_limits[4], upper_limits[4])
            Deg5 = random.uniform(lower_limits[5], upper_limits[5])
            Deg6 = random.uniform(lower_limits[6], upper_limits[6])

            detection = self.Collides([Deg0,Deg1,Deg2,Deg3,Deg4,Deg5,Deg6])

        config = [Deg0,Deg1,Deg2,Deg3,Deg4,Deg5,Deg6]

        return numpy.array(config)
        


    
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


    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        a = start_config
        b = end_config
        points = []
        for i in range(1,101):
            step0 = (b[0]-a[0])/100
            step1 = (b[1]-a[1])/100
            step2 = (b[2]-a[2])/100
            step3 = (b[3]-a[3])/100
            step4 = (b[4]-a[4])/100
            step5 = (b[5]-a[5])/100
            step6 = (b[6]-a[6])/100

            points.append([step0*i+a[0],step1*i+a[1],step2*i+a[2],step3*i+a[3],step4*i+a[4],step5*i+a[5],step6*i+a[6]])
        collision_index = 99
        for i in range(0,100):
            if (self.Collides(points[i])):
                collision_index = i
                break
        if collision_index == 0:
            return None
        elif collision_index < 99:
            return numpy.array(points[i-1])
        else:
            return numpy.array(points[len(points)-1]) 
        
    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        initial_time = time.time()
        removed = 1 
        times = 1       
        while removed == 1 and times < timeout:           
            removed = 0
            idx =1 
            while idx < len(path)-1:
                n1 = path[idx-1]
                n2 = path[idx+1]
                if(self.Bs(n1,n2)):
                    removed = 1
                    path[idx] = None
                    path.remove(None)
                else:
                    idx = idx + 1
                get_time = time.time()
                times = get_time - initial_time
        return path


    def Bs(self, start_config, end_config):
        
        a = start_config
        b = end_config
        points = []
        for i in range(1,101):
            step0 = (b[0]-a[0])/100
            step1 = (b[1]-a[1])/100
            step2 = (b[2]-a[2])/100
            step3 = (b[3]-a[3])/100
            step4 = (b[4]-a[4])/100
            step5 = (b[5]-a[5])/100
            step6 = (b[6]-a[6])/100
            points.append([step0*i+a[0],step1*i+a[1],step2*i+a[2],step3*i+a[3],step4*i+a[4],step5*i+a[5],step6*i+a[6]])
        collision_index = 99
        for i in range(0,100):
            if (self.Collides(points[i])):
                collision_index = i
                break
        if collision_index < 99:
            return False
        else:
            return True

    def ComputePathLength(self, plan):      
        dist = 0 
        for i in range(0,len(plan)-1):
            x = self.ComputeDistance(plan[i],plan[i+1])
            dist = dist + x       
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