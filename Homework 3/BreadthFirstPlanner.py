
class BreadthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        
    def Plan(self, start_config, goal_config):
        
        plan = []

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        ll0 = self.planning_env.lower_limits[0]
        ll1 = self.planning_env.lower_limits[1]
        ul0 = self.planning_env.upper_limits[0]
        ul1 = self.planning_env.upper_limits[1]
        res = self.planning_env.discrete_env.resolution
        graph = []
        count = 0

        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)        

        def moves(nid):
            up = 0
            down = 0
            right = 0
            left = 0
            if nid/((ul0 - ll0)/res) > 0:
                up = 1
            if nid/((ul0 - ll0)/res) < ((ul1 - ll1)/res) - 1:
                down = 1
            if nid%((ul0 - ll0)/res) < ((ul0 - ll0)/res) - 1:
                right = 1
            if nid%((ul0 - ll0)/res) > 0:
                left = 1
            return up, down, right, left    

        class node:
            parent = None
            current = None
            move = None
            g = 0

        n = node()
        not_visited = []
        visited = []
        n.current = start_id
        not_visited.append(n)
        now = []
        del n
        n = node()
        T = self.planning_env.robot.GetTransform()

        while cmp(not_visited[0].current, goal_id) != 0:
            up, down, right, left = moves(not_visited[0].current)
            if up == 1:
                n.current = not_visited[0].current - (ul0 - ll0)/res
                n.parent = not_visited[0].current
                n.g = not_visited[0].g + 1
                not_collision = True
                T[0,3], T[1,3] = self.planning_env.discrete_env.NodeIdToConfiguration(n.current)
                self.planning_env.robot.SetTransform(T)
                for b in self.planning_env.robot.GetEnv().GetBodies():
                    if self.planning_env.robot.GetEnv().CheckCollision(b, self.planning_env.robot) == True:
                        not_collision = False
                        break
                if ((n.current not in now) and not_collision):
                    not_visited.append(n)
                    #x = self.planning_env.discrete_env.NodeIdToConfiguration(n.current)
                    #self.planning_env.PlotEdge(x,x)
                    now.append(n.current)
                del n
                n = node()

            if right == 1:
                n.current = not_visited[0].current + 1
                n.parent = not_visited[0].current
                n.g = not_visited[0].g + 1
                not_collision = True
                T[0,3], T[1,3] = self.planning_env.discrete_env.NodeIdToConfiguration(n.current)
                self.planning_env.robot.SetTransform(T)
                for b in self.planning_env.robot.GetEnv().GetBodies():
                    if self.planning_env.robot.GetEnv().CheckCollision(b, self.planning_env.robot) == True:
                        not_collision = False
                        break
                if ((n.current not in now) and not_collision):
                    not_visited.append(n)
                    #x = self.planning_env.discrete_env.NodeIdToConfiguration(n.current)
                    #self.planning_env.PlotEdge(x,x)
                    now.append(n.current)
                del n
                n = node()

            if left == 1:
                n.current = not_visited[0].current - 1
                n.parent = not_visited[0].current
                n.g = not_visited[0].g + 1
                not_collision = True
                T[0,3], T[1,3] = self.planning_env.discrete_env.NodeIdToConfiguration(n.current)
                self.planning_env.robot.SetTransform(T)
                for b in self.planning_env.robot.GetEnv().GetBodies():
                    if self.planning_env.robot.GetEnv().CheckCollision(b, self.planning_env.robot) == True:
                        not_collision = False
                        break
                if ((n.current not in now) and not_collision):
                    not_visited.append(n)
                    #x = self.planning_env.discrete_env.NodeIdToConfiguration(n.current)
                    #self.planning_env.PlotEdge(x,x)
                    now.append(n.current)
                del n
                n = node()
                
            if down == 1:
                n.current = not_visited[0].current + (ul0 - ll0)/res
                n.parent = not_visited[0].current
                n.g = not_visited[0].g + 1
                not_collision = True
                T[0,3], T[1,3] = self.planning_env.discrete_env.NodeIdToConfiguration(n.current)
                self.planning_env.robot.SetTransform(T)
                for b in self.planning_env.robot.GetEnv().GetBodies():
                    if self.planning_env.robot.GetEnv().CheckCollision(b, self.planning_env.robot) == True:
                        not_collision = False
                        break
                if ((n.current not in now) and not_collision):
                    not_visited.append(n)
                    #x = self.planning_env.discrete_env.NodeIdToConfiguration(n.current)
                    #self.planning_env.PlotEdge(x,x)
                    now.append(n.current)
                del n
                n = node()         

            visited.append(not_visited[0])
            del not_visited[0]

        visited.append(not_visited[0])
        del not_visited[0]

        r = []
        r.append(visited[-1])

        plan.append(self.planning_env.discrete_env.NodeIdToConfiguration(r[-1].current))

        while r[0].g > 0:
            for i in range(len(visited)):
                if r[0].parent == visited[i].current:
                    plan.insert(0,self.planning_env.discrete_env.NodeIdToConfiguration(visited[i].current))
                    r.insert(0,visited[i])
                    break    
                
        for i in range(len(plan)-1):
            self.planning_env.PlotEdge(plan[i],plan[i+1])

        #print "nodes expanded: ", len(visited) 
        #print "path length: ",(len(plan)-1)*res 
        #distance = self.planning_env.ComputePathLength(plan)
        #print "path length:", distance
        return plan
