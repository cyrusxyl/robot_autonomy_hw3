import numpy
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
    
    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        coord = self.discrete_env.NodeIdToGridCoord(node_id)


        ## this only changes one joint at a time, considering the +/- 1 coord, with the other coords remaining constant
        neighbors = []
        for idx, dof in enumerate(coord):
            
            cfg1 = coord.copy()
            cfg2 = coord.copy()
            cfg1[idx] = cfg1[idx] + 1
            cfg2[idx] = cfg2[idx] - 1

            neighbors.append(cfg1)
            neighbors.append(cfg2)
      
        bodies =  self.robot.GetEnv().GetBodies()
        orig_config = self.robot.GetActiveDOFValues()

        # print neighbors

        for ncoord in neighbors:
            if numpy.any(ncoord < 0) or numpy.any(ncoord >= (self.discrete_env.num_cells)):
                #print "...invalid neighbor"
                continue
            
            config = self.discrete_env.GridCoordToConfiguration(ncoord)
            
            with self.robot.GetEnv():
                self.robot.SetActiveDOFValues(config)
                collision = (self.robot.CheckSelfCollision() or self.robot.GetEnv().CheckCollision(bodies[1],bodies[0]))
                    # self.robot.SetActiveDOFValues(orig_config)
                
                if not collision:
    		#print ncoord,self.discrete_env.GridCoordToNodeId(ncoord)
                    successors.append(self.discrete_env.GridCoordToNodeId(ncoord))
                    # print successors
                else:
                    #print "collision"
                    self.robot.SetActiveDOFValues(orig_config)

        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)
        # start_coord = self.discrete_env.NodeIdToGridCoord(start_id)
        # end_coord = self.discrete_env.NodeIdToGridCoord(end_id)
        
        # dist = sum(abs(start_config-end_config))
        dist = numpy.linalg.norm(start_config-end_config,1)

       
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids

        ## Heuristic = Euclidean Distance between two configs
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(goal_id)

        cost = numpy.linalg.norm(start_config-end_config)

        
        return cost

