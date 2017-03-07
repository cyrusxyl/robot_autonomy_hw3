import numpy
import time
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.resolution = resolution
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.5], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes

        # For node_id, get 4-connected neighborhood
        # Check for collision here, return only collision-free successors

        coord = self.discrete_env.NodeIdToGridCoord(node_id)

        if numpy.any(coord < 0) or numpy.any(coord > (self.discrete_env.num_cells - numpy.array([1,1]))):
            print "input coord out of bounds"
            return None

        neighbors_coords = [coord - numpy.array([1,0]), coord - numpy.array([0,1]), coord + numpy.array([0,1]), coord + numpy.array([1,0]) ]
        # print neighbors_coords
        orig_T = self.robot.GetTransform()
        for neighbor in neighbors_coords:
            # print neighbor,
            if numpy.any(neighbor < 0) or numpy.any(neighbor > (self.discrete_env.num_cells - numpy.array([1,1]))):
                # print "...invalid neighbor"
                continue

            collision = False
            with self.robot.GetEnv():
                T = self.robot.GetTransform()
                # orig_T = T
                T[0][3] = self.discrete_env.GridCoordToConfiguration(neighbor)[0]
                T[1][3] = self.discrete_env.GridCoordToConfiguration(neighbor)[1]
                self.robot.SetTransform(T)
                

                collision = self.robot.GetEnv().CheckCollision(self.robot.GetEnv().GetRobots()[0])
                if not (collision):
                    
                    successors.append(self.discrete_env.GridCoordToNodeId(neighbor))
                    # print "...added to successors"
                    # print successors
                # else:
                    # print "...collision detected"
                    
                self.robot.SetTransform(orig_T)
                
        # successors.sort()
        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

        ## Distance = Manhattan distance between two coords

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

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        self.cnt = 0
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig, render_interval = 1):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=0.5)
        

        self.cnt += 1
        # render_interval = 100/self.resolution
        # render_interval = 1
        if self.cnt > render_interval:
            
            pl.draw()
            self.cnt = 0

    def PlotEdgePlan(self, sconfig, econfig, color='r'):

        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                color+'.-', linewidth=1.0)
        # pl.draw()


    def ShowPlan(self, plan, color='r'):

        for i in range(len(plan)-1):
            self.PlotEdgePlan(plan[i],plan[i+1], color)
        pl.draw()
        
