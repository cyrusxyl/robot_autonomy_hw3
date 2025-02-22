import time
import numpy
from collections import deque

class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()
        self.costs = dict()
        self.dist_so_far = dict()


    def Plan(self, start_config, goal_config):
	robotName = self.planning_env.robot.GetName()
	robotNameComp = robotName == u'pr2'
	if (robotNameComp == True):
	    hGain = 1
	else:
	    hGain = 3
	#print hGain	

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)

        # q = deque([])
        q= []
        visited = []
        plan = []
	expanded = []

        start_node = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_node = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)


        self.nodes[start_node] = start_node
        self.costs[start_node] = self.planning_env.ComputeHeuristicCost(start_node, goal_node)
        self.dist_so_far[start_node] = 0
        

        print start_config, goal_config
        print start_node,goal_node
        print "Trying to get to: ", self.planning_env.discrete_env.NodeIdToConfiguration(goal_node)
        print "Lower Limits: ", self.planning_env.lower_limits
        print "Upper Limits  ", self.planning_env.upper_limits
        q.append(start_node)
        visited.append(start_node)
        while len(q) is not 0:


            #print "Queue:", q
            #print "Visited: ", visited

            #sort the queue
            # print [self.costs[i] for i in q] 
            q = sorted(q, key=self.costs.__getitem__, reverse=False)	#sort q based on costs
            node = q.pop(0)


            # print "Curr Node:", node, " Goal Node", goal_node
            # print "Nodes away: ", abs(goal_node - node)
            # print "Heuristic :", hGain*self.planning_env.ComputeHeuristicCost(node,goal_node)
            # print "Path Cost: ", self.dist_so_far[node]


            last_node_config = self.planning_env.discrete_env.NodeIdToConfiguration(self.nodes[node])
            node_config = self.planning_env.discrete_env.NodeIdToConfiguration(node)
	    if self.visualize:
            	self.planning_env.PlotEdge(last_node_config,node_config, 10/self.planning_env.resolution)

            #print "Node popped: ",node
            if node == goal_node:
                print "goal found!"
                curr_node = node

                while curr_node is not start_node:
                    plan.insert(0, self.planning_env.discrete_env.NodeIdToConfiguration(curr_node))
                    #print "Current Node: ", curr_node
                    curr_node = self.nodes[curr_node]

                plan.insert(0,start_config)
                
                print "Plan Length: ", len(plan)*self.planning_env.resolution
		print "Nodes Expanded: ", len(expanded)
	        
	 	if self.visualize:
         	    self.planning_env.ShowPlan(plan)

                plan = numpy.asarray(plan).reshape(len(plan),-1)
                
                print plan.shape
                return plan
                
            successors = self.planning_env.GetSuccessors(node)
            #print "Successors: ",successors
            for neighbor in successors:
                new_dist = self.dist_so_far[node] + self.planning_env.ComputeDistance(node, neighbor)
                # print "new_dist:", new_dist, " dist_so_far: ", self.dist_so_far[node]
                if neighbor not in visited or new_dist < self.dist_so_far[neighbor]:
                    self.dist_so_far[neighbor] = new_dist
                    q.append(neighbor)
                    visited.append(neighbor)

                    ##assign cost = distance + heuristic
                    self.costs[neighbor] = self.dist_so_far[neighbor] + hGain*self.planning_env.ComputeHeuristicCost(neighbor,goal_node)
                    self.nodes[neighbor] = node
    	    expanded.append(node)
            #print "Plan: ",plan
            #print "Visited: ",visited
            #time.sleep(0.5)

        return None


