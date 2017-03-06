import time
import numpy

class BreadthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()
        
    def Plan(self, start_config, goal_config):
        
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)

        q = []
        visited = []

        plan = []

        start_node = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_node = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        print start_config, goal_config
        print start_node,goal_node

        q.append(start_node)
        last_node = start_node

        while len(q) is not 0:
            # print "Queue:", q
            # print "Visited: ", visited
            node = q.pop(0)
            visited.append(node)

            for n in self.planning_env.GetSuccessors(node):
                if n in visited:
                    last_node = n
                    break

            
            self.nodes[node] = last_node

            last_node_config = self.planning_env.discrete_env.NodeIdToConfiguration(last_node)
            node_config = self.planning_env.discrete_env.NodeIdToConfiguration(node)
            self.planning_env.PlotEdge(last_node_config,node_config)

            #print "Node popped: ",node
            if node == goal_node:
                print "goal found!"
                curr_node = goal_node

                while curr_node is not start_node:
                    plan.insert(0, self.planning_env.discrete_env.NodeIdToConfiguration(curr_node))
                    curr_node = self.nodes[curr_node]

                plan.insert(0,start_config)
                # for node in plan:
                    # plan.append(self.planning_env.discrete_env.NodeIdToConfiguration(node))
                print "Plan Length: ", len(plan)

                plan = numpy.asarray(plan).reshape(len(plan),-1)
                
                print plan.shape 
                
                return plan

            # if node not in visited:
                
            successors = self.planning_env.GetSuccessors(node)
            #print "Successors: ",successors
            for neighbor in successors:
                if neighbor not in visited:
                    q.append(neighbor)
                    visited.append(neighbor)

            last_node = node

                
                    

            #print "Plan: ",plan
            #print "Visited: ",visited
            #time.sleep(0.5)

        return None
