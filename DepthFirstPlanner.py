import time

class DepthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    def Plan(self, start_config, goal_config):

        # TODO: Here you will implement the depth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        #plan.append(start_config)
        #plan.append(goal_config)
        
	if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
		self.planning_env.InitializePlot(goal_config)

	plan = []
	visited = []

	plan_config = []

	start_node = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
	goal_node = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

	plan.append(start_node)
	while len(plan) is not 0:
		node = plan.pop()
		if node == goal_node:
			for node in plan:
				plan_config.append(self.planning_env.discrete_env.NodeIdToConfiguration(node))
				return plan_config
		if node not in visited:
			visited.append(node)
			successors = self.planning_env.GetSuccessors(node)
			for neighbor in successors:
				neighbor_config = self.planning_env.discrete_env.NodeIdToConfiguration(neighbor)
				node_config = self.planning_env.discrete_env.NodeIdToConfiguration(node)
				self.planning_env.PlotEdge(neighbor_config,node_config)
				plan.append(neighbor)
				

			print plan
			# time.sleep(0.5)

        return None
