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
        
        plan = []
	visited = []

	plan_config = []

	start_node = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
	goal_node = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

	plan.append(start_node)
	while plan is not empty:
		node = plan.pop()
		if node == goal_node:
			for node in plan:
				plan_config.append(self.planning_env.discrete_env.NodeIdToConfiguration(node))
				return plan_config
		if node is not in visited:
			visited.append(node)
			successors = self.planning_env.GetSuccessors(node)
			for all neighbors in successors:
				plan.append(neighbors)

        return None
