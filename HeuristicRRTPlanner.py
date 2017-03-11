import numpy
import time
from RRTTree import RRTTree

class HeuristicRRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        p_min = 0.1
        C_max = 0

        while(1):           
            #import IPython
            #IPython.embed()
            #drop one valid random point that only connect with start tree
            p = 0

            while numpy.random.uniform(0,1) > p:
                point_drop = self.planning_env.GenerateRandomConfiguration()
                index, near_point = tree.GetNearestVertex(point_drop)
                
                # print near_point
                curr_path = self.find_path(tree,0,index)
                curr_cost = self.planning_env.ComputePathLength(curr_path)
                curr_h = self.planning_env.ComputeHeuristicC(near_point,goal_config)
                
                C_q = curr_cost + 5*curr_h
                C_max =  max(C_q, C_max)
                C_opt = self.planning_env.ComputeDistanceC(start_config, goal_config)
                # print C_max, C_opt
                try:
                    mq = 1 - float(C_q - C_opt)/float(C_max - C_opt)
                except:
                    mq = 0
                p = max(mq, p_min)



            
            point_chosen_from_s = self.planning_env.Extend(tree.vertices[index],point_drop)
            point_chosen_from_g = self.planning_env.Extend(goal_config,point_drop)

            #Check whether the point_drop can be directly used for connecting both

            if(point_chosen_from_s == None or point_chosen_from_g == None): continue

            dist_s = self.planning_env.ComputeDistanceC(point_chosen_from_s, point_drop)
            dist_g = self.planning_env.ComputeDistanceC(point_chosen_from_g,point_drop)

            if (dist_s <= epsilon and dist_g <= epsilon):
                point_addon_s_final = tree.AddVertex(point_chosen_from_s)
                tree.AddEdge(index,point_addon_s_final)
                point_addon_g_final = tree.AddVertex(point_chosen_from_g)
                tree.AddEdge(point_addon_s_final,point_addon_g_final)
                #self.planning_env.PlotEdge(near_point,point_chosen_from_s)
                #self.planning_env.PlotEdge(point_chosen_from_g,goal_config)
                break

            #If point_drop can only connect to start tree but not connect to goal point
            elif (dist_s <= epsilon and dist_g > epsilon):
                point_addon_s = tree.AddVertex(point_chosen_from_s)
                tree.AddEdge(index,point_addon_s)
                #self.planning_env.PlotEdge(near_point,point_chosen_from_s)

            #If point_drop can connect either the start tree or the goal point
            elif (dist_s > epsilon and dist_g > epsilon):
                point_addon_s = tree.AddVertex(point_chosen_from_s)
                tree.AddEdge(index,point_addon_s)
                #self.planning_env.PlotEdge(near_point,point_chosen_from_s)

        # total_time = time.time() - start_time;
        # Find the path        
        path_start_tree = self.find_path(tree, 0, point_addon_g_final)
        path_start_tree.reverse()
        for i in path_start_tree:
            plan.append(i)
        plan.append(goal_config)

        dist_plan = self.planning_env.ComputePathLength(plan)
        print "total plan distance"
        print dist_plan
        
        print "total vertices in tree "
        print len(tree.vertices)
        # end of implement
        # print "total plan time = "
        # print total_time
        # print " "
        return plan


    def find_path(self, tree, start_id, end_id): #[start_id, end_id)
        id_next_v = end_id
        path = []
        while(id_next_v != start_id):
            id_next_v = tree.edges[id_next_v]
            path.append(tree.vertices[id_next_v])
        return path

    # def ComputeNodeQuality(self, config, C_max):

    #     curr_path = self.find_path(tree,0,index)
    #     curr_cost = self.planning_env.ComputePathLength(curr_path)
    #     curr_h = self.planning_env.ComputeHeuristicC(config,goal_config)
    #     C_q = curr_cost + curr_h
    #     C_max =  max(C_q, C_max)
    #     C_opt = self.planning_env.ComputeDistanceC(start_config, goal_config)
    #     mq = 1 - (C_q - C_opt)/(C_max - C_opt)

    #     return mq, C_max

