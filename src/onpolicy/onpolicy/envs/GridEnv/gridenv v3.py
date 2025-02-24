from PIL import Image
import numpy as np
import copy
import math
import time
from .window import Window
# from window import Window
import gym
import sys
from .Astar import AStar
# from Astar import AStar
import random
import os
import matplotlib.pyplot as plt
from .utils import *
from .nodemanage import *
from .parameter import *
from .quads import *

class GridEnv(gym.Env):
    def __init__(self, resolution, sensor_range, num_agents, max_steps, 
        use_merge = True,
        use_same_location = True,
        use_complete_reward = True,
        use_multiroom = False,
        use_time_penalty = False,
        use_single_reward = False,
        visualization = False):

        self.num_agents = num_agents
    
        # map_img = Image.open(map_file)
        # self.gt_map = np.array(map_img)
        # self.inflation_map = obstacle_inflation(self.gt_map, 0.15, 0.05)
        self.resolution = resolution
        self.call_id = 0  # 初始化调用计数器
        self.sensor_range = sensor_range
        self.pos_traj = []
        self.grid_traj = []
        self.map_per_frame = []
        self.step_time = 0.1
        self.dw_time = 1
        self.minDis2Frontier = 2*self.resolution
        self.frontiers = []
        self.path_log = []
        for e in range(self.num_agents):
            self.path_log.append([])
        self.built_map = []

        # self.width = self.gt_map.shape[1]
        # self.height = self.gt_map.shape[0]
        
        
        self.resize_width = 64
        self.resize_height = 64
        
        ## graph construction
        self.node_coords=None
        self.current_index=None
        self.adjacent_matrix=None
        self.current_index=None
        self.neighbor_indices=None
        self.utility=None
        self.guidpost=None
        self.cell_size = CELL_SIZE
        self.node_resolution = NODE_RESOLUTION 
        self.updating_map_size = UPDATING_MAP_SIZE
        self.node_manager = NodeManager()
        #self.node_manager = NodeManager(plot=self.plot)
        self.map_info = None
        self.updating_map_info=None
        self.accumulated_map= None
        
        ##
        self.robot_discrete_dir = [i*math.pi for i in range(16)]
        self.agent_view_size = int(sensor_range/self.resolution)
        self.target_ratio = 0.98 
        self.merge_ratio = 0
        self.merge_reward = 0
        self.agent_reward = np.zeros((num_agents))
        self.agent_ratio_step = np.ones((num_agents)) * max_steps
        self.merge_ratio_step = max_steps
        self.max_steps = max_steps
        # self.total_cell_size = np.sum((self.gt_map != 205).astype(int))
        
        self.use_same_location = use_same_location
        self.use_complete_reward = use_complete_reward
        self.use_multiroom = use_multiroom
        self.use_time_penalty = use_time_penalty
        self.use_merge = use_merge
        self.use_single_reward = use_single_reward
        # define space
        self.action_space = [gym.spaces.Box(low=0.0, high=1.0, shape=(2,), dtype=np.float32) for _ in range(self.num_agents)]
        
        # Observations are dictionaries containing an
        # encoding of the grid and a textual 'mission' string
        global_observation_space = {}
        global_observation_space['global_obs'] = gym.spaces.Box(
            low=0, high=255, shape=(4, self.resize_width, self.resize_height), dtype='uint8')
        
        #global_observation_space['global_merge_goal'] = gym.spaces.Box(
            #low=0, high=255, shape=(2, self.width, self.height), dtype='uint8')

        # global_observation_space['image'] = gym.spaces.Box(
        #     low=0, high=255, shape=(self.resize_width, self.resize_height, 3), dtype='uint8')

        # global_observation_space['vector'] = gym.spaces.Box(
        #     low=-1, high=1, shape=(self.num_agents,), dtype='float')
        global_observation_space['vector'] = gym.spaces.Box(
            low=-1, high=1, shape=(2,), dtype='float')
        if use_merge:
            global_observation_space['global_merge_obs'] = gym.spaces.Box(
                low=0, high=255, shape=(4, self.resize_width, self.resize_height), dtype='uint8')
            # global_observation_space['global_direction'] = gym.spaces.Box(
            #     low=-1, high=1, shape=(self.num_agents, 4), dtype='float')
        # else:
        #     global_observation_space['global_direction'] = gym.spaces.Box(
        #         low=-1, high=1, shape=(1, 4), dtype='float')
        share_global_observation_space = global_observation_space.copy()
        # share_global_observation_space['gt_map'] = gym.spaces.Box(
        #     low=0, high=255, shape=(1, self.width, self.height), dtype='uint8')
        
        global_observation_space = gym.spaces.Dict(global_observation_space)
        share_global_observation_space = gym.spaces.Dict(share_global_observation_space)

        self.observation_space = []
        self.share_observation_space = []

        for agent_id in range(self.num_agents):
            self.observation_space.append(global_observation_space)
            self.share_observation_space.append(share_global_observation_space)

        self.visualization = visualization
        if self.visualization:
            self.window = Window('map')
            self.window.show(block=False)

        # self.visualize_map = np.zeros((self.width, self.height))
        self.visualize_goal = [[0,0] for i in range(self.num_agents)]
       
    def reset(self):
        # 1. read from blueprints files randomly
        map_file = random.choice(os.listdir('/home/chenyue001/Khattiya-Explore-Bench/src/onpolicy/onpolicy/envs/GridEnv/datasets'))
        map_img = Image.open(os.path.join('/home/chenyue001/Khattiya-Explore-Bench/src/onpolicy/onpolicy/envs/GridEnv/datasets', map_file))
        # map_img = Image.open('/home/nics/workspace/blueprints/room1_modified.pgm')
        
        self.gt_map = np.array(map_img)
        self.inflation_map = obstacle_inflation(self.gt_map, 0.15, 0.05)
        self.width = self.gt_map.shape[1]
        self.height = self.gt_map.shape[0]
        # 初始化accumulated_map为全UNKNOWN，希望构造一个累积的全局地图。
        self.accumulated_map = np.full(self.gt_map.shape, 205, dtype=np.uint8)
        ##
        #print(f"地图的宽度是: {self.width}")
        #print(f"地图的高度是: {self.height}")
        
        self.total_cell_size = np.sum((self.gt_map != 205).astype(int))
        self.visualize_map = np.zeros((self.width, self.height))

        self.num_step = 0
        obs = []
        self.built_map = []

        # reset robot pos and dir
        # self.agent_pos = [self.continuous_to_discrete([-8,8])]
        # self.agent_dir = [0]
        self.agent_pos = []
        self.agent_init_pos=[]
        self.agent_dir = []
        #print(f"[Process {os.getpid()}] Before resetting: self.agent_pos = {self.agent_pos}")
        ## agent_pos 是网格坐标，代表机器人的实时位置
        for i in range(self.num_agents):
            random_at_obstacle_or_unknown = True
            while(random_at_obstacle_or_unknown):
                x = random.randint(0, self.width - 1)
                y = random.randint(0, self.height - 1)
                if self.gt_map[x][y] == 254:     # free space
                    self.agent_pos.append([x, y])
                    self.agent_init_pos.append([x, y])
                ##将机器人信念地图的原点设置为gt map中心
                    self.belief_origin_x, self.belief_origin_y = self.discrete_to_continuous(
                    [self.gt_map.shape[0] / 2, self.gt_map.shape[1] / 2])

                    # self.agent_dir.append(random.randint(0, 15)*math.pi)
                    self.agent_dir.append(random.randint(0, 3))
                    random_at_obstacle_or_unknown = False
                    #print("the initial point is:{}".format(self.agent_pos))
        # init local map
        self.explored_each_map = []
        self.obstacle_each_map = []
        #self.free_each_map=[]
        self.previous_explored_each_map = []
        current_agent_pos = []

        for i in range(self.num_agents):
            self.explored_each_map.append(np.zeros((self.width, self.height)))
            self.obstacle_each_map.append(np.zeros((self.width, self.height)))
            #self.free_each_map.append(np.zeros((self.width, self.height)))
            self.previous_explored_each_map.append(np.zeros((self.width, self.height)))

        for i in range(self.num_agents):
            #print(f"[reset function,Before optimized_build_map] self.agent_pos[i]={self.agent_pos[i]}")
            #print(f"[reset function,Before optimized_build_map] discrete_to_continuous={self.discrete_to_continuous(self.agent_pos[i])}")
            _, map_this_frame, _, _, _, _= self.optimized_build_map(self.discrete_to_continuous(self.agent_pos[i]), 0, self.gt_map, self.resolution, self.sensor_range)
            
             # 检查 partial_map 的数据类型和维度
            #print(f"[Agent {i}] partial_map type: {type(partial_map)}")
            #print(f"[Agent {i}] partial_map unique values: {np.unique(partial_map)}")
            #if isinstance(partial_map, np.ndarray):
              #print(f"[Agent {i}] partial_map shape: {partial_map.shape}, dtype: {partial_map.dtype}")
            #else:
             # print(f"[Agent {i}] partial_map is not a numpy array. Check data source.")
            
            # 验证 map_this_frame 是否包含空闲区域         
            #print(f"[Agent {i}] map_this_frame unique values: {np.unique(map_this_frame)}")
            #if 254 in np.unique(map_this_frame):
               #print(f"[Agent {i}] map_this_frame contains free space.")
            #else:
               #print(f"[Agent {i}] map_this_frame does NOT contain free space.")
        # unknown: 205   free: 254   occupied: 0
            self.built_map.append(map_this_frame) ## map_this_frame=scale_map
            
        ##形成各自的初始观测
            obs.append(map_this_frame)
            current_agent_pos.append(self.agent_pos[i])       
            self.explored_each_map[i] = (map_this_frame != 205).astype(int)
            self.obstacle_each_map[i] = (map_this_frame == 0).astype(int)
            #self.free_each_map[i]=(map_this_frame == 254).astype(int)
                       
        explored_all_map = np.zeros((self.width, self.height))
        obstacle_all_map = np.zeros((self.width, self.height))
        free_all_map=np.zeros((self.width, self.height))
        self.previous_all_map = np.zeros((self.width, self.height))
        for i in range(self.num_agents):
            explored_all_map += self.explored_each_map[i]
            obstacle_all_map += self.obstacle_each_map[i]
            #free_all_map +=self.free_each_map[i]    
        explored_all_map = (explored_all_map > 0).astype(int)
        obstacle_all_map = (obstacle_all_map > 0).astype(int)
        #free_all_map=(free_all_map >0).astype(int)

        # if we have both explored map and obstacle map, we can merge them to get complete map
        # obstacle: 2   free: 1   unknown: 0
        temp = explored_all_map + obstacle_all_map
        self.complete_map = np.zeros(temp.shape)
        self.complete_map[temp == 2] = 0
        self.complete_map[temp == 1] = 254
        self.complete_map[temp == 0] = 205
 
        info = {}
        info['explored_all_map'] = np.array(explored_all_map)
        info['current_agent_pos'] = np.array(current_agent_pos)
        info['free_all_map']=np.array(free_all_map)
        info['explored_each_map'] = np.array(self.explored_each_map)
        info['obstacle_all_map'] = np.array(obstacle_all_map)
        info['obstacle_each_map'] = np.array(self.obstacle_each_map)
        info['agent_direction'] = np.array(self.agent_dir)
        # info['agent_local_map'] = self.agent_local_map

        info['merge_explored_ratio'] = self.merge_ratio
        info['merge_explored_reward'] = self.merge_reward
        info['agent_explored_reward'] = self.agent_reward
        info['merge_ratio_step'] = self.merge_ratio_step
        for i in range(self.num_agents):
            info["agent{}_ratio_step".format(i)] = self.agent_ratio_step[i]
        
        
        self.merge_ratio = 0
        self.merge_reward = 0
        self.agent_reward = np.zeros((self.num_agents))
        self.agent_ratio_step = np.ones((self.num_agents)) * self.max_steps
        self.merge_ratio_step = self.max_steps

        obs = np.array(obs)
        #sys.stdout.close()
        #sys.stdout = sys.__stdout__
        if self.visualization:
            ##
            # self.window.show_img(self.built_map[0])
            self.window.show_img(self.complete_map)
            #print("Shape of complete_map:", self.complete_map.shape)
        #print(f"[Process {os.getpid()}] After resetting: self.agent_pos = {self.agent_pos}")
        
        
        ##1. 不是在reset函数中把地图信息传进planning state，而是在step函数中传入
        ##2. mask map是局部地图信息
        #for i in range(self.num_agents):
            #self.robot_belief=MapInfo(partial_map, mask_map_origin_x, mask_map_origin_y, self.cell_size)
            #print(f"Agent {i}, map shape:self.robot_belief.map.shape:{self.robot_belief.map.shape}")
            #print(f"Agent {i}, map data: {self.robot_belief.map}")
            ##robot_belief 就是**“机器人内部对真实环境的地图估计”**，随着不断探索和传感器观测而更新，目标是逐渐逼近真实世界
            #self.update_planning_state(self.robot_belief, self.discrete_to_continuous(self.agent_pos[i]))    
        return obs, info
    
    def step(self, action):
        #sys.stdout = open('/home/chenyue001/Khattiya-Explore-Bench/src/onpolicy/onpolicy/envs/GridEnv/test.csv', 'a')
        obs = []
        flag = False
        self.explored_each_map_t = []
        self.obstacle_each_map_t = []
        #self.free_each_map_t=[]
        
        current_agent_pos = []
        each_agent_rewards = []
        self.num_step += 1
        reward_obstacle_each_map = np.zeros((self.num_agents, self.width, self.height))
        delta_reward_each_map = np.zeros((self.num_agents, self.width, self.height))
        reward_explored_each_map = np.zeros((self.num_agents, self.width, self.height))
        explored_all_map = np.zeros((self.width, self.height))
        obstacle_all_map = np.zeros((self.width, self.height))
        for i in range(self.num_agents):
            self.explored_each_map_t.append(np.zeros((self.width, self.height)))
            self.obstacle_each_map_t.append(np.zeros((self.width, self.height)))
        agent_goals = []  # 保存所有代理的目标点
        
        ##构建可维护的机器人全局地图
        for i in range(self.num_agents):
            partial_map, _ ,_ ,_, _, _ = self.optimized_build_map(self.discrete_to_continuous(self.agent_pos[i]), 0, self.gt_map, self.resolution, self.sensor_range)
            _, _ ,_ ,_, mask_map_origin_x, mask_map_origin_y = self.optimized_build_map(self.discrete_to_continuous(self.agent_pos[i]), 0, self.gt_map, self.resolution, self.sensor_range)
            self.accumulated_map = self.fuse_map(self.accumulated_map, partial_map, self.gt_map, self.discrete_to_continuous(self.agent_pos[i]), self.resolution, self.sensor_range)
            #print(f'Agent {i} partial_map: {partial_map}')
            #print(f'Agent {i} accumulated_map: {self.accumulated_map}')
            
            #free_indices = np.where(self.accumulated_map == 254)
            #print("free 区域的索引:", free_indices)
            
            #print(f'Agent{i} mask_map_origin_x and y is:{mask_map_origin_x},{mask_map_origin_y}')
            #print(f'Agent{i} accumulated map shape:{self.accumulated_map.shape}')
            #print(f'Agent{i} mask_map shape is{partial_map.shape}')
            self.robot_belief= MapInfo(self.accumulated_map, self.belief_origin_x, self.belief_origin_y, self.cell_size)
            self.update_planning_state(self.robot_belief, self.discrete_to_continuous(self.agent_pos[i]))
            
        for i in range(self.num_agents): 
            robotGoal = action[i]
            agent_goals.append(list(robotGoal))  # 将目标位置保存到 agent_goals 中
            current_agent_pos.append(self.agent_pos[i])
        
            #print(f"Agent {i}: Action {robotGoal}, Current Position {self.agent_pos[i]}")
            
            if robotGoal[0] == self.agent_pos[i][0] and robotGoal[1] == self.agent_pos[i][1]:
                print("finish exploration")
                flag = True
                pass
            else:
                if self.gt_map[robotGoal[0], robotGoal[1]] == 254 and self.gt_map[self.agent_pos[i][0], self.agent_pos[i][1]] == 254:#判断当前位置和目标点位置是否为自由空间
                    global_plan = self.Astar_global_planner(self.agent_pos[i], robotGoal)   
                    pose = self.naive_local_planner(global_plan)
                    self.path_log[0].extend(pose)
                    self.agent_pos[i] = pose[-1][0]
                    self.agent_dir[i] = pose[-1][1]
                    self.agent_dir[i] = random.randint(0, 3)
                    self.build_map_given_path_for_multi_robot(pose, i)
                else:
                    print("Choose a non-free frontier")
        ##这个逻辑段之后，agent已经到到达短期目标点
        
        for i in range(self.num_agents): 
            # _, map_this_frame, _, _ = self.optimized_build_map(self.discrete_to_continuous(self.agent_pos[i]), 0, self.gt_map, self.resolution, self.sensor_range)
            # unknown: 205   free: 254   occupied: 0
            obs.append(self.built_map[i])
            #print(f"Agent {i} built_map shape:{self.built_map[i].shape}")
            ##实时地图更新部分
            self.explored_each_map_t[i] = (self.built_map[i] != 205).astype(int)
            self.obstacle_each_map_t[i] = (self.built_map[i] == 0).astype(int)
            #self.free_each_map_t[i]=(self.built_map[i]==254).astype(int)
           
        self.visualization_map(current_agent_pos, agent_goals, self.global_goals,explored_maps=self.explored_each_map)  # 每个智能体的已探索区域
        frontiers, _ = self.frontiers_detection()
        #self.visualize_graph(frontiers, current_agent_pos, self.node_coords, self.utility, self.guidpost, self.map_info)
        ##
        #print("Length of agent_goals:", len(agent_goals))
        #print("Length of agent_pos:",len(current_agent_pos))
        for i in range(self.num_agents):
            self.explored_each_map[i] = np.maximum(self.explored_each_map[i], self.explored_each_map_t[i])
            self.obstacle_each_map[i] = np.maximum(self.obstacle_each_map[i], self.obstacle_each_map_t[i])
            #self.free_each_map[i] = np.maximum(self.obstacle_each_map[i], self.free_each_map_t[i])
            
            reward_explored_each_map[i] = self.explored_each_map[i].copy()
            reward_explored_each_map[i][reward_explored_each_map[i] != 0] = 1
            
            reward_previous_explored_each_map = self.previous_explored_each_map[i].copy()
            reward_previous_explored_each_map[reward_previous_explored_each_map != 0] = 1

            # reward_obstacle_each_map[i] = self.obstacle_each_map[i].copy()
            # reward_obstacle_each_map[i][reward_obstacle_each_map[i] != 0] = 1

            delta_reward_each_map[i] = reward_explored_each_map[i]
            
            each_agent_rewards.append((np.array(delta_reward_each_map[i]) - np.array(reward_previous_explored_each_map)).sum())
            self.previous_explored_each_map[i] = self.explored_each_map[i]
        
        for i in range(self.num_agents):
            explored_all_map = np.maximum(self.explored_each_map[i], explored_all_map)
            obstacle_all_map = np.maximum(self.obstacle_each_map[i], obstacle_all_map)

        temp = explored_all_map + obstacle_all_map
        ## 
        self.complete_map = np.zeros(temp.shape)
        self.complete_map[temp == 2] = 0
        self.complete_map[temp == 1] = 254
        self.complete_map[temp == 0] = 205

        reward_explored_all_map = explored_all_map.copy()
        reward_explored_all_map[reward_explored_all_map != 0] = 1

        delta_reward_all_map = reward_explored_all_map

        reward_previous_all_map = self.previous_all_map.copy()
        reward_previous_all_map[reward_previous_all_map != 0] = 1

        merge_explored_reward = (np.array(delta_reward_all_map) - np.array(reward_previous_all_map)).sum()
        self.previous_all_map = explored_all_map
        ##将 current_agent_pos 放到 info 字典里返回给训练脚本或可视化模块
        info = {}
        info['explored_all_map'] = np.array(explored_all_map)
        info['current_agent_pos'] = np.array(current_agent_pos)
        info['explored_each_map'] = np.array(self.explored_each_map)
        info['obstacle_all_map'] = np.array(obstacle_all_map)
        info['obstacle_each_map'] = np.array(self.obstacle_each_map)
        info['agent_direction'] = np.array(self.agent_dir)
        # info['agent_local_map'] = self.agent_local_map
        if self.use_time_penalty:
            info['agent_explored_reward'] = np.array(each_agent_rewards) * 0.02 - 0.01
            info['merge_explored_reward'] = merge_explored_reward * 0.02 - 0.01
        else:
            info['agent_explored_reward'] = np.array(each_agent_rewards) * 0.02
            info['merge_explored_reward'] = merge_explored_reward * 0.02
        done = False
        if delta_reward_all_map.sum() / self.total_cell_size >= self.target_ratio or flag:#(self.width * self.height)
            done = True  
            # save trajectory for visualization
            # import pickle
            # traj_file = open('rl_traj.pickle', 'w')
            # pickle.dump(self.path_log, traj_file) 
            # print("save successfully")    
            self.merge_ratio_step = self.num_step
            if self.use_complete_reward:
                info['merge_explored_reward'] += 0.1 * (delta_reward_all_map.sum() / self.total_cell_size)     
                
        for i in range(self.num_agents):
            if delta_reward_each_map[i].sum() / self.total_cell_size >= self.target_ratio:#(self.width * self.height)
                self.agent_ratio_step[i] = self.num_step
                # if self.use_complete_reward:
                #     info['agent_explored_reward'][i] += 0.1 * (reward_explored_each_map[i].sum() / (self.width * self.height))
        
        self.agent_reward = info['agent_explored_reward']
        self.merge_reward = info['merge_explored_reward']
        self.merge_ratio = delta_reward_all_map.sum() / self.total_cell_size #(self.width * self.height)
        info['merge_explored_ratio'] = self.merge_ratio
        info['merge_ratio_step'] = self.merge_ratio_step
        for i in range(self.num_agents):
            info["agent{}_ratio_step".format(i)] = self.agent_ratio_step[i]

        dones = np.array([done for agent_id in range(self.num_agents)])
        if self.use_single_reward:
            rewards = 0.3 * np.expand_dims(info['agent_explored_reward'], axis=1) + 0.7 * np.expand_dims(np.array([info['merge_explored_reward'] for _ in range(self.num_agents)]), axis=1)
        else:
            rewards = np.expand_dims(np.array([info['merge_explored_reward'] for _ in range(self.num_agents)]), axis=1)

        obs = np.array(obs)
        ##
        #self.plot_map_with_path()
        ## graph construction
            
        
        return obs, rewards, dones, info

    def get_short_term_goal(self, data):
        #sys.stdout = open('/home/chenyue001/Khattiya-Explore-Bench/src/onpolicy/onpolicy/envs/GridEnv/test.csv', 'a')
        print("Entering get_short_term_goal method")
        map_goal = []
        self.global_goals=[]
        for e in range(self.num_agents):
           
            ##
            goal_x = data['global_goal'][e][0]
            goal_y = data['global_goal'][e][1]
            goal = [int(self.width * goal_x), int(self.height * goal_y)]
            
             # 后处理：检查并调整 goal
            #if self.built_map[e][goal[0], goal[1]] != 254:
              #print("Global goal in obstacle or unknown region. Adjusting...")
            #for x in range(-1, 2):
                #for y in range(-1, 2):
                    #nx, ny = goal[0] + x, goal[1] + y
                    #if 0 <= nx < self.built_map[e].shape[0] and 0 <= ny < self.built_map[e].shape[1]:  # 添加边界检查
                     #if self.built_map[e][nx, ny] == 254:
                        #goal = [nx, ny]
                        #break
                #else:
                    #continue
                #break
            #if self.built_map[e][goal[0], goal[1]] != 254:
                #goal = self.agent_pos[e]
                #print("No valid goal found, fallback to agent's current position.")
            
            
            #print(f"Agent {e}: Global Goal = {goal}")
            self.global_goals.append(goal)  # 将全局目标添加到列表中
            ##
            
            self.visualize_goal[e] = goal
            
            occupancy_grid = data['global_obs'][e, 0] + data['global_obs'][e, 1]
            # obstacle: 2  unknown: 0   free: 1
            frs, _ = self.detect_frontiers(occupancy_grid)
            #print(f"Agent {e} Frontiers: {frs}")
            # cluster targets into different groups and find the center of each group.
            target_process = copy.deepcopy(frs)
            cluster_center = []
            infoGain_cluster = []
            # path = []
            # currentLoc = self.continuous_to_discrete(self.robot.pos)
            # path.append(currentLoc)
            while(len(target_process) > 0):
                target_cluster = []
                target_cluster.append(target_process.pop())
                
                ##
                #print(f"Agent {e} Target Cluster Center: {target_cluster[0]}")
                
                condition = True
                while(condition):
                    condition = False
                    size_target_process = len(target_process)
                    for i in reversed(range(size_target_process)):
                        for j in range(len(target_cluster)):
                            dis = abs(target_process[i][0] - target_cluster[j][0]) +  abs(target_process[i][1] - target_cluster[j][1])
                            if dis < 3:
                                target_cluster.append(target_process[i])
                                del target_process[i]
                                condition = True
                                break

                center_ = [0, 0]
                num_ = len(target_cluster)
                for i in range(num_):
                    center_[0] += target_cluster[i][0]
                    center_[1] += target_cluster[i][1]

                center_float = [float(center_[0])/float(num_), float(center_[1])/float(num_)]
                min_dis_ = 100.0
                min_idx_ = 10000
                for i in range(num_):
                    temp_dis_ = abs(center_float[0]-float(target_cluster[i][0])) + abs(center_float[1]-float(target_cluster[i][1]))
                    if temp_dis_ < min_dis_:
                        min_dis_ = temp_dis_
                        min_idx_ = i

                cluster_center.append([target_cluster[min_idx_][0], target_cluster[min_idx_][1]])
                infoGain_cluster.append(num_)
            free_cluster_center = []
            for i in range(len(cluster_center)):
                # find the nearest free grid
                for x in range(3):
                    for y in range(3):
                        if self.built_map[e][cluster_center[i][0]-1+x, cluster_center[i][1]-1+y] == 254:
                            free_cluster_center.append([cluster_center[i][0]-1+x, cluster_center[i][1]-1+y])
                            break
                    else:
                        continue
                    break
            if len(free_cluster_center) == 0:
                map_goal.append(self.agent_pos[e])
                print("cannot detect valid frontiers")
            else:
            # choose the frontier which is closest to the goal
                min_dis = 10000
                min_idx = -1
                for idx, fr in enumerate(free_cluster_center):
                    dis = math.sqrt(math.hypot(fr[0]-goal[0], fr[1]-goal[1]))
                    ##
                    
                    #print("the goal[0] is {}".format(goal[0]))
                    #print("the goal[1] is {}".format(goal[1]))
                    
                    if dis < min_dis:
                        min_dis = dis
                        min_idx = idx
                map_goal.append(free_cluster_center[min_idx])
                
        if self.visualization:
            self.visualize_map = copy.deepcopy(self.complete_map)
            for pt in self.visualize_goal:
                if pt[0] > 0 and pt[0] < 299 and pt[1] > 0 and pt[1] < 299:
                    ##
                    self.visualize_map[pt[0], pt[1]] = 128
                    self.visualize_map[pt[0]-1, pt[1]] = 128
                    self.visualize_map[pt[0]+1, pt[1]] = 128
                    self.visualize_map[pt[0], pt[1]-1] = 128
                    self.visualize_map[pt[0]-1, pt[1]-1] = 128
                    self.visualize_map[pt[0]+1, pt[1]-1] = 128
                    self.visualize_map[pt[0], pt[1]+1] = 128
                    self.visualize_map[pt[0]-1, pt[1]+1] = 128
                    self.visualize_map[pt[0]+1, pt[1]+1] = 128
                    
                    # 这里增加扩展区域
                    self.visualize_map[pt[0]-2, pt[1]] = 128
                    self.visualize_map[pt[0]+2, pt[1]] = 128
                    self.visualize_map[pt[0], pt[1]-2] = 128
                    self.visualize_map[pt[0], pt[1]+2] = 128
                    self.visualize_map[pt[0]-2, pt[1]-2] = 128
                    self.visualize_map[pt[0]+2, pt[1]-2] = 128
                    self.visualize_map[pt[0]-2, pt[1]+2] = 128
                    self.visualize_map[pt[0]+2, pt[1]+2] = 128
                else:
                    self.visualize_map[pt[0], pt[1]] = 128
            ##
            for goal in map_goal:
                    self.visualize_map[goal[0], goal[1]] = 64  # 高亮显示
            ##
            self.window.show_img(self.visualize_map)
            #
        #sys.stdout.close()
        #sys.stdout = sys.__stdout__
        ##
      
        return np.array(map_goal)

    def detect_frontiers(self, explored_map):
        '''
        detect frontiers from current built map
        '''
        obstacles = []
        frontiers = []
        height = explored_map.shape[0]
        width = explored_map.shape[1]
        for i in range(2, height-2):
            for j in range(2, width-2):
                if explored_map[i][j] == 2:
                    obstacles.append([i,j])
                elif explored_map[i][j] == 0:
                    numFree = 0
                    temp1 = 0
                    if explored_map[i+1][j] == 1:
                        temp1 += 1 if explored_map[i+2][j] == 1 else 0
                        temp1 += 1 if explored_map[i+1][j+1] == 1 else 0
                        temp1 += 1 if explored_map[i+1][j-1] == 1 else 0
                        numFree += (temp1 > 0)
                    if explored_map[i][j+1] == 1:
                        temp1 += 1 if explored_map[i][j+2] == 1 else 0
                        temp1 += 1 if explored_map[i+1][j+1] == 1 else 0
                        temp1 += 1 if explored_map[i-1][j+1] == 1 else 0
                        numFree += (temp1 > 0)     
                    if explored_map[i-1][j] == 1:
                        temp1 += 1 if explored_map[i-1][j+1] == 1 else 0
                        temp1 += 1 if explored_map[i-1][j-1] == 1 else 0
                        temp1 += 1 if explored_map[i-2][j] == 1 else 0
                        numFree += (temp1 > 0)
                    if explored_map[i][j-1] == 1:
                        temp1 += 1 if explored_map[i][j-2] == 1 else 0
                        temp1 += 1 if explored_map[i+1][j-1] == 1 else 0
                        temp1 += 1 if explored_map[i-1][j-1] == 1 else 0
                        numFree += (temp1 > 0)     
                    if numFree > 0:
                        frontiers.append([i,j])      
        
        return frontiers, obstacles
        
    def optimized_build_map(self, pos, orn, gt_map, resolution, sensor_range):
        '''
        build map from a specific pos with a omni-directional scan
        pos: [x, y]
        '''
        self.call_id += 1  # 维护一个计数
        #print(f"[PID={os.getpid()}][Call={self.call_id}] optimized_build_map start, pos={pos}")
        #print("Function optimized_build_map is called.", flush=True)
        grid_range = int(sensor_range / resolution)
        height = gt_map.shape[0]
        width = gt_map.shape[1]
        # init_map = np.zeros((width, height))
        #print(f"pos: {pos}")
        #print(f"pos[0]: {pos[0]}, pos[1]: {pos[1]}")
        x, y = int(height/2+pos[0]/resolution), int(width/2+pos[1]/resolution)  ##pos[i]应该是一个具体的值而不是列表
        x_min, y_min = max(0, x-grid_range), max(0, y-grid_range)
        x_max, y_max = min(x+grid_range, height), min(y+grid_range, width)
        # x, y = int(height/2+pos[0]/resolution-grid_range), int(width/2+pos[1]/resolution-grid_range) # relative to the upper left corner of the picture
        init_map = gt_map[x_min:x_max, y_min:y_max]
        mask_map = copy.deepcopy(init_map)  ##局部地图
        mask_map_origin = [x-x_min, y-y_min]  ##设置坐标原点
        mask_map_origin_x=x-x_min
        mask_map_origin_y=y-y_min
        for j in range(mask_map.shape[1]):
            laser_path = self.optimized_simulate_laser(0, j, init_map, mask_map_origin)
            path_map = self.plot_path(laser_path, mask_map)
            laser_path.reverse()
            laser_path.append([0,j])
            for idx, p in enumerate(laser_path[:-1]):
                if (init_map[p[0],p[1]] == 0 and init_map[laser_path[idx+1][0], laser_path[idx+1][1]] > 0) or (init_map[p[0],p[1]] == 0 and init_map[laser_path[idx+1][0], laser_path[idx+1][1]] == 0 and p[1] != laser_path[idx+1][1]) or (init_map[p[0],p[1]] == 0 and p[1] == mask_map_origin[1] - 1) or (init_map[p[0],p[1]] == 0 and p[1] == mask_map_origin[1]):
                    for pp in laser_path[idx+1:]:
                        mask_map[pp[0],pp[1]] = 205
                    break     
        for j in range(mask_map.shape[1]):
            laser_path = self.optimized_simulate_laser(mask_map.shape[0]-1, j, init_map, mask_map_origin)
            laser_path.reverse()
            laser_path.append([mask_map.shape[0]-1,j])
            for idx, p in enumerate(laser_path[:-1]):
                if (init_map[p[0],p[1]] == 0 and init_map[laser_path[idx+1][0], laser_path[idx+1][1]] > 0) or (init_map[p[0],p[1]] == 0 and init_map[laser_path[idx+1][0], laser_path[idx+1][1]] == 0 and p[1] != laser_path[idx+1][1]) or (init_map[p[0],p[1]] == 0 and p[1] == mask_map_origin[1] - 1) or (init_map[p[0],p[1]] == 0 and p[1] == mask_map_origin[1]):
                    for pp in laser_path[idx+1:]:
                        mask_map[pp[0],pp[1]] = 205
                    break  
        for i in range(mask_map.shape[0]):
            laser_path = self.optimized_simulate_laser(i, 0, init_map, mask_map_origin)
            laser_path.reverse()
            laser_path.append([i,0])
            for idx, p in enumerate(laser_path[:-1]):
                if (init_map[p[0],p[1]] == 0 and init_map[laser_path[idx+1][0], laser_path[idx+1][1]] > 0) or (init_map[p[0],p[1]] == 0 and init_map[laser_path[idx+1][0], laser_path[idx+1][1]] == 0 and p[0] != laser_path[idx+1][0]) or (init_map[p[0],p[1]] == 0 and p[0] == mask_map_origin[0] - 1) or (init_map[p[0],p[1]] == 0 and p[0] == mask_map_origin[0]):
                    for pp in laser_path[idx+1:]:
                        mask_map[pp[0],pp[1]] = 205
                    break      
        for i in range(mask_map.shape[0]):
            laser_path = self.optimized_simulate_laser(i, mask_map.shape[1]-1, init_map, mask_map_origin)
            laser_path.reverse()
            laser_path.append([i,mask_map.shape[1]-1])
            for idx, p in enumerate(laser_path[:-1]):
                if (init_map[p[0],p[1]] == 0 and init_map[laser_path[idx+1][0], laser_path[idx+1][1]] > 0) or (init_map[p[0],p[1]] == 0 and init_map[laser_path[idx+1][0], laser_path[idx+1][1]] == 0 and p[0] != laser_path[idx+1][0]) or (init_map[p[0],p[1]] == 0 and p[0] == mask_map_origin[0] - 1) or (init_map[p[0],p[1]] == 0 and p[0] == mask_map_origin[0]):
                    for pp in laser_path[idx+1:]:
                        mask_map[pp[0],pp[1]] = 205
                    break
        scale_map = np.zeros(gt_map.shape)
        scale_map[:,:] = 205
        scale_map[x_min:x_max, y_min:y_max] = mask_map
        for w in range(3):
            for h in range(3):
                scale_map[x-1+w, y-1+h] = gt_map[x-1+w, y-1+h]
                #print(f"x-1+w={x-1+w}, y-1+h={y-1+h}, gt_map.shape={gt_map.shape}, scale_map.shape={scale_map.shape}")
        # if self.map_info is None:
        #    raise ValueError("self.map_info is not initialized.") 
        return mask_map, scale_map, (x_min, x_max), (y_min, y_max), mask_map_origin_x, mask_map_origin_y 
    
    ##scale map is observation
        
    def fuse_map(self, global_map, local_map, gt_map, pos, resolution, sensor_range):
        '''
        construct a global map by fusing two maps
        '''
        height = gt_map.shape[0]
        width = gt_map.shape[1]
        grid_range = int(sensor_range / resolution)
        x, y = int(height/2+pos[0]/resolution), int(width/2+pos[1]/resolution)
        #print(f'x: {x}, y: {y}')
        x_min, y_min = max(0, x-grid_range), max(0, y-grid_range)
        x_max, y_max = min(x+grid_range, height), min(y+grid_range, width)
        #print(f'x_min: {x_min}, x_max: {x_max}, y_min: {y_min}, y_max: {y_max}')
        for i in range(x_min,x_max):
            for j in range(y_min,y_max):
                local_i=i-x_min
                local_j=j-y_min
                observed_map = local_map[local_i, local_j]
                if observed_map !=205:
                    global_map[i,j]=observed_map
        return global_map
        
        
        
    def optimized_simulate_laser(self, i, j, mask_map, map_origin):
        
        # left upper
        if i + 0.5 - map_origin[0] < -1 and j + 0.5 - map_origin[1] < -1:
            path = self.return_laser_path(i+1, j+1, mask_map, map_origin)
        # left down
        elif i + 0.5 - map_origin[0] > 1 and j + 0.5 - map_origin[1] < -1:
            path = self.return_laser_path(i, j+1, mask_map, map_origin)
        # right upper
        elif i + 0.5 - map_origin[0] < -1 and j + 0.5 - map_origin[1] > 1:
            path = self.return_laser_path(i+1, j, mask_map, map_origin)
        # right down
        elif i + 0.5 - map_origin[0] > 1 and j + 0.5 - map_origin[1] > 1:
            path = self.return_laser_path(i, j, mask_map, map_origin)
        elif i - map_origin[0] == -1 and j - map_origin[1] < -1:
            path = self.return_laser_path(i, j+1, mask_map, map_origin)
        elif i - map_origin[0] == -1 and j - map_origin[1] > 1:
            path = self.return_laser_path(i, j, mask_map, map_origin)
        elif i - map_origin[0] == 0 and j - map_origin[1] < -1:
            path = self.return_laser_path(i+1, j+1, mask_map, map_origin)
        elif i - map_origin[0] == 0 and j - map_origin[1] > 1:
            path = self.return_laser_path(i+1, j, mask_map, map_origin)
        elif i - map_origin[0] < -1 and j - map_origin[1] == -1:
            path = self.return_laser_path(i+1, j, mask_map, map_origin)
        elif i - map_origin[0] > 1 and j - map_origin[1] == -1:
            path = self.return_laser_path(i, j, mask_map, map_origin)
        elif i - map_origin[0] < -1 and j - map_origin[1] == 0:
            path = self.return_laser_path(i+1, j+1, mask_map, map_origin)
        elif i - map_origin[0] > 1 and j - map_origin[1] == 0:
            path = self.return_laser_path(i, j+1, mask_map, map_origin)
        return path
    
    def return_laser_path(self, i, j, map, map_origin):
        '''
        check whether [i, j] pixel is visible from the origin of map
        '''
        laser_path = []
        step_length = max(abs(i - map_origin[0]),
                        abs(j - map_origin[1]))
        path_x = np.linspace(i, map_origin[0], int(step_length)+2)
        path_y = np.linspace(j, map_origin[1], int(step_length)+2)
        for i in range(1, int(step_length)+1):
            # print(int(math.floor(path_x[i])))
            # print(int(math.floor(path_y[i])))
            laser_path.append([int(math.floor(path_x[i])), int(math.floor(path_y[i]))])
        return laser_path

    def build_map_given_path(self, path):
        for pose in path:
            _, map_this_frame, (x_min, x_max), (y_min, y_max), _, _ = self.optimized_build_map(self.discrete_to_continuous(pose[0]), 0, self.gt_map, self.resolution, self.sensor_range)  # can be modified to replace self.gt_map, self.resolution and self.sensor_range
            self.built_map = self.merge_two_map(self.built_map, map_this_frame, [x_min, x_max], [y_min, y_max])
        self.inflation_built_map = obstacle_inflation(self.built_map, 0.15, 0.05)

    def build_map_given_path_for_multi_robot(self, path, agent_id):
        # for pose in path:
        #     _, map_this_frame, (x_min, x_max), (y_min, y_max) = self.optimized_build_map(self.discrete_to_continuous(pose[0]), 0, self.gt_map, self.resolution, self.sensor_range)  # can be modified to replace self.gt_map, self.resolution and self.sensor_range
        #     self.built_map[agent_id] = self.merge_two_map(self.built_map[agent_id], map_this_frame, [x_min, x_max], [y_min, y_max])
        for i in range(int(len(path)/3)):
            ##print(f"[build_map_given_path_for_multi_robot,Before optimized_build_map] discrete_to_continuous={self.discrete_to_continuous(self.agent_pos)}")
            _, map_this_frame, (x_min, x_max), (y_min, y_max), _, _ = self.optimized_build_map(self.discrete_to_continuous(path[i*3+1][0]), 0, self.gt_map, self.resolution, self.sensor_range)  # can be modified to replace self.gt_map, self.resolution and self.sensor_range
            self.built_map[agent_id] = self.merge_two_map(self.built_map[agent_id], map_this_frame, [x_min, x_max], [y_min, y_max])
        ##确保路径中的所有点都可以参与地图构建
        _, map_this_frame, (x_min, x_max), (y_min, y_max), _, _ = self.optimized_build_map(self.discrete_to_continuous(path[-1][0]), 0, self.gt_map, self.resolution, self.sensor_range)  # can be modified to replace self.gt_map, self.resolution and self.sensor_range
        self.built_map[agent_id] = self.merge_two_map(self.built_map[agent_id], map_this_frame, [x_min, x_max], [y_min, y_max])
        # self.inflation_built_map = obstacle_inflation(self.built_map, 0.15, 0.05)

    def merge_two_map(self, map1, map2, x, y):
        '''
        merge two map into one map
        should be accelerated
        '''
        # merge_map = map1 + map2
        # for i in range(merge_map.shape[0]):
        #     for j in range(merge_map.shape[1]):
        #         if merge_map[i][j] == 0 or merge_map[i][j] == 205 or merge_map[i][j] == 254:
        #             merge_map[i][j] = 0
        #         elif merge_map[i][j] == 410:
        #             merge_map[i][j] = 205
        #         elif merge_map[i][j] == 459 or merge_map[i][j] == 508:
        #             merge_map[i][j] = 254
        test_map = map1 + map2
        merge_map = copy.deepcopy(map1)
        for i in range(x[0], x[1]):
            for j in range(y[0], y[1]):
                if test_map[i][j] == 0 or test_map[i][j] == 205 or test_map[i][j] == 254:
                    merge_map[i][j] = 0
                elif test_map[i][j] == 410:
                    merge_map[i][j] = 205
                elif test_map[i][j] == 459 or test_map[i][j] == 508:
                    merge_map[i][j] = 254
        return merge_map
    
    ##世界坐标到网格坐标
    def continuous_to_discrete(self, pos):
        idx_x = int(pos[0] / self.resolution) + int(self.gt_map.shape[0]/2)
        idx_y = int(pos[1] / self.resolution) + int(self.gt_map.shape[1]/2)
        return [idx_x, idx_y]
    ##网格坐标到世界坐标
    def discrete_to_continuous(self, grid_idx):
        ##打印坐标转换输入,处理单体坐标用
        #print(f"[discrete_to_continuous] Input grid_idx: {grid_idx}")
        pos_x = (grid_idx[0] - self.gt_map.shape[0]/2) * self.resolution
        pos_y = (grid_idx[1] - self.gt_map.shape[1]/2) * self.resolution
        ##打印坐标转换结果
        #print(f"[discrete_to_continuous] Output pos_x: {pos_x}, pos_y: {pos_y}")
        return [pos_x, pos_y]

    def naive_local_planner(self, global_plan):
        '''
        Naive local planner
        always move along the global path
        '''
        pose = []
        # add orn to global path
        for idx, pos in enumerate(global_plan):
            if idx == 0:
                pose.append(self.calculate_pose(c_pos=pos, n_pos=global_plan[idx+1]))
            elif idx == len(global_plan)-1:
                pose.append(self.calculate_pose(c_pos=pos, p_pos=global_plan[idx-1]))
            else:
                pose.append(self.calculate_pose(c_pos=pos, p_pos=global_plan[idx-1], n_pos=global_plan[idx+1]))
        return pose

    def calculate_pose(self, p_pos=None, c_pos=None, n_pos=None):
        '''
        For naive local planner only
        p_pos: previous robot's position
        c_pos: current robot's position
        n_pos: next robot's position
        '''
        # n_pos - c_pos
        start_pos2orn = {(-1,-1):3*math.pi/4, (-1,0):math.pi/2, (-1,1):math.pi/4, (0,1):0, (1,1):7*math.pi/4, (1,0):3*math.pi/2, (1,-1):5*math.pi/4, (0,-1):math.pi}
        if not p_pos:
            return [c_pos, start_pos2orn[tuple((np.array(n_pos)-np.array(c_pos)).tolist())]]
        # p_pos - c_pos
        end_pos2orn = {(-1,-1):7*math.pi/4, (-1,0):3*math.pi/2, (-1,1):5*math.pi/4, (0,1):math.pi, (1,1):3*math.pi/4, (1,0):math.pi/2, (1,-1):math.pi/4, (0,-1):0}
        if not n_pos:
            return [c_pos, end_pos2orn[tuple((np.array(p_pos)-np.array(c_pos)).tolist())]]

        # tuple (p_pos - c_pos, n_pos - c_pos)   
        mid_end_pos2orn = {(-1,-1,-1,1):0, (-1,-1,0,1):15*math.pi/8, (-1,-1,1,1):7*math.pi/4, (-1,-1,1,0):13*math.pi/8,(-1,-1,1,-1):3*math.pi/2,
                           (-1,0,0,1):7*math.pi/4, (-1,0,1,1):13*math.pi/8, (-1,0,1,0):3*math.pi/2, (-1,0,1,-1):11*math.pi/8, (-1,0,0,-1):5*math.pi/4,
                           (-1,1,1,1):3*math.pi/2, (-1,1,1,0):11*math.pi/8, (-1,1,1,-1):5*math.pi/4, (-1,1,0,-1):9*math.pi/8, (-1,1,-1,-1):math.pi,
                           (0,1,1,0):5*math.pi/4, (0,1,1,-1):9*math.pi/8, (0,1,0,-1):math.pi, (0,1,-1,-1):7*math.pi/8, (0,1,-1,0):3*math.pi/4,
                           (1,1,1,-1):math.pi, (1,1,0,-1):7*math.pi/8, (1,1,-1,-1):3*math.pi/4, (1,1,-1,0):5*math.pi/8, (1,1,-1,1):math.pi/2,
                           (1,0,0,-1):3*math.pi/4, (1,0,-1,-1):5*math.pi/8, (1,0,-1,0):math.pi/2, (1,0,-1,1):3*math.pi/8, (1,0,0,1):math.pi/4,
                           (1,-1,-1,-1):math.pi/2, (1,-1,-1,0):3*math.pi/8, (1,-1,-1,1):math.pi/4, (1,-1,0,1):math.pi/8, (1,-1,1,1):0,
                           (0,-1,-1,0):math.pi/4, (0,-1,-1,1):math.pi/8, (0,-1,0,1):0, (0,-1,1,1):15*math.pi/8, (0,-1,1,0):7*math.pi/4}
        return [c_pos, mid_end_pos2orn[tuple(np.concatenate([np.array(p_pos)-np.array(c_pos),np.array(n_pos)-np.array(c_pos)]).tolist())]]

    def plot_path(self, path, map):
        vis_map = copy.deepcopy(map)
        for pixel in path:
            vis_map[pixel[0],pixel[1]] = 128
        return vis_map

    def ObstacleCostFunction(self, trajectory):
        for each in trajectory:
            if self.inflation_map[each[0],each[1]] == 0:
                return True
            else:
                pass
        return False
    
    def MapGri4dCostFunction(self, trajectory, global_plan):
        pass

    def Astar_global_planner(self, start, goal):
        # start_pos = self.continuous_to_discrete(start)
        # goal_pos = self.continuous_to_discrete(goal)
        astar = AStar(tuple(start), tuple(goal), self.gt_map, "euclidean")
        # plot = plotting.Plotting(s_start, s_goal)
        path, visited = astar.searching()
        # vis_map = self.plot_path(path)
        # img = Image.fromarray(vis_map.astype('uint8'))
        # img.show()
        # import pdb; pdb.set_trace()
        return list(reversed(path))

    def point_to_path_min_distance(self, point, path):
        dis = []
        for each in path:
            d_each = self.discrete_to_continuous(each)
            dis.append(math.hypot(point[0]-d_each[0], point[1]-d_each[1]))
        return min(dis), dis.index(min(dis))

    def frontiers_detection(self):
        '''
        detect frontiers from current built map
        '''
        
        obstacles = []
        frontiers = []
        if isinstance(self.built_map, list):
          self.built_map = np.array(self.built_map)  # 转换为 NumPy 数组
        height = self.built_map.shape[0]
        width = self.built_map.shape[1]
        for i in range(2, height-2):
            for j in range(2, width-2):
                if self.built_map[i][j] == 0:
                    obstacles.append([i,j])
                elif self.built_map[i][j] == 205:
                    numFree = 0
                    temp1 = 0
                    if self.built_map[i+1][j] == 254:
                        temp1 += 1 if self.built_map[i+2][j] == 254 else 0
                        temp1 += 1 if self.built_map[i+1][j+1] == 254 else 0
                        temp1 += 1 if self.built_map[i+1][j-1] == 254 else 0
                        numFree += (temp1 > 0)
                    if self.built_map[i][j+1] == 254:
                        temp1 += 1 if self.built_map[i][j+2] == 254 else 0
                        temp1 += 1 if self.built_map[i+1][j+1] == 254 else 0
                        temp1 += 1 if self.built_map[i-1][j+1] == 254 else 0
                        numFree += (temp1 > 0)     
                    if self.built_map[i-1][j] == 254:
                        temp1 += 1 if self.built_map[i-1][j+1] == 254 else 0
                        temp1 += 1 if self.built_map[i-1][j-1] == 254 else 0
                        temp1 += 1 if self.built_map[i-2][j] == 254 else 0
                        numFree += (temp1 > 0)
                    if self.built_map[i][j-1] == 254:
                        temp1 += 1 if self.built_map[i][j-2] == 254 else 0
                        temp1 += 1 if self.built_map[i+1][j-1] == 254 else 0
                        temp1 += 1 if self.built_map[i-1][j-1] == 254 else 0
                        numFree += (temp1 > 0)     
                    if numFree > 0:
                        frontiers.append([i,j])      
        return frontiers, obstacles
    
    def visualize_frontiers(self):
        frs, _ = self.frontiers_detection()
        vis_map = copy.deepcopy(self.built_map)
        for fr in frs:
            vis_map[fr[0], fr[1]] = 128
        return vis_map

    def arrival_detection(self, goal):
        '''
        detect whether robot arrives goal
        goal: discrete idx 
        '''
        goal = self.discrete_to_continuous(goal)       
        print(f"Current goal position (continuous): {goal}")
        dis = math.hypot(self.robot.pos[0]-goal[0], self.robot.pos[1]-goal[1])
        if dis <= self.minDis2Frontier:
            print("Robot has reached the goal!")
            return True
        
        else:
            print("The robot has not reached the goal!")
            return False

    #def Steer(self, x_nearest, x_rand, eta):
        #x_new = []
        #if (self.distance(x_nearest, x_rand) <= eta):
            #x_new = x_rand
        #else:
            #m = (x_rand[1]-x_nearest[1])/(x_rand[0]-x_nearest[0])
            #x_new.append((np.sign(x_rand[0]-x_nearest[0]))*(math.sqrt( (math.pow(eta,2)) / ((math.pow(m,2))+1) )   )+x_nearest[0])
            #x_new.append(m*(x_new[0]-x_nearest[0])+x_nearest[1])   
        #return x_new     

    def ObstacleFree(self, xnear, xnew):
        rez = self.resolution*0.2
        stepz = int(math.ceil(self.distance(xnew,xnear))/rez)
        xi = xnear
        # Free='0', Frontier='2', Obstacle='1'
        # map data:  0 occupied      205 unknown       254 free
        for i in range(stepz):
            xi = self.Steer(xi, xnew, rez)
            if self.mapValue(self.inflation_built_map, xi) == 0:
                return 1
            if self.mapValue(self.inflation_built_map, xi) == 205:
                x_new=xi
                return 2
        return 0

    def mapValue(self, mapData, point):
        point = self.continuous_to_discrete(point)
        return mapData[point[0], point[1]]

    ##可视化
    def visualization_map(self, agent_positions, agent_goals, global_goals, explored_maps=None):
       # 确保 agent_positions 和 agent_goals 都是列表
       agent_positions = [list(pos) for pos in agent_positions]
       agent_goals = [list(goal) for goal in agent_goals]
       global_goals = [list(goal) for goal in global_goals]  # 将 global_goals 也转换为列表
      
       plt.title("Map")
       plt.axis('off')
       plt.clf()  # 清除之前的图形
       plt.imshow(self.gt_map, cmap='gray', interpolation='nearest')  # 显示地图
       
        # 如果传入了 explored_maps，则在地图上绘制已探索的区域
       if explored_maps is not None:
        for i, explored_map in enumerate(explored_maps):
            # 使用 alpha 控制透明度，以便与背景地图叠加  
         plt.imshow(explored_map, cmap='Blues', alpha=0.3, interpolation='nearest')
    # 绘制代理的当前位置和目标位置
       for i in range(self.num_agents):
        try:
            plt.scatter(agent_positions[i][1], agent_positions[i][0], color='red',marker='^', label='Current Position' if i == 0 else "")  # 代理位置
            plt.scatter(agent_goals[i][1], agent_goals[i][0], color='blue', marker='x', label='Target Position' if i == 0 else "")  # 目标位置
            global_goal = self.global_goals[i]
            
            plt.scatter(global_goal[1], global_goal[0], color='green', marker='o', label='Global Goal' if i == 0 else "")
        except Exception as e:
            print("Error while plotting:", e)

    # 添加图例
       handles, labels = plt.gca().get_legend_handles_labels()
       by_label = dict(zip(labels, handles))
       plt.legend(by_label.values(), by_label.keys(), loc='upper right')  # 添加图例
       plt.draw()  # 更新图形
       plt.pause(0.01)
       
    ##guidepost表示已经访问过的点
        
 ##更新地图
    def update_map(self, map_info):
        self.map_info = map_info
##应该传入世界坐标的agent position
    def get_updating_map(self,agent_position):
        ## 根据 agent_position 计算局部地图的范围： 局部地图的原点和终点是通过 agent_position 计算得到的
        print(f"[get_updating_map] agent_position: {agent_position[0]}, {agent_position[1]}")
        updating_map_origin_x=(agent_position[0]-self.updating_map_size/2)
        updating_map_origin_y=(agent_position[1]-self.updating_map_size/2)
        print(f"[get_updating_map] updating_map_origin_x: {updating_map_origin_x}")
        print(f"[get_updating_map] updating_map_origin_y: {updating_map_origin_y}")
        updating_map_top_x=updating_map_origin_x + self.updating_map_size
        updating_map_top_y=updating_map_origin_y + self.updating_map_size
        min_x = self.map_info.map_origin_x - self.cell_size * (self.map_info.map.shape[1] / 2)
        min_y = self.map_info.map_origin_y - self.cell_size * (self.map_info.map.shape[0] / 2)
        max_x = self.map_info.map_origin_x + self.cell_size * (self.map_info.map.shape[1] / 2)
        max_y = self.map_info.map_origin_y + self.cell_size * (self.map_info.map.shape[0] / 2)
        ## 限制剪裁范围在全局地图边界内： 剪裁的原点和终点不能超出全局地图的范围
        if updating_map_origin_x < min_x:
            updating_map_origin_x = min_x
        if updating_map_origin_y < min_y:
            updating_map_origin_y = min_y
        if updating_map_top_x > max_x:
            updating_map_top_x = max_x
        if updating_map_top_y > max_y:
            updating_map_top_y = max_y
        ## 将局部地图的原点和终点对齐到全局地图的网格线上
        updating_map_origin_x = round(updating_map_origin_x / self.cell_size) * self.cell_size
        updating_map_origin_y = round(updating_map_origin_y / self.cell_size) * self.cell_size
        updating_map_top_x = round(updating_map_top_x / self.cell_size) * self.cell_size
        updating_map_top_y = round(updating_map_top_y / self.cell_size) * self.cell_size


        updating_map_origin = np.array([updating_map_origin_x, updating_map_origin_y])
        updating_map_origin_in_global_map = get_cell_position_from_coords(updating_map_origin, self.map_info)
        print(f"[get_updating_map], map_info.map_origin_x: {self.map_info.map_origin_x}")
        print(f"[get_updating_map], map_info.map_origin_y: {self.map_info.map_origin_y}")
        updating_map_top = np.array([updating_map_top_x, updating_map_top_y])
        print(f'[get_updating_map] updating_map_top: {updating_map_top}')
        updating_map_top_in_global_map = get_cell_position_from_coords(updating_map_top, self.map_info)

        y1, x1 = updating_map_origin_in_global_map
        y2, x2 = updating_map_top_in_global_map
        y2 = min(y2, self.map_info.map.shape[0] - 1)
        x2 = min(x2, self.map_info.map.shape[1] - 1)

        updating_map = self.map_info.map[y1:y2, x1:x2]
        
        ##检查地图剪裁是否合理
        print(f"[get_updating_map] updating_map_origin_in_global_map: {updating_map_origin_in_global_map}")
        print(f"[get_updating_map] updating_map_top_in_global_map: {updating_map_top_in_global_map}")
        print(f"[get_updating_map] Extracted map shape: {updating_map.shape}")

        updating_map_info=MapInfo(updating_map, updating_map_origin_x, updating_map_origin_y, self.cell_size)
        ##打印更新地图信息
        print(f"Updating Map Info:")
        print(f"  Origin X: {updating_map_info.map_origin_x}")
        print(f"  Origin Y: {updating_map_info.map_origin_y}")
        #print(f"  Cell Size: {updating_map_info.cell_size}")
        print(f"  Map Shape: {updating_map_info.map.shape}")
        #print(f"  Map Data:\n{updating_map_info.map}")

        return updating_map_info
        
        
    def update_updating_map(self,agent_position):
        #mask_map, scale_map, bounds_x, bounds_y, _, _ = self.optimized_build_map(self.discrete_to_continuous(agent_position), 0, self.gt_map, self.resolution, self.sensor_range)
        self.updating_map_info=self.get_updating_map(agent_position)
                  
    #def update_position(self, agent_position):
        #self.agent_position = agent_position
        #node = self.node_manager.nodes_dict.find(agent_position)
        #if self.node_manager.nodes_dict.__len__() == 0:
            #pass
        #else:
            #node.data.set_visited()
    
    
    def update_frontiers(self):  ##不传参
        #if self.updating_map_info is None or self.updating_map_info.map is None:
         #print(f"[update_frontiers] updating_map_info.map is None!")
         #return
        self.frontiers = self.node_manager.frontiers_detect(self.updating_map_info)
    
    ##更新规划状态
    def update_planning_state(self, global_map_info, agent_position):
        self.update_map(global_map_info)
        #self.update_position(agent_position)
        self.update_updating_map(agent_position)
        self.update_frontiers()
        self.node_manager.update_graph(
            agent_position,
            self.frontiers,
            self.updating_map_info,
            self.map_info
        )
        self.node_coords, self.utility, self.guidepost, self.adjacent_matrix = \
            self.update_graphinfo()
    
    def update_graphinfo(self):
        ##检查节点是否存在
        print(f"Number of nodes in nodes_dict: {len(self.node_manager.nodes_dict)}")
        all_node_coords = []
        for node in self.node_manager.nodes_dict.__iter__():
            all_node_coords.append(node.data.coords)
        all_node_coords = np.array(all_node_coords).reshape(-1, 2)
        
        print(f'all_node_coords: {all_node_coords}')
        
        utility = []
        guidepost = []
        n_nodes = all_node_coords.shape[0]
        adjacent_matrix = np.ones((n_nodes, n_nodes)).astype(int)
        node_coords_to_check = all_node_coords[:, 0] + all_node_coords[:, 1] * 1j
        print(f'node_coords_to_check: {node_coords_to_check}')
        for i, coords in enumerate(all_node_coords):
            node = self.node_manager.nodes_dict.find((coords[0], coords[1])).data
            utility.append(node.utility)
            guidepost.append(node.visited)
            for neighbor in node.neighbor_set:
                index = np.argwhere(node_coords_to_check == neighbor[0] + neighbor[1] * 1j)
                assert index is not None
                index = index[0][0]
                adjacent_matrix[i, index] = 0
        utility = np.array(utility)
        guidepost = np.array(guidepost)
       
        print(f'adjacent_matrix:{adjacent_matrix}')
        for j in range(self.num_agents):
             
         print(f'{j}th agent_position:{self.discrete_to_continuous(self.agent_pos[j])}')
        #current_index = np.argwhere(node_coords_to_check == self.agent_position[0] + self.agent_position[1] * 1j)[0][0]
        #neighbor_indices = np.argwhere(adjacent_matrix[current_index] == 0).reshape(-1)
        return all_node_coords, utility, guidepost, adjacent_matrix, #current_index, neighbor_indices
    
    ##
    #def plot_map_with_path(self):
     #print(f"Window attribute: {self.window}")  # 调试输出
     #vis = copy.deepcopy(self.complete_map)
     #for e in range(self.num_agents):
        #for pose in self.path_log[e]:
            #print(f"Agent {e} Pose: {pose}")  # 添加调试输出
            #if isinstance(pose, list) and len(pose) == 2:  # 检查 pose 是否为列表且长度为2
                #position = pose[0]  # 取出位置部分
                #if isinstance(position, tuple) and len(position) == 2:  # 检查位置部分是否为元组且长度为2
                    #x, y = int(position[0]), int(position[1])
                    #if 0 <= x < vis.shape[0] and 0 <= y < vis.shape[1]:
                        #vis[x, y] = 127
                #else:
                    #print(f"Invalid position format: {position}")  # 输出无效位置格式
            #else:
                #print(f"Invalid pose format: {pose}")  # 输出无效格式

     #self.window.show_img(vis)  # 使用 vis 显示图像

def obstacle_inflation(map, radius, resolution):
    inflation_grid = math.ceil(radius / resolution)
    import copy
    inflation_map = copy.deepcopy(map)
    for i in range(map.shape[0]):
        for j in range(map.shape[1]):
            if map[i][j] == 0:
                neighbor_list = get_neighbor(i, j, inflation_grid, map.shape[0], map.shape[1])
                for inflation_point in neighbor_list:
                    inflation_map[inflation_point[0],inflation_point[1]] = 0
    return inflation_map

def get_neighbor(x, y, radius, x_max, y_max):
    neighbor_list = []
    for i in range(-radius, radius+1):
        for j in range(-radius, radius+1):
            if x+i > -1 and x+i < x_max and y+j > -1 and y+j < y_max:
                neighbor_list.append([x+i,y+j])
    return neighbor_list

##规定地图的属性
class MapInfo:
    def __init__(self, map, map_origin_x, map_origin_y, cell_size):
        self.map = map
        self.map_origin_x = map_origin_x
        self.map_origin_y = map_origin_y
        self.cell_size= cell_size
        self.mask_map = None  # 局部地图的掩码
        self.scale_map = None  # 与全局地图同尺寸的部分更新地图
        self.bounds_x = None  # 局部地图在全局地图中的 x 轴范围 (x_min, x_max)
        self.bounds_y = None  # 局部地图在全局地图中的 y 轴范围 (y_min, y_max)
        
    def update_map_info(self, map, map_origin_x, map_origin_y):
        self.map = map
        self.map_origin_x = map_origin_x
        self.map_origin_y = map_origin_y
        
    def set_mask_map(self, mask_map, scale_map, bounds_x, bounds_y):
        self.mask_map = mask_map
        self.scale_map = scale_map
        self.bounds_x = bounds_x  # (x_min, x_max)
        self.bounds_y = bounds_y  # (y_min, y_max)
    
    #def get_local_map_info(self):
        #if self.mask_map is not None:
            #return MapInfo(
                #map=self.mask_map,
                #map_origin_x=self.map_origin_x + self.bounds_x[0] * self.cell_size,
                #map_origin_y=self.map_origin_y + self.bounds_y[0] * self.cell_size,
                #cell_size=self.cell_size
            #)
        #else:
            #return None






    
